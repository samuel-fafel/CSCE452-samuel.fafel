import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Float64
from rclpy.clock import Clock
import numpy as np
from project4.disc_robot import *
from project4.load_world import *
from nav_msgs.msg import OccupancyGrid
from tf2_ros import TransformBroadcaster
from math import sin, cos, ceil, floor
import random

def euler_to_quaternion(roll, pitch, yaw):
    """
    Convert an Euler angle to a quaternion.
    
    Input
      :param roll: The roll (rotation around x-axis) angle
      :param pitch: The pitch (rotation around y-axis) angle
      :param yaw: The yaw (rotation around z-axis) angle
    
    Output
      :return qx, qy, qz, qw: The quaternion as a tuple
    """
    qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
    qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
    qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
    qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)

    return qx, qy, qz, qw

class Simulator(Node):
    def __init__(self):
        super().__init__('simulator')
        self.model = 'ideal.robot'
        worlds = ['brick.world', 'pillars.world', 'rectangle.world', 'custom.world']
        self.world = worlds[random.randint(0,3)]

        # Subscribers for wheel velocities
        self.vl_subscriber = self.create_subscription(Float64, '/vl', self.vl_callback, 10)
        self.vr_subscriber = self.create_subscription(Float64, '/vr', self.vr_callback, 10)

        # Transform publishers
        self.tf_broadcaster = TransformBroadcaster(self)

        # Publisher for Occupancy Grid
        self.map_publisher = self.create_publisher(OccupancyGrid, '/map', 10)

        # Robot parameters
        self.robot_model = self.declare_parameter('robot', self.model).value
        self.robot = load_disc_robot(self.robot_model)
        self.wheel_separation = self.robot['wheels']['distance']
        self.robot_radius = self.robot['body']['radius']
        self.robot_height = self.robot['body']['height']

        # Parse Initial Pose and Occupancy Grid
        self.world = World(self.world)
        self.occupancy_grid = self.world.get_occupancy_grid()
        initial_pose = self.world.get_initial_pose()
        self.x = initial_pose[0]
        self.y = initial_pose[1]
        self.theta = initial_pose[2]
        self.ogm = OccupancyGrid()
        self.publish_map()
        
        # Wheel velocities
        self.v_left = 0.0
        self.v_right = 0.0
        
        # Set the rate of pose update and the last update time
        self.rate = self.create_rate(10)  # 10 Hz
        self.last_update_time = self.get_clock().now()

        # Create Command Timer
        self.last_cmd_vel_time = Clock().now()
        self.timer = self.create_timer(0.1, self.timer_callback)  # Check at 10 Hz

        # Start the main loop
        self.create_timer(0.1, self.update_pose)  # 10 Hz

    def publish_map(self):
        # Create and Publish Occupancy Grid Message (ogm)
        self.ogm.header.stamp = self.get_clock().now().to_msg()
        self.ogm.header.frame_id = 'world'

        self.ogm.info.resolution = self.world.get_resolution()
        self.ogm.info.width = self.world.get_dimensions()[0]
        self.ogm.info.height = self.world.get_dimensions()[1]

        self.ogm.data = self.occupancy_grid
        self.map_publisher.publish(self.ogm)

    def timer_callback(self):
        current_time = Clock().now()
        if (current_time - self.last_cmd_vel_time).nanoseconds > 1e9:  # 1 second in nanoseconds
            self.stop_robot()
    
    def stop_robot(self):
        vr_msg = Float64()
        vl_msg = Float64()
        vr_msg.data = 0.0
        vl_msg.data = 0.0
        self.vr_callback(vr_msg)
        self.vl_callback(vl_msg)

    def vl_callback(self, msg):
        self.v_left = msg.data
        self.last_cmd_vel_time = Clock().now()

    def vr_callback(self, msg):
        self.v_right = msg.data
        self.last_cmd_vel_time = Clock().now()

    def update_pose(self):
        x = self.x
        y = self.y
        theta = self.theta

        # Get current time and calculate delta time
        current_time = self.get_clock().now()
        dt = (current_time - self.last_update_time).nanoseconds / 1e9
        self.last_update_time = current_time
        
        # Calculate linear and angular velocities
        v_linear = (self.v_right + self.v_left) / 2.0
        v_angular = (self.v_right - self.v_left) / self.wheel_separation
        
        # Compute the new pose
        delta_x = v_linear * cos(theta) * dt
        delta_y = v_linear * sin(theta) * dt
        delta_theta = v_angular * dt
        
        x += delta_x
        y += delta_y
        theta += delta_theta
        
        # Normalize theta
        theta = (theta + 3.14159265) % (2 * 3.14159265) - 3.14159265
        
        # COLLISION DETECTION
        x_boundary = 0
        y_boundary = 0
        if self.world.get_resolution() > self.robot_radius:
            boundary = self.world.get_resolution() / 2 # prevent actual intersection with boundary
            x_boundary = boundary if delta_x > 0 else -boundary
            y_boundary = boundary if delta_y > 0 else -boundary

        if self.will_be_in_collision(x + x_boundary, y + y_boundary):
            self.stop_robot
        else:
            self.x = x
            self.y = y
            self.theta = theta
        self.publish_pose() # Publish the new pose

    def publish_pose(self):
        # Broadcast the transform from the world frame to the robot's frame
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0  # Assuming flat ground
        q = euler_to_quaternion(0, 0, self.theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        
        self.tf_broadcaster.sendTransform(t)

    def will_be_in_collision(self, new_x, new_y):
        resolution = self.world.get_resolution()

        # Convert robot position to grid coordinates
        grid_x = int(new_x / resolution)
        grid_y = int(new_y / resolution)

        # Determine the cells covered by the robot's footprint
        footprint_cells = self.calculate_footprint_cells(grid_x, grid_y, resolution)

        # Check each cell in the footprint for collision
        for cell in footprint_cells:
            # Check grid bounds
            if cell[0] < 0 or cell[0] >= self.ogm.info.width or cell[1] < 0 or cell[1] >= self.ogm.info.height:
                continue  # Skip cells outside of the grid
            
            # Check occupancy value
            index = cell[1] * self.ogm.info.width + cell[0]
            if self.ogm.data[index] == 100:
                return True  # Collision detected

        return False  # No collision detected

    def calculate_footprint_cells(self, grid_x, grid_y, resolution):
        # This is for a circular robot.
        cells = []
        radius_in_cells = int(self.robot_radius / resolution)
        for x in range(grid_x - radius_in_cells, grid_x + radius_in_cells + 1):
            for y in range(grid_y - radius_in_cells, grid_y + radius_in_cells + 1):
                if (x - grid_x)**2 + (y - grid_y)**2 <= radius_in_cells**2:
                    cells.append((x, y))
        return cells

def main(args=None):
    rclpy.init(args=args)
    simulator = Simulator()
    rclpy.spin(simulator)

    # Destroy the node explicitly
    simulator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()