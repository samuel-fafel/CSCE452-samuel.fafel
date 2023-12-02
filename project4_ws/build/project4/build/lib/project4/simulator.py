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
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
from math import sqrt, sin, cos, ceil, floor, nan, atan2
from itertools import combinations
import random

# rm -r junk/outputs; ros2 launch project4 launch.py bag_in:=project4-bags/input2/input2_0.db3 bag_out:=junk/outputs/junk1

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

def bresenham_line(x0, y0, x1, y1, resolution):
    """
    Bresenham's Line Algorithm
    Produces a list of tuples from start and end

    :param x0, y0: The start coordinate
    :param x1, y1: The end coordinate
    :param resolution: The resolution of the grid
    """
    # Convert world coordinates to grid coordinates
    x0, y0 = int(x0 / resolution), int(y0 / resolution)
    x1, y1 = int(x1 / resolution), int(y1 / resolution)

    points = []
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    x, y = x0, y0
    sx = -1 if x0 > x1 else 1
    sy = -1 if y0 > y1 else 1
    err = dx - dy

    while True:
        points.append((x * resolution, y * resolution))
        if x == x1 and y == y1:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x += sx
        if e2 < dx:
            err += dx
            y += sy

    return points

class Simulator(Node):
    def __init__(self):
        # ------- Initialize Parameters -------
        super().__init__('simulator')
        self.model = 'ideal.robot'
        worlds = ('brick.world', 'pillars.world', 'open.world', 'ell.world', 'custom.world')
        self.world = 'ell.world'
        #self.world = worlds[random.randint(0,4)]

        # Parse Robot Parameters
        self.robot_model = self.declare_parameter('robot', self.model).value
        self.robot = load_disc_robot(self.robot_model)
        self.wheel_separation = self.robot['wheels']['distance']
        self.robot_radius = self.robot['body']['radius']
        self.robot_height = self.robot['body']['height']

        # Parse Laser Parameters
        self.laser_rate = self.robot['laser']['rate'] # Publish a new scan every X seconds
        self.laser_count = self.robot['laser']['count'] # Number of measurements in each scan
        self.laser_angle_min = self.robot['laser']['angle_min'] # Minimum angle in radians 
        self.laser_angle_max = self.robot['laser']['angle_max'] # Maximum angle in radians
        self.laser_angle_increment = (self.laser_angle_max - self.laser_angle_min) / (self.laser_count - 1)  # Angular distance between measurements
        self.laser_range_min = self.robot['laser']['range_min'] # Minimum range in meters
        self.laser_range_max = self.robot['laser']['range_max'] # Maximum range in meters
        self.laser_variance = self.robot['laser']['error_variance'] # Error Variance
        self.laser_fail_prob = self.robot['laser']['fail_probability'] # Probability of Laser Failure

        # Parse World File
        self.world = World(self.world)
        self.resolution = self.world.get_resolution()
        self.initial_pose = self.world.get_initial_pose()
        self.dimensions = self.world.get_dimensions()
        self.occupancy_grid = self.world.get_occupancy_grid()
        self.grid2d = self.world.get_grid_2d()

        # Set Inital Pose
        self.x = self.initial_pose[0]
        self.y = self.initial_pose[1]
        self.theta = self.initial_pose[2]

        # Set Wheel velocities
        self.v_left = 0.0
        self.v_right = 0.0

        # ------- Initialize Subscribers/Publishers -------

        # Subscribers for wheel velocities
        self.vl_subscriber = self.create_subscription(Float64, '/vl', self.vl_callback, 10)
        self.vr_subscriber = self.create_subscription(Float64, '/vr', self.vr_callback, 10)

        # Transform publishers
        self.tf_broadcaster = TransformBroadcaster(self)

        # Publisher for Occupancy Grid
        self.map_publisher = self.create_publisher(OccupancyGrid, '/map', 10)

        # Publisher for LaserScan
        self.laser_publisher = self.create_publisher(LaserScan, '/scan', self.laser_rate)

        # ---- Obstacle Computations -------

        # Compute Obstacle Boundaries
        self.obstacle_corners = self.get_obstacle_corners()        

        # ------- Timers -------

        # Publish Map
        #self.create_timer(0.1, self.publish_map)
        self.publish_map()

        # Laser Scan
        self.create_timer(self.laser_rate, self.laserscan)

        # Timer to check for last recieved command
        self.last_cmd_vel_time = Clock().now()
        self.timer = self.create_timer(0.1, self.timer_callback)
  
        # Start the Primary Loop
        self.last_update_time = self.get_clock().now()
        self.create_timer(0.1, self.update_pose)

    # ------- Movement Functions -------
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
        if not self.will_be_in_collision(x, y):
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

    # ------- Collision Detection Functions -------
    def publish_map(self): # Create and Publish Occupancy Grid Message (ogm)
        self.ogm = OccupancyGrid()
        self.ogm.header.stamp = self.get_clock().now().to_msg()
        self.ogm.header.frame_id = 'world'
        self.ogm.info.resolution = self.resolution
        self.ogm.info.width = self.dimensions[0]
        self.ogm.info.height = self.dimensions[1]
        self.ogm.data = self.occupancy_grid
        self.map_publisher.publish(self.ogm)
    
    def get_obstacle_corners(self): # Function to convert the array into a list of obstacle corner points
        # List to hold the coordinates of corners
        corners = []
        cols = self.dimensions[0]
        rows = self.dimensions[1]
        
        # Iterate through the array to find obstacles
        for i in range(rows):
            for j in range(cols):
                # Check if we have an obstacle
                if self.grid2d[i][j] == 100:
                    # Add the coordinates of the corners of the obstacle cell
                    corners.append((round(j * self.resolution, 2), round(i * self.resolution, 2)))  # Bottom Left Corner
                    corners.append((round((j + 1) * self.resolution, 2), round(i * self.resolution,2)))  # Bottom Right Corner
                    corners.append((round(j * self.resolution,2), round((i + 1) * self.resolution,2)))  # Top Left Corner
                    corners.append((round((j + 1) * self.resolution,2), round((i + 1) * self.resolution,2)))  # Top Right Corner
        
        # Remove duplicates by converting the list to a set, then back to a list
        unique_corners = list(set(corners))
        
        # Sort the list of corners
        unique_corners.sort()
        return unique_corners
    
    def will_be_in_collision(self, x, y):
        for corner in self.obstacle_corners:
            if sqrt((x-corner[0])**2 + (y-corner[1])**2) <= self.robot_radius:
                return True
        return False

    # ------- Laser Scan Functions -------
    def simulate_laser_error(self, var): # Simulates laser error
        mean = 0
        std_dev = np.sqrt(var)

        return np.random.normal(mean, std_dev) # Returns random error from Gaussian distribution

    def simulate_laser_failure(self, fail_prob): # Simulates a failed laser reading 
        random_number = random.uniform(0, 1) # Generate a random number between 0 and 1
        if random_number < fail_prob: # Check if the random number is less than the given probability
            return True # The event occurs
        else:
            return False # The event does not occur

    def calculate_scan_distances(self):
        distances = []
        for i in range(self.laser_count):
            angle = self.laser_angle_min + i * self.laser_angle_increment
            distance = self.cast_ray(angle)
            distances.append(distance)
        return distances

    def cast_ray(self, angle):
        # Determine the end point of the ray based on the laser's max range
        ray_end_x = self.x + self.laser_range_max * cos(angle + self.theta)
        ray_end_y = self.y + self.laser_range_max * sin(angle + self.theta)

        # Iterate over points along the ray
        for current_point in bresenham_line(self.x, self.y, ray_end_x, ray_end_y, self.resolution):
            if self.is_occupied(*current_point):
                # Calculate the distance from the robot to this point
                distance = sqrt((current_point[0] - self.x)**2 + (current_point[1] - self.y)**2)
                return distance  # Return the distance to the first occupied point

        return self.robot_radius * 3  # If no obstacle is found, return max range

    def is_occupied(self, x, y):
        grid_x = floor(x / self.resolution)
        grid_y = floor(y / self.resolution)
        if self.grid2d[grid_y][grid_x] == 100:
            return True
        return False
 
    def laserscan(self):
        # Create a LaserScan message
        laser_scan_msg = LaserScan()
        laser_scan_msg.header = Header(stamp=self.get_clock().now().to_msg(), frame_id='laser')
        laser_scan_msg.angle_min = self.laser_angle_min
        laser_scan_msg.angle_max = self.laser_angle_max
        laser_scan_msg.angle_increment = self.laser_angle_increment
        laser_scan_msg.time_increment = self.laser_rate/self.laser_count
        laser_scan_msg.scan_time = float(self.laser_rate)
        laser_scan_msg.range_min = self.laser_range_min
        laser_scan_msg.range_max = self.laser_range_max

        # Calculate Distances for each Laser
        scan_distances = self.calculate_scan_distances()

        # Add errors to values
        for i in range(len(scan_distances)):
            scan_distances[i] = scan_distances[i] + self.simulate_laser_error(self.laser_variance)

        # Take out values outside of value range
        scan_distances = [val for val in scan_distances if laser_scan_msg.range_min <= val <= laser_scan_msg.range_max]

        # Replace values with NaN if failed
        for i in range(len(scan_distances)):
            if (self.simulate_laser_failure(self.laser_fail_prob)):
                scan_distances[i] = nan #math.nan

        # Range and intensity values
        laser_scan_msg.ranges = scan_distances # Find values from function
        laser_scan_msg.intensities = [] # Blank since intensities not required

        self.laser_publisher.publish(laser_scan_msg) # Publish Scan

def main(args=None):
    rclpy.init(args=args)
    simulator = Simulator()
    rclpy.spin(simulator)

    # Destroy the node explicitly
    simulator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()