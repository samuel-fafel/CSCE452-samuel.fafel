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
from math import sqrt, sin, cos, ceil, floor, nan, radians
from itertools import combinations
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

# Define the function to calculate the distance from a point to a line segment
def point_to_line_distance(point, line_segment):
    # Unpack the point and line segment
    (x0, y0) = point
    (x1, y1), (x2, y2) = line_segment

    # Calculate the differences
    dx = x2 - x1
    dy = y2 - y1

    # Calculate the t that minimizes the distance
    t = ((x0 - x1) * dx + (y0 - y1) * dy) / (dx * dx + dy * dy)

    # Restrict t to the segment
    t = max(0, min(1, t))

    # Calculate the nearest point on the segment
    nearest_x = x1 + t * dx
    nearest_y = y1 + t * dy

    # Calculate the distance from the point to the nearest point on the segment
    distance = np.sqrt((nearest_x - x0) ** 2 + (nearest_y - y0) ** 2)

    return distance

class Simulator(Node):
    def __init__(self):
        super().__init__('simulator')
        self.model = 'normal.robot'
        worlds = ['brick.world', 'pillars.world', 'open.world', 'ell.world', 'custom.world']
        self.world = 'pillars.world'

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

        # Publisher for LaserScan
        self.laser_publisher = self.create_publisher(LaserScan, '/scan', self.robot["laser"]["rate"]) #???

        # Parse Initial Pose and Occupancy Grid
        self.world = World(self.world)
        self.occupancy_grid = self.world.get_occupancy_grid()
        self.grid2d = self.world.get_grid_2d()
        self.resolution = self.world.get_resolution()
        initial_pose = self.world.get_initial_pose()
        self.x = initial_pose[0]
        self.y = initial_pose[1]
        self.theta = initial_pose[2]
        self.ogm = OccupancyGrid()
        self.publish_map()

        # Compute Obstacle Boundaries
        self.obstacle_corners = self.get_obstacle_corners()
        
        #Create Laser Scan Message
        self.create_timer(self.robot["laser"]["rate"], self.publish_laserscan)

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

    def calc_laser_points(self, point_count): #may need more parameters
        # Convert occupancy grid into numpy array to help with calculation
        np_grid = np.array(self.world.get_grid_2d())
        # Convert fov to radians
        fov_rad = self.robot["laser"]["angle_max"] - self.robot["laser"]["angle_min"]

        # Initialize the list of laser scan ranges
        ranges = []

        # Calculate the angular increment between each laser beam
        angular_increment = fov_rad / (point_count - 1)

        #Get resolution
        grid_resolution = self.world.get_resolution()

        # Iterate through each laser beam
        for i in range(point_count):
            # Calculate the current angle of the laser beam
            angle = self.theta - (fov_rad / 2.0) + i * angular_increment

            # Initialize the current range to the maximum range
            current_range = self.robot["laser"]["range_max"]

        
            # Iterate along the laser beam until the maximum range is reached or an obstacle is encountered
            for r in np.arange(0.0, self.robot["laser"]["range_max"] + grid_resolution, grid_resolution):
                # Calculate the coordinates of the point along the laser beam
                x = int(self.x + r * cos(angle) / grid_resolution)
                y = int(self.y + r * sin(angle) / grid_resolution)

                # Check if the point is within the occupancy grid bounds
                if 0 <= x < np_grid.shape[0] and 0 <= y < np_grid.shape[1]:
                    # Check if the point is occupied
                    if np_grid[x, y] == 100:
                        current_range = r
                        break

            # Convert the range from grid coordinates to meters
            current_range_meters = current_range * grid_resolution
            ranges.append(current_range_meters)

        return ranges

    #Gives error value for laser
    def laser_error(self, var):
        mean = 1
        std_dev = np.sqrt(var)

        return np.random.normal(mean, std_dev) #Returns random error from Gaussian distribution

    #Provides a check for a failed reading for laser 
    def fail_test(self, fail_prob):
        # Generate a random number between 0 and 1
        random_number = random.uniform(0, 1)

        # Check if the random number is less than the given probability
        if random_number < fail_prob:
            # The event occurs
            return True
        else:
            # The event does not occur
            return False
    
    def publish_laserscan(self):
        # Create a LaserScan message
        laser_scan_msg = LaserScan()

        # Fill in the message fields
        laser_scan_msg.header = Header(stamp=self.get_clock().now().to_msg(), frame_id="laser")
        laser_scan_msg.angle_min = self.robot["laser"]["angle_min"]  # Minimum angle in radians 
        laser_scan_msg.angle_max = self.robot["laser"]["angle_max"]   # Maximum angle in radians
        laser_scan_msg.angle_increment = (laser_scan_msg.angle_max - laser_scan_msg.angle_min) / self.robot["laser"]["count"]  # Angular distance between measurements
        laser_scan_msg.time_increment = 0.04  # Time between measurements in seconds (any value)
        laser_scan_msg.scan_time = float(self.robot["laser"]["rate"])  # Time to complete a full scan in seconds
        laser_scan_msg.range_min = self.robot["laser"]["range_min"]  # Minimum valid range in meters
        laser_scan_msg.range_max = self.robot["laser"]["range_max"] # Maximum valid range in meters

        range_vals = self.calc_laser_points(self.robot["laser"]["count"]) #Create function to calculate values

        #Add errors to values
        for i in range(len(range_vals)):
            range_vals[i] = range_vals[i] * self.laser_error(self.robot["laser"]["error_variance"])

        #Take out values outside of value range
        range_vals = [val for val in range_vals if laser_scan_msg.range_min <= val <= laser_scan_msg.range_max]

        #Replace values with NaN if failed
        for i in range(len(range_vals)):
            if (self.fail_test(self.robot["laser"]["fail_probability"])):
                range_vals[i] = nan #math.nan

        # Example range and intensity values
        laser_scan_msg.ranges = range_vals #Find values from function
        laser_scan_msg.intensities = [] #Blank since intensities not required

        self.laser_publisher.publish(laser_scan_msg)

    def publish_map(self):
        # Create and Publish Occupancy Grid Message (ogm)
        self.ogm.header.stamp = self.get_clock().now().to_msg()
        self.ogm.header.frame_id = 'world'

        self.ogm.info.resolution = self.resolution
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

    # Function to convert the array into a list of obstacle corner points
    def get_obstacle_corners(self):
        # List to hold the coordinates of corners
        corners = []
        rows = self.ogm.info.height
        cols = self.ogm.info.width
        
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


def main(args=None):
    rclpy.init(args=args)
    simulator = Simulator()
    rclpy.spin(simulator)

    # Destroy the node explicitly
    simulator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
