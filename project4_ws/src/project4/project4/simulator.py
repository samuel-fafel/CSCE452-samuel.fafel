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
        #self.world = 'pillars.world'
        self.world = worlds[random.randint(0,4)]

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
        self.laser_publisher = self.create_publisher(LaserScan, '/scan', 1)

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
        # Convert fov to radians
        fov_rad = self.robot["laser"]["angle_max"] - self.robot["laser"]["angle_min"]
        
        # Calculate the angular increment between each laser beam
        angular_increment = fov_rad / (point_count - 1)

        # Initialize the list of laser scan ranges
        ranges = []

        # Iterate through each laser beam
        for i in range(point_count):
            # Calculate the current angle of the laser beam
            angle = self.robot['laser']['angle_min'] + i*angular_increment
            pot_ranges = []

            #Iterate through obstacles
            for obs in self.obstacle_corners:
                x_obs = obs[0]
                y_obs = obs[1]

                angle_diff = atan2(y_obs - self.y, x_obs - self.x) #Find angle difference between

                angle_looking = angle_diff - self.theta #Find difference relative to robot's theta

                true_angle_diff = atan2(sin(angle_looking), cos(angle_looking)) #Keep bounds between pi and -pi
                
                #Get the correct difference for the current angle reading
                if ((angle-0.05) <= true_angle_diff <= (angle+0.05)):
                    distance = sqrt((x_obs - self.x)**2 + (y_obs - self.y)**2)
                    pot_ranges.append(distance)
                
            #Choose closest range out of pot_ranges
            if (len(pot_ranges) > 0):
                ranges.append(min(pot_ranges))
                    
        return ranges

    def calculate_scan_distances(self, point_count):
        # Convert fov to radians
        fov_rad = self.robot["laser"]["angle_max"] - self.robot["laser"]["angle_min"]
        
        # Calculate the angular increment between each laser beam
        angle_increment = fov_rad / (point_count - 1)

        distances = []
        for i in range(point_count):
            angle = self.robot['laser']['angle_min'] + i * angle_increment*2
            distance = self.cast_ray(angle)
            distances.append(distance)
        return distances

    def cast_ray(self, angle):
        # Ray's starting point (robot's position)
        ray_x, ray_y = self.x, self.y
        range_max = self.robot['laser']['range_max']

        # Determine the end point of the ray based on the laser's max range
        ray_end_x = ray_x + range_max * cos(angle + self.theta)
        ray_end_y = ray_y + range_max * sin(angle + self.theta)

        # Iterate over points along the ray
        for current_point in bresenham_line(ray_x, ray_y, ray_end_x, ray_end_y, self.resolution):
            if self.is_occupied(*current_point):
                # Calculate the distance from the robot to this point
                distance = sqrt((current_point[0] - self.x)**2 + (current_point[1] - self.y)**2)
                return distance  # Return the distance to the first occupied point

        return self.range_max  # If no obstacle is found, return max range

    def is_occupied(self, x, y):
        grid_x = int(x / self.resolution)
        grid_y = int(y / self.resolution)
        if self.grid2d[grid_y][grid_x] == 100:
            return True
        return False

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
        laser_scan_msg.angle_increment = (laser_scan_msg.angle_max - laser_scan_msg.angle_min) / (self.robot["laser"]["count"] - 1)  # Angular distance between measurements
        laser_scan_msg.time_increment = 0.04  # Time between measurements in seconds (any value)
        laser_scan_msg.scan_time = float(self.robot["laser"]["rate"])  # Time to complete a full scan in seconds
        laser_scan_msg.range_min = self.robot["laser"]["range_min"]  # Minimum valid range in meters
        laser_scan_msg.range_max = self.robot["laser"]["range_max"] # Maximum valid range in meters

        range_vals = self.calculate_scan_distances(self.robot["laser"]["count"]) #Create function to calculate values

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
