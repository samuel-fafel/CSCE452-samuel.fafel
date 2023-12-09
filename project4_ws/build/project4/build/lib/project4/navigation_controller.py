import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from math import nan
from statistics import mean, median_low

class NavigationController(Node):
    def __init__(self):
        super().__init__('navigation_controller')
        self.subscription = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.min_safe_distance = 0.22

    #Calculates the means of a list
    def calc_means(self, section):
        if len(section) > 0:
            return mean(section)
        else:
            return 0

    # Divide the ranges from LaserScan into sections and find the minimum distance of each section
    def laser_process(self, msg):
        #There should always be 25 points in the message, including nan
        if (len(msg.ranges) != 25):
            print("Invalid number of ranges")
            return -1

        # Define the number of points per section
        points_per_section = 3

        S0 = [] # Pure West
        S1 = [] # West North West
        S2 = [] # Pure North West
        S3 = [] # North North West
        S4 = [] # Pure North
        S5 = [] # North North East
        S6 = [] # Pure North East
        S7 = [] # East North East
        S8 = [] # Pure East

        # Initialize a list to hold the minimum distance for each section
        min_distances = []
        S0 = msg.ranges[0:3]
        S1 = msg.ranges[3:6]
        S2 = msg.ranges[6:9]
        S3 = msg.ranges[9:12]
        S4 = msg.ranges[11:14]
        S5 = msg.ranges[13:16]
        S6 = msg.ranges[16:19]
        S7 = msg.ranges[19:22]
        S8 = msg.ranges[22:25]

        sections = [S0,S1,S2,S3,S4,S5,S6,S7,S8]

        # Iterate over the ranges and divide them into sections
        for section in sections:
            section = [num for num in section if num != nan] #Remove nan values from sections

            # Calculate the minimum distance for the section
            min_distance = min(section) if section else float('inf')
            avg_distance = 0
            for num in section:
                avg_distance += num
            avg_distance /= points_per_section
            min_distances.append((avg_distance + min_distance)/2)
        
        return min_distances

    def scan_callback(self, msg):
        # Process the laser scan data and generate velocity commands
        linear_speed = 0.0
        angular_speed = 0.0

        # Get the minimum distances for each region from the laser scan
        region_min_dists = self.laser_process(msg)

        # Determine the direction to move based on the minimum distances
        if not region_min_dists:  # In case all measurements are invalid
            linear_speed = 0.0
            angular_speed = 2.0
        else:
            # Find the index with the maximum value, which indicates the direction with the most open space
            max_dist = max(region_min_dists)
            min_dist = min(region_min_dists)
            max_dist_index = region_min_dists.index(max_dist)
            min_dist_index = region_min_dists.index(min_dist)

            # Define speeds based on laser processing
            # The robot should move towards the most open space (WITHOUT bumping corners)
            override = False
            for i, dist in enumerate(region_min_dists):
                if (dist <= self.min_safe_distance) or (i == 1 and dist < 0.4):
                    print("COLLISION")
                    linear_speed = 0.0
                    override = True
                    if region_min_dists.index(dist) == 0: # Pure West
                        angular_speed = 0.5 # Turn East
                    elif region_min_dists.index(dist) == 1: 
                        linear_speed = 0.1
                        angular_speed = 0.4
                    elif region_min_dists.index(dist) == 2: # Pure North West
                        angular_speed = 0.3
                    elif region_min_dists.index(dist) == 3:
                        angular_speed = 0.2
                    elif region_min_dists.index(dist) == 4: # Pure North
                        angular_speed = -2.0
                    elif region_min_dists.index(dist) == 5:
                        angular_speed = -0.2
                    elif region_min_dists.index(dist) == 6: # Pure North East
                        angular_speed = -0.3
                    elif region_min_dists.index(dist) == 7:
                        linear_speed = 0.1
                        angular_speed = -0.4
                    elif region_min_dists.index(dist) == 8: # Pure East
                        angular_speed = -0.5 # Turn West
                    else:
                        linear_speed = -0.05
                        angular_speed = -2.0
            if region_min_dists[4] > 1:
                override = True
                if (region_min_dists[3] > 1 and region_min_dists[5] > 1):
                    linear_speed = min(max_dist * 0.1, 0.4)
                    angular_speed = 0.0

                elif region_min_dists[0] > 1:
                    linear_speed = 0.0
                    angular_speed = -0.4
                elif region_min_dists[8] > 1:
                    linear_speed = 0.0
                    angular_speed = 0.4

                elif region_min_dists[3] > 1:
                    linear_speed = 0.1
                    angular_speed = -0.2
                elif region_min_dists[5] > 1:
                    linear_speed = 0.1
                    angular_speed = 0.2
            if not override:
                if max_dist_index == 0:  # Pure West
                    linear_speed = 0.0
                    angular_speed = -0.5 # Turn West
                elif max_dist_index == 1:  # West North West
                    linear_speed = 0.0
                    angular_speed = -0.4
                elif max_dist_index == 2:  # Pure North West
                    linear_speed = 0.0
                    angular_speed = -0.3
                elif max_dist_index == 3:  # North North West
                    linear_speed = 0.1
                    angular_speed = -0.2
                elif max_dist_index == 4:  # Pure North
                    linear_speed = min(max_dist * 0.1, 0.4)
                    angular_speed = 0.0
                elif max_dist_index == 5:  # North North East
                    linear_speed = 0.1
                    angular_speed = 0.2
                elif max_dist_index == 6:  # Pure North East
                    linear_speed = 0.0
                    angular_speed = 0.3
                elif max_dist_index == 7:  # East North East
                    linear_speed = 0.0
                    angular_speed = 0.4
                elif max_dist_index == 8:  # Pure east
                    linear_speed = 0.0
                    angular_speed = 0.5 # Turn East

        # Create Twist message
        twist_msg = Twist()
        twist_msg.linear.x = linear_speed
        twist_msg.angular.z = angular_speed

        # Publish the Twist message
        self.publisher.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)

    navigation_controller = NavigationController()

    rclpy.spin(navigation_controller)

    navigation_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
