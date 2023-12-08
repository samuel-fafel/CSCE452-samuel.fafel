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

    #Calculates the means of a list
    def calc_means(self, section):
        if len(section) > 0:
            return mean(section)
        else:
            return 0

    #Divide the ranges from LaserScan into sections and find the average distance of each section
    def laser_process(self, msg):
        #Initialize variables
        avgs = []
        section_E = []
        section_NE = []
        section_N = []
        section_NW = []
        section_W = []

        #There should always be 25 points in the message, including nan
        if (len(msg.ranges) != 25):
            print("Invalid number of ranges")
            return -1
        
        #Divide into five regions with five points per region (W, NW, N, NE, E). Starts with E
        for i in range(5):
            section_E.append(msg.ranges[i])
        
        for i in range(5, 10):
            section_NE.append(msg.ranges[i])

        for i in range(10, 15):
            section_N.append(msg.ranges[i])

        for i in range(15, 20):
            section_NW.append(msg.ranges[i])

        for i in range(20, 25):
            section_NW.append(msg.ranges[i])

        #Remove nan values from sections
        section_E = [num for num in section_E if num != nan]
        section_NE = [num for num in section_NE if num != nan]
        section_N = [num for num in section_N if num != nan]
        section_NW = [num for num in section_NW if num != nan]
        section_W = [num for num in section_W if num != nan]

        #Calculate means and append to avgs
        avgs.append(self.calc_means(section_E))
        avgs.append(self.calc_means(section_NE))
        avgs.append(self.calc_means(section_N))
        avgs.append(self.calc_means(section_NW))
        avgs.append(self.calc_means(section_W))

        return avgs

    #Find the maximum distance and return the index. Going straight should take precedence over turning
    def find_max_dist(self, avgs):
        max = avgs[0]
        max_index = 0
        middle_index = 2

        #Find max
        for i in range(1, len(avgs)):
            if avgs[i] > max:
                max = avgs[i]
                max_index = i
            elif avgs[i] == max:
                #Find which index is closer to median
                diff_curr = abs(max_index - middle_index)
                diff_new = abs(i - middle_index)
                
                if diff_new > diff_curr:
                    max = avgs[i]
                    max_index = i
        
        return max_index

    def scan_callback(self, msg):
        # Process the laser scan data and generate velocity commands
        linear_speed = 0.0
        angular_speed = 0.0

        region_avgs = self.laser_process(msg)

        max_dist = self.find_max_dist(region_avgs)

        #Define speeds based on laser processing
        #Robot should go to the most open space (region with farthest distance away)
        if max_dist == 0: # East
            linear_speed = 0.00
            angular_speed = -0.4
        elif max_dist == 1: # North East
            linear_speed = 0.1
            angular_speed = -0.2
        elif max_dist == 2: # North
            linear_speed = 0.3
            angular_speed = 0.0
        elif max_dist == 3: # North West
            linear_speed = 0.1
            angular_speed = 0.2
        elif max_dist == 4: # West
            linear_speed = 0.00
            angular_speed = 0.4

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
