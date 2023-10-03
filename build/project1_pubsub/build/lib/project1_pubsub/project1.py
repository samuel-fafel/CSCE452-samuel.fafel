# Samuel Fafel
# 330003077
# Texas A&M University
# CSCE 452
# Project 1

# A program which publishes messages to /turtle1/cmd_vel and subscribes to /turtle1/color_sensor

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Color

class Controller(Node):

    def __init__(self):
        super().__init__('controller')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.subscription = self.create_subscription(
            Color,
            '/turtle1/color_sensor',
            self.listener_callback,
            10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 0.5  # Move forward
        msg.angular.z = 0.2  # Turn left
        self.publisher_.publish(msg)

    def listener_callback(self, msg):
        self.get_logger().info(f'Received color data: R:{msg.r} G:{msg.g} B:{msg.b}')

def main(args=None):
    rclpy.init(args=args)

    my_controller = Controller()

    rclpy.spin(my_controller)

    my_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

