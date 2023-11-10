import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from project4.disc_robot import *
import numpy as np


class VelocityTranslator(Node):

    def __init__(self):
        super().__init__('velocity_translator')
        self.robot_model = self.declare_parameter('robot', 'normal.robot').value
        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.vr_publisher = self.create_publisher(Float64, '/vr', 10)
        self.vl_publisher = self.create_publisher(Float64, '/vl', 10)

    def cmd_vel_callback(self, msg):
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        # Calculate wheel velocities based on linear and angular components
        robot = load_disc_robot(self.robot_model)
        variance_left = robot['wheels']['error_variance_left']
        variance_right = robot['wheels']['error_variance_right']
        L = robot['wheels']['distance']

        error_vl = self.wheel_error(variance_left)
        error_vr = self.wheel_error(variance_right)

        #Determine special cases (linear.x or angular.z == 0)
        if(angular_z == 0.0): #No rotation
            vr = linear_x
            vl = linear_x
        elif(linear_x == 0.0):
            vr = angular_z * (L/2)
            vl = angular_z * (-L/2)
        else:
            #Insert calculations here
            vr = linear_x + (angular_z*L)/2
            vl = linear_x - (angular_z*L)/2


        # Publish the wheel velocities
        vr_msg = Float64()
        vl_msg = Float64()
        vr_msg.data = vr * error_vr
        vl_msg.data = vl * error_vl
        self.vr_publisher.publish(vr_msg)
        self.vl_publisher.publish(vl_msg)

    def wheel_error(self, var):
        mean = 1
        std_dev = np.sqrt(var)

        return np.random.normal(mean, std_dev) #Returns random error from Gaussian distribution


def main(args=None):
    rclpy.init(args=args)
    velocity_translator = VelocityTranslator()
    rclpy.spin(velocity_translator)
    velocity_translator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
