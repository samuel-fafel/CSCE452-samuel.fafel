import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from project4.disc_robot import *
from rclpy.clock import Clock

import numpy as np

class VelocityTranslator(Node):

    def __init__(self):
        super().__init__('velocity_translator')
        self.model = 'ideal.robot'

        self.robot_model = self.declare_parameter('robot', self.model).value
        self.cmd_vel_subscription = self.create_subscription(Twist,'/cmd_vel',self.cmd_vel_callback,10)
        self.vr_publisher = self.create_publisher(Float64, '/vr', 10)
        self.vl_publisher = self.create_publisher(Float64, '/vl', 10)

        #Set timer
        self.last_update_error_time = Clock().now()
        self.create_timer(0.1, self.timer_callback)

        #Load robot info
        self.robot = load_disc_robot(self.robot_model)
        self.variance_left = self.robot['wheels']['error_variance_left']
        self.variance_right = self.robot['wheels']['error_variance_right']
        self.L = self.robot['wheels']['distance']
        self.error_rate = self.robot['wheels']['error_update_rate']

        #Get initial errors
        self.error_vl = self.wheel_error(self.variance_left)
        self.error_vr = self.wheel_error(self.variance_right)
    
    def timer_callback(self):
        current_time = Clock().now()
        if ((current_time - self.last_update_error_time).nanoseconds >= 1e9 * self.error_rate): # nanoseconds
            #Determine a new error
            self.error_vl = self.wheel_error(self.variance_left)
            self.error_vr = self.wheel_error(self.variance_right)
            self.last_update_error_time = current_time

    def cmd_vel_callback(self, msg):
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        print(f"/cmd_vel: Linear X:{linear_x}, Angular Z: {angular_z}")

        #Determine special cases (linear.x or angular.z == 0)
        if(angular_z == 0.0): #No rotation
            vr = linear_x
            vl = linear_x
        elif(linear_x == 0.0):
            vr = angular_z * (self.L/2)
            vl = angular_z * (-self.L/2)
        else:
            #Insert calculations here
            vr = linear_x + (angular_z*self.L)/2
            vl = linear_x - (angular_z*self.L)/2

        # Publish the wheel velocities
        vr_msg = Float64()
        vl_msg = Float64()
        vr_msg.data = vr * self.error_vr
        vl_msg.data = vl * self.error_vl
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

