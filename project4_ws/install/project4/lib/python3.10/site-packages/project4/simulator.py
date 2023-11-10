import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from project4.disc_robot import *
#import tf_transformations
from math import sin, cos

class Simulator(Node):
    def __init__(self):
        super().__init__('simulator')
        
        # Load the robot dictionary from the YAML file
        robot_dict = load_disc_robot('normal.robot')
        # Set the 'robot_description' parameter to the URDF string
        self.declare_parameter('robot_description', robot_dict['urdf'])
        
        # Subscribers for wheel velocities
        self.vl_subscriber = self.create_subscription(Float64, '/vl', self.vl_callback, 10)
        self.vr_subscriber = self.create_subscription(Float64, '/vr', self.vr_callback, 10)
        
        # Pose and transform publishers
        self.pose_publisher = self.create_publisher(Twist, '/robot_pose', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Initialize pose variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Wheel velocities
        self.v_left = 0.0
        self.v_right = 0.0
        
        # Robot width parameter (distance between wheels)
        self.wheel_separation = 0.5  # Example value, adjust to your robot's specification
        
        # Set the rate of pose update and the last update time
        self.rate = self.create_rate(10)  # 10 Hz
        self.last_update_time = self.get_clock().now()
        
        # Start the main loop
        self.create_timer(0.1, self.update_pose)  # 10 Hz

    def vl_callback(self, msg):
        self.v_left = msg.data

    def vr_callback(self, msg):
        self.v_right = msg.data
        
    def update_pose(self):
        # Get current time and calculate delta time
        current_time = self.get_clock().now()
        dt = (current_time - self.last_update_time).nanoseconds / 1e9
        self.last_update_time = current_time
        
        # Calculate linear and angular velocities
        v_linear = (self.v_right + self.v_left) / 2.0
        v_angular = (self.v_right - self.v_left) / self.wheel_separation
        
        # Compute the new pose
        delta_x = v_linear * cos(self.theta) * dt
        delta_y = v_linear * sin(self.theta) * dt
        delta_theta = v_angular * dt
        
        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta
        
        # Normalize theta
        self.theta = (self.theta + 3.14159265) % (2 * 3.14159265) - 3.14159265
        
        # Publish the new pose
        self.publish_pose()

    def publish_pose(self):
        # Calculate and publish the robot's current pose
        # as a Twist message (or another appropriate message type)
        pose_msg = Twist()
        pose_msg.linear.x = self.x
        pose_msg.linear.y = self.y
        pose_msg.angular.z = self.theta
        self.pose_publisher.publish(pose_msg)

        # Broadcast the transform from the world frame to the robot's frame
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0  # Assuming flat ground
        #q = tf_transformations.quaternion_from_euler(0, 0, self.theta)
        #t.transform.rotation.x = q[0]
        #t.transform.rotation.y = q[1]
        #t.transform.rotation.z = q[2]
        #t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    simulator = Simulator()
    rclpy.spin(simulator)

    # Destroy the node explicitly
    simulator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()