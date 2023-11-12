import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from project4.disc_robot import *
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from math import sin, cos
from rclpy.clock import Clock

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
        # Subscribers for wheel velocities
        self.vl_subscriber = self.create_subscription(Float64, '/vl', self.vl_callback, 10)
        self.vr_subscriber = self.create_subscription(Float64, '/vr', self.vr_callback, 10)
        
        # Transform publishers
        self.tf_broadcaster = TransformBroadcaster(self)

        # Create Command Timer
        self.last_cmd_vel_time = Clock().now()
        self.timer = self.create_timer(0.1, self.timer_callback)  # Check at 10 Hz
        
        # Initialize pose variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Wheel velocities
        self.v_left = 0.0
        self.v_right = 0.0
        
        # Robot width parameter (distance between wheels)
        self.robot_model = self.declare_parameter('robot', 'bad.robot').value
        self.robot = load_disc_robot(self.robot_model)
        self.wheel_separation = self.robot['wheels']['distance']
        
        # Set the rate of pose update and the last update time
        self.rate = self.create_rate(10)  # 10 Hz
        self.last_update_time = self.get_clock().now()
        
        # Start the main loop
        self.create_timer(0.1, self.update_pose)  # 10 Hz

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

def main(args=None):
    rclpy.init(args=args)
    simulator = Simulator()
    rclpy.spin(simulator)

    # Destroy the node explicitly
    simulator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()