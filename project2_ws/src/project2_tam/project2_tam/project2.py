import rclpy
from rclpy.node import Node
from turtlesim.srv import TeleportAbsolute ##### REMOVE #####
from turtlesim.srv import SetPen
from turtlesim.srv import Spawn
from turtlesim.srv import Kill
from std_srvs.srv import Empty
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from math import atan2, cos, sin, degrees, sqrt, pi
import os
import project2_tam.tam_logo as tam
from steering import euclidean_distance, linear_vel, steering_angle, angular_vel

class DrawTAM(Node):
	def __init__(self, node_name):
		super().__init__(node_name)
		self.control_node = rclpy.create_node('turtle_control')
		self.coordinates = tam.my_points ##### REMOVE #####
		self.segments = tam.my_lines
		
		self.num_turtles = self.declare_parameter('num_turtles', 1).value
		self.turtle_names = [f'turtle{i}' for i in range(1, self.num_turtles + 1)] # Name the turtles 

		self.rate = self.control_node.create_rate(10)

		self.initialize_simulator() # Prep turtlesim

		# Initialize all turtles with publishers and subscribers
		self.SetPen_Clients = {}
		self.Twist_Publishers = {}
		self.Pose_Subscriptions = {}
		self.Teleport_Clients = {} ##### REMOVE #####
		self.Turtle_Poses = {}
		for turtle_name in self.turtle_names:
			self.spawn_turtle(turtle_name) # Spawn turtle

			self.SetPen_Clients[turtle_name]	 = self.create_client(SetPen, f"/{turtle_name}/set_pen")
			self.Twist_Publishers[turtle_name]   = self.create_publisher(Twist, f'/{turtle_name}/cmd_vel', 10)
			self.Pose_Subscriptions[turtle_name] = self.control_node.create_subscription(Pose,f'/{turtle_name}/pose',
													lambda msg, name=turtle_name: self.pose_callback(name, msg), 10)
			self.Teleport_Clients[turtle_name]   = self.create_client(TeleportAbsolute, f'/{turtle_name}/teleport_absolute') ##### REMOVE #####

			self.set_pen_white(turtle_name)

		for turtle_name in self.turtle_names:
			print(self.segments[0])
			#self.rotate_turtle(turtle_name, self.Turtle_Poses[turtle_name], self.segments[0].start)
			#self.move_to_target(turtle_name, self.Turtle_Poses[turtle_name], self.segments[0].start)
			#self.rotate_turtle(turtle_name, self.Turtle_Poses[turtle_name], self.segments[0].end)
			#self.move_to_target(turtle_name, self.Turtle_Poses[turtle_name], self.segments[0].end)

			self.draw_logo_one_turtle(turtle_name, self.Turtle_Poses[turtle_name])
			self.draw_logo_one_turtle(turtle_name, self.Turtle_Poses[turtle_name])

			#self.draw_logo_one_turtle(turtle_name, self.Turtle_Poses[turtle_name])
			#self.teleport_turtle(turtle_name)
			self.initialize_simulator()

	def pose_callback(self, turtle_name, msg):
		self.Turtle_Poses[turtle_name].x = msg.x
		self.Turtle_Poses[turtle_name].y = msg.y
		self.Turtle_Poses[turtle_name].theta = msg.theta

	def initialize_simulator(self): # Clear and set Background
		self.client_clear = self.create_client(Empty, '/clear')  # Client for /clear service
		self.client_clear.wait_for_service()
		
		req_clear = Empty.Request()
		self.client_clear.call_async(req_clear) # Clear Turtlesim
		self.get_logger().info('Simulator cleared.')

		path_to_params = "~/ros2_humble/CSCE452-samuel.fafel/project2_ws/src/project2_tam/project2_tam/background_params.txt"
		os.system(f"ros2 param load /turtlesim {path_to_params}") # Set background color to Aggie Maroon
		
	def spawn_turtle(self, turtle_name): # Spawn a turtle in the center
		self.client_spawn = self.create_client(Spawn, 'spawn')
		req_spawn = Spawn.Request()
		req_spawn.name = turtle_name
		req_spawn.x = 5.544445
		req_spawn.y = 5.544445
		req_spawn.theta = 0.0
		self.client_spawn.call_async(req_spawn)
		self.Turtle_Poses[turtle_name] = Pose()
		self.Turtle_Poses[turtle_name].x = 5.44445
		self.Turtle_Poses[turtle_name].y = 5.44445
		self.Turtle_Poses[turtle_name].theta = 0.0

	def set_pen_white(self, turtle_name, on_off=True): # Set the pen color to white
		self.SetPen_Clients[turtle_name].wait_for_service()
		req_pen = SetPen.Request()
		req_pen.r = 255
		req_pen.g = 255
		req_pen.b = 255
		req_pen.width = 2
		req_pen.off = not on_off
		self.SetPen_Clients[turtle_name].call_async(req_pen)

	def teleport_turtle(self, turtle_name): ##### REMOVE #####
		self.Teleport_Clients[turtle_name].wait_for_service()
		for coord in self.coordinates:
			req = TeleportAbsolute.Request()
			req.x = coord.x
			req.y = coord.y
			req.theta = 0.0

			self.get_logger().info(f"Teleporting turtle to ({req.x}, {req.y})")
			future = self.Teleport_Clients[turtle_name].call_async(req)
			rclpy.spin_until_future_complete(self, future)
			self.spin()
		self.remove_turtle(turtle_name)
	
	def draw_logo_one_turtle(self, turtle_name, turtle_pose):
		for segment in self.segments:
			self.set_pen_white(turtle_name, on_off=False) # Move to line segment, with pen off
			self.rotate_turtle(turtle_name, turtle_pose, segment.start)
			self.move_to_target(turtle_name, turtle_pose, segment.start)

			self.set_pen_white(turtle_name, on_off=True) # Draw line segment, with pen on
			self.rotate_turtle(turtle_name, turtle_pose, segment.end)
			self.move_to_target(turtle_name, turtle_pose, segment.end)

	def rotate_turtle(self, turtle_name, turtle_pose, point, tolerance=0.01):
		# Calculate the angle to rotate towards the target point
		angle_to_target = steering_angle(point, turtle_pose)
		if (point.y > turtle_pose.y): angle_to_target += pi
		angle_diff = angle_to_target - turtle_pose.theta

		print("------ Rotating ------")
		print(f"Target: ({point.x},{point.y})")
		print(f"Turtle: ({turtle_pose.x},{turtle_pose.y}, {turtle_pose.theta})")
		print(f"{angle_diff} = {angle_to_target} - {turtle_pose.theta}")

		twist_msg = Twist()
		# Rotate the turtle towards the target point
		while abs(steering_angle(point, turtle_pose) - turtle_pose.theta) > tolerance:
			# Change angle
			twist_msg.linear.x = 0.0
			twist_msg.angular.z = angular_vel(point, turtle_pose, 1)
			self.Twist_Publishers[turtle_name].publish(twist_msg)
			rclpy.spin_once(self.control_node)
			
			# Get new pose
			turtle_pose = self.Turtle_Poses[turtle_name]
		
		# Stop rotating
		twist_msg = Twist()
		twist_msg.angular.z = 0.0
		self.Twist_Publishers[turtle_name].publish(twist_msg)
		rclpy.spin_once(self.control_node)

		print("------ Stopping ------")
		print(f"Target: ({point.x},{point.y})")
		print(f"Turtle: ({turtle_pose.x},{turtle_pose.y}, {turtle_pose.theta})")
		print(f"Angle Diff = {angle_diff} = {angle_to_target} - {turtle_pose.theta}")
		print("----------------------")

	def move_to_target(self, turtle_name, turtle_pose, point, tolerance=0.1):
		print("------ Swimming ------")
		print(f"Target: ({point.x},{point.y})")
		print(f"Turtle: ({turtle_pose.x},{turtle_pose.y}, {turtle_pose.theta})")
		#print(f"Distance to Target = {distance_to_target}")

		# Move the turtle to the target point
		while euclidean_distance(point, turtle_pose) >= tolerance:
			twist_msg = Twist()
			twist_msg.linear.x = linear_vel(point, turtle_pose, 1)
			self.Twist_Publishers[turtle_name].publish(twist_msg)
			rclpy.spin_once(self.control_node)

			turtle_pose = self.Turtle_Poses[turtle_name]
		
		#self.rate.sleep()
	
		# Stop movement
		twist_msg = Twist()
		twist_msg.linear.x = 0.0
		self.Twist_Publishers[turtle_name].publish(twist_msg)
		rclpy.spin_once(self.control_node)

		print("------ Stopping ------")
		print(f"Target: ({point.x},{point.y})")
		print(f"Turtle: ({turtle_pose.x},{turtle_pose.y}, {turtle_pose.theta})")
		#print(f"{distance_to_target}")
		print("----------------------")
	
	def remove_turtle(self, turtle_name): # Kill a turtle :(
		self.client_kill = self.create_client(Kill, "/kill") # Client for kill service
		self.client_kill.wait_for_service()
		req_kill = Kill.Request()
		req_kill.name = turtle_name
		future_kill = self.client_kill.call_async(req_kill)
		rclpy.spin_until_future_complete(self, future_kill)
		
def main(args=None):
	rclpy.init(args=args)

	# List of coordinates to teleport to
	node_name = "turtle_teleporter"
	
	teleported = DrawTAM(node_name)
	#rclpy.spin(teleported)

	teleported.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()

