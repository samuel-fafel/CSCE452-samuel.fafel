import rclpy
from rclpy.node import Node
from turtlesim.srv import SetPen
from turtlesim.srv import Spawn
from turtlesim.srv import Kill
from std_srvs.srv import Empty
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from math import atan2, cos, sin, degrees, sqrt, pi
import os
import project2_tam.tam_logo as tam
import project2_tam.steering
from project2_tam.steering import euclidean_distance, linear_vel, steering_angle, angular_vel, theory_distance

class DrawTAM(Node):
	def __init__(self, node_name):
		super().__init__(node_name)
		self.segments = tam.my_lines
		
		self.num_turtles = self.declare_parameter('num_turtles', 1).value
		self.turtle_names = [f'turtle{i}' for i in range(1, self.num_turtles + 1)] # Name the turtles 

		self.initialize_simulator() # Prep turtlesim

		# Initialize all turtles with publishers and subscribers
		self.SetPen_Clients = {}
		self.Twist_Publishers = {}
		self.Pose_Subscriptions = {}
		self.Turtle_Poses = {}
		for turtle_name in self.turtle_names:
			self.spawn_turtle(turtle_name) # Spawn turtle

			self.SetPen_Clients[turtle_name]	 = self.create_client(SetPen, f"/{turtle_name}/set_pen")
			self.Twist_Publishers[turtle_name]   = self.create_publisher(Twist, f'/{turtle_name}/cmd_vel', 10)
			self.Pose_Subscriptions[turtle_name] = self.create_subscription(Pose,f'/{turtle_name}/pose',
													lambda msg, name=turtle_name: self.pose_callback(name, msg), 10)

			self.set_pen_white(turtle_name)

		#for turtle_name in self.turtle_names:
		timer_period = 0.1
		self.timer = self.create_timer(timer_period, self.move_to_goal(self.turtle_names[0], self.segments[0].start, self.segments[::-1]))

	def pose_callback(self, turtle_name, msg):
		self.Turtle_Poses[turtle_name].x = msg.x
		self.Turtle_Poses[turtle_name].y = msg.y
		self.Turtle_Poses[turtle_name].theta = msg.theta
		POSITION  = "POSE:  X: {:.3}, Y: {:.3}, Theta: {:f}".format(self.Turtle_Poses[turtle_name].x, self.Turtle_Poses[turtle_name].y, self.Turtle_Poses[turtle_name].theta)
		#print(POSITION)

	def initialize_simulator(self): # Clear and set Background
		self.client_reset = self.create_client(Empty, '/reset')  # Client for /reset service
		self.client_reset.wait_for_service()
		
		req_reset = Empty.Request()
		self.client_reset.call_async(req_reset) # Clear Turtlesim
		self.get_logger().info('Simulator reset.')

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

	def calculate_Twist(self, pose, goal):
		vel_msg = Twist()
		vel_msg.x = 0.0
		vel_msg.z = 0.0
		vel_msg.theta = 0.0
		flag = False

		# If I stopped turning and went forward, how close would I be to the goal?
		current_distance  = euclidean_distance(goal, pose)
		hypothesis_result = theory_distance(goal, pose)
		error = abs(hypothesis_result - current_distance)
		
		if abs(steering_angle(goal, pose) - pose.theta) >= angular_tolerance:
			vel_msg.linear.x = 0.0
			vel_msg.angular.z = angular_vel(goal, pose)
		else:
			print("Steering angle within tolerance")
			vel_msg.angular.z = 0.0
			if euclidean_distance(goal, pose) >= distance_tolerance:
				vel_msg.linear.x = linear_vel(goal, pose)
				print("moving")
			else:
				print("distance within tolerance")
				vel_msg.linear.x = 0.0
				flag = True

		if self.flag:
			if euclidean_distance(goal, self.job.start) == 0:
				goal.x = self.job.end.x
				goal.y = self.job.end.y
				flag = False
				print("MADE IT TO THE START!")
			elif euclidean_distance(goal, self.job.end) == 0 and len(self.segments):
				self.job = self.segments.pop()
				goal.x = self.job.start.x
				goal.y = self.job.start.y
				self.flag = False
				print("MADE IT TO THE END!")
			else:
				return vel_msg
		return vel_msg

	def publish_Twists(self, turtle_name, twists):
		for t in twists:
			self.Twist_Publishers[turtle_name].publish(t)
			rclpy.spin_once(self)
		

	def move_to_goal(self, turtle_name, goal, segments):
		twist_msg = Twist()
		angluar_tolerance=0.01
		linear_tolerance=0.005

		job = segments.pop()
		
		JOB_START = "START: X: {:.3}, Y: {:.3}".format(job.start.x, job.start.y)
		JOB_END   = "END:   X: {:.3}, Y: {:.3}".format(job.end.x, job.end.y)
		GOAL	  = "GOAL:  X: {:.3}, Y: {:.3}".format(goal.x, goal.y)
		POSITION  = "POSE:  X: {:.3}, Y: {:.3}, Theta: {:.3}".format(self.Turtle_Poses[turtle_name].x, self.Turtle_Poses[turtle_name].y, self.Turtle_Poses[turtle_name].theta)
		STR_ANGL  = f"ANGLE: {steering_angle(goal, self.Turtle_Poses[turtle_name])}"
		print(JOB_START)
		print(JOB_END)
		print(GOAL)
		print(POSITION)
		print(STR_ANGL)

		# Move the turtle to the target goal
		while euclidean_distance(goal, self.Turtle_Poses[turtle_name]) >= linear_tolerance:
			# If I stopped turning and went forward, how close would I be to the goal?
			current_distance  = euclidean_distance(goal, self.Turtle_Poses[turtle_name])
			hypothesis_result = theory_distance(goal, self.Turtle_Poses[turtle_name])
			error = abs(hypothesis_result - current_distance)
			#print(f"Hypothesis: {hypothesis_result}")
			#print(f"Current: {current_distance}")
			#print(f"Error: {error}")

			# Rotate the turtle towards the target goal
			if hypothesis_result > linear_tolerance:
				# Start rotating
				twist_msg.linear.x = 0.0
				twist_msg.angular.z = angular_vel(goal, self.Turtle_Poses[turtle_name], 1)
				self.Twist_Publishers[turtle_name].publish(twist_msg)
				rclpy.spin_once(self)
			else:
				# Stop rotating
				twist_msg = Twist()
				twist_msg.angular.z = 0.0
				self.Twist_Publishers[turtle_name].publish(twist_msg)
				rclpy.spin_once(self)

				# Start moving
				twist_msg = Twist()
				twist_msg.angular.z = 0.0
				twist_msg.linear.x = linear_vel(goal, self.Turtle_Poses[turtle_name], 5)
				self.Twist_Publishers[turtle_name].publish(twist_msg)
				rclpy.spin_once(self)
	
		# Arrived at destination, Stop movement
		twist_msg = Twist()
		twist_msg.linear.x = 0.0
		self.Twist_Publishers[turtle_name].publish(twist_msg)
		rclpy.spin_once(self)

		if euclidean_distance(goal, job.start) < linear_tolerance:
			self.move_to_goal(turtle_name, job.end, segments)
			print("MADE IT TO THE START!")
		elif euclidean_distance(goal, job.end) < linear_tolerance and len(segments):
			job = segments.pop()
			self.move_to_goal(turtle_name, job.start, segments)
			print("MADE IT TO THE END!")
		else:
			quit()

		print("-----")
	
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

