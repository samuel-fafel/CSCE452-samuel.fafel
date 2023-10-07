import rclpy
from rclpy.node import Node
from turtlesim.srv import SetPen
from turtlesim.srv import Spawn
from turtlesim.srv import Kill
from std_srvs.srv import Empty
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import os
import project2_tam.tam_logo as tam
import project2_tam.steering
from project2_tam.steering import euclidean_distance, linear_vel, steering_angle, angular_vel, theory_distance

class DrawTAM(Node):
	def __init__(self, node_name):
		super().__init__(node_name)
		self.segments = tam.my_lines[::-1]
		self.num_turtles = self.declare_parameter('num_turtles', 1).value
		if self.num_turtles > len(self.segments): # Cannot have more turtles than lines to draw
			self.num_turtles = len(self.segments)

		self.turtle_names = [f'turtle{i}' for i in range(1, self.num_turtles + 1)] # Name the turtles 
		self.num_segments_per_turtle = len(self.segments) // self.num_turtles
		self.left_over_segments = len(self.segments) % self.num_turtles
		self.accuracy = 1000 # Affects error tolerance in calculate_twist

		self.initialize_simulator() # Prep turtlesim

		# Initialize all turtles with publishers and subscribers
		self.SetPen_Clients = {} 	# Clients for all Turtles
		self.Twist_Publishers = {}	# Publishers for all Turtles
		self.Pose_Subscriptions = {}# Subscribers for all Turtles
		self.Turtle_Poses = {}		# Poses of all Turtles
		self.Turtle_Segments = {}	# Segments Assigned to Turtles
		for turtle_name in self.turtle_names:
			self.spawn_turtle(turtle_name) # Spawn turtle

			self.SetPen_Clients[turtle_name]	 = self.create_client(SetPen, f"/{turtle_name}/set_pen")
			self.Twist_Publishers[turtle_name]   = self.create_publisher(Twist, f'/{turtle_name}/cmd_vel', 10)
			self.Pose_Subscriptions[turtle_name] = self.create_subscription(Pose,f'/{turtle_name}/pose',
													lambda msg, name=turtle_name: self.pose_callback(name, msg), 10)

			self.set_pen_white(turtle_name)
		
		self.distribute_segments() # Distribute Segments Among Turtles
		self.direct_multiple_turtles(self.num_segments_per_turtle) # Draw TAMU Logo!
		quit() # Terminate

	def pose_callback(self, turtle_name, msg): # Callback for Pose Subscription
		self.Turtle_Poses[turtle_name].x = msg.x
		self.Turtle_Poses[turtle_name].y = msg.y
		self.Turtle_Poses[turtle_name].theta = msg.theta
		POSITION  = "POSE:  X: {:.3}, Y: {:.3}, Theta: {:f}".format(self.Turtle_Poses[turtle_name].x, self.Turtle_Poses[turtle_name].y, self.Turtle_Poses[turtle_name].theta)
		#print(POSITION)

	def distribute_segments(self): # Distributes Segments Among Turtles
		for turtle_name in self.turtle_names:
			self.Turtle_Segments[turtle_name] = [] # Create empty list of segments for each turtle
		while self.segments: # While segments to give
			for turtle_name in self.turtle_names:
				if not self.segments:
					break
				else:
					self.Turtle_Segments[turtle_name].append(self.segments.pop()) # Give segment to turtle

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

	def set_pen_white(self, turtle_name, on=False): # Set the pen color to white
		self.SetPen_Clients[turtle_name].wait_for_service()
		req_pen = SetPen.Request()
		req_pen.r = 255
		req_pen.g = 255
		req_pen.b = 255
		req_pen.width = 2
		req_pen.off = not on # Helps bool logic
		self.SetPen_Clients[turtle_name].call_async(req_pen)

	def calculate_twist(self, goal, pose): # Calculate the movement from given pose to goal
		vel_msg = Twist()
		vel_msg.linear.x = 0.0
		vel_msg.angular.z = 0.0

		# If I stopped turning and went forward, how close would I be to the goal?
		if theory_distance(goal, pose) >= 1/self.accuracy:
			# Stop moving, Start rotating
			vel_msg.linear.x = 0.0
			vel_msg.angular.z = angular_vel(goal, pose)
		else:
			# Stop rotating
			vel_msg.angular.z = 0.0
			if euclidean_distance(goal, pose) >= 1/self.accuracy:
				# Start moving
				vel_msg.linear.x = linear_vel(goal, pose, 2)
			else:
				vel_msg.linear.x = 0.0

		return vel_msg

	def publish_twists(self, turtle_name, twist): # Publish a twist to a turtle's publisher
		self.Twist_Publishers[turtle_name].publish(twist)
		rclpy.spin_once(self)

	def turtles_go_to_points(self, turtles, num, pen_toggle): # Tell the turtles to go to their respective next point
		#num = num if num >= 1 else -1 # Check for leftover lines
		Instructions = {}
		for turtle_name in turtles:
			self.set_pen_white(turtle_name, on=pen_toggle) 	# Set all turtles' pens on or off
		for TICK in range(self.accuracy-200): 				# Give all of the turtles the same amount of time to get to their assigned point
			#print(f"tick:{TICK}")
			for turtle_name in turtles: 					# Calculate how to go to point
				if not pen_toggle: 								# Turtle is going to the start of a line segment (Don't Draw)
					Instructions[turtle_name] = self.calculate_twist(self.Turtle_Segments[turtle_name][num].start, self.Turtle_Poses[turtle_name])
				else: 											# Turtle is going to the end of the line segment (Do Draw)
					Instructions[turtle_name] = self.calculate_twist(self.Turtle_Segments[turtle_name][num].end, self.Turtle_Poses[turtle_name])
			for turtle_name in turtles: 
				self.publish_twists(turtle_name, Instructions[turtle_name]) # Tell them to go point

	def direct_multiple_turtles(self, num_points): # Draw the TAMU Logo!
		for num in range(num_points):
			#print(f"STARTING! {num}")
			self.turtles_go_to_points(self.turtle_names, num, pen_toggle=False) # Turtles go to segment start
			#print(f"ENDING! {num}")
			self.turtles_go_to_points(self.turtle_names, num, pen_toggle=True)  # Turtles go to segment end
			
		for turtle_name in self.turtle_names[self.left_over_segments:]: # Remove finished turtles
			self.remove_turtle(turtle_name)

		self.turtles_go_to_points(self.turtle_names[:self.left_over_segments], -1, False) # Turtles go to segment start
		self.turtles_go_to_points(self.turtle_names[:self.left_over_segments], -1, True)  # Turtles go to segment end

		for turtle_name in self.turtle_names[:self.left_over_segments]:
			self.remove_turtle(turtle_name)

	def remove_turtle(self, turtle_name): # Kill a turtle :(
		self.client_kill = self.create_client(Kill, "/kill") # Client for kill service
		self.client_kill.wait_for_service()
		req_kill = Kill.Request()
		req_kill.name = turtle_name
		future_kill = self.client_kill.call_async(req_kill)
		rclpy.spin_until_future_complete(self, future_kill)
		
def main(args=None):
	rclpy.init(args=args)
	node_name = "turtle_controller"

	controller = DrawTAM(node_name)
	controller.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()

