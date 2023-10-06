import rclpy
from rclpy.node import Node
from turtlesim.srv import TeleportAbsolute ##### REMOVE #####
from turtlesim.srv import SetPen
from turtlesim.srv import Spawn
from turtlesim.srv import Kill
from std_srvs.srv import Empty
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import os
import project2_tam.tam_logo as tam

class TurtleTeleporter(Node):
	def __init__(self, node_name):
		super().__init__(node_name)
		self.control_node = rclpy.create_node('turtle_control')
		self.coordinates = tam.my_points
		self.segments = tam.my_lines
		self.num_turtles = self.declare_parameter('num_turtles', 4).value

		self.turtle_names = [f'turtle{i}' for i in range(1, self.num_turtles + 1)] # Name the turtles 

		self.initialize_simulator() # Prep turtlesim and turtles

		# Initialize all turtles with publishers and subscribers
		self.SetPen_clients = {}
		self.Twist_publishers = {}
		self.Pose_subscriptions = {}
		self.Teleport_clients = {} ##### REMOVE #####
		self.turtle_pose = None
		for turtle_name in self.turtle_names:
			self.spawn_turtle(turtle_name) # Spawn turtle

			self.SetPen_clients[turtle_name]     = self.create_client(SetPen, f"/{turtle_name}/set_pen")
			self.Twist_publishers[turtle_name]   = self.create_publisher(Twist, f'/{turtle_name}/cmd_vel', 10)
			self.Pose_subscriptions[turtle_name] = self.control_node.create_subscription(Pose, f'/{turtle_name}/pose', self.pose_callback, 10)
			self.Teleport_clients[turtle_name]   = self.create_client(TeleportAbsolute, f'/{turtle_name}/teleport_absolute') ##### REMOVE #####

			self.set_pen_white(turtle_name)

		for turtle_name in self.turtle_names:
			self.teleport_turtle(turtle_name)
			self.initialize_simulator()

	def pose_callback(self, msg):
		self.turtle_pose = msg

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

	def set_pen_white(self, turtle_name, on_off=True): # Set the pen color to white
		self.SetPen_clients[turtle_name].wait_for_service()
		req_pen = SetPen.Request()
		req_pen.r = 255
		req_pen.g = 255
		req_pen.b = 255
		req_pen.width = 2
		req_pen.off = not on_off
		self.SetPen_clients[turtle_name].call_async(req_pen)

	def teleport_turtle(self, turtle_name): ##### REMOVE #####
		self.Teleport_clients[turtle_name].wait_for_service()
		for coord in self.coordinates:
			req = TeleportAbsolute.Request()
			req.x = coord.x
			req.y = coord.y
			req.theta = 0.0

			self.get_logger().info(f"Teleporting turtle to ({req.x}, {req.y})")
			future = self.Teleport_clients[turtle_name].call_async(req)
			rclpy.spin_until_future_complete(self, future)
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

	# List of coordinates to teleport to
	node_name = "turtle_teleporter"
	
	teleported = TurtleTeleporter(node_name)
	#rclpy.spin(teleported)

	teleported.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()

