import rclpy
from rclpy.node import Node
from turtlesim.srv import TeleportAbsolute ##### REMOVE #####
from turtlesim.srv import SetPen
from turtlesim.srv import Spawn
from turtlesim.srv import Kill
from std_srvs.srv import Empty
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import math
import os
import tam_logo as tam

class TurtleTeleporter(Node):
	def __init__(self, coordinates, segments, num_turtles):
		super().__init__('turtle_teleporter')
		self.coordinates = coordinates ##### REMOVE #####
		self.segments = segments # Lines to draw
		self.num_turtles = num_turtles # Count the turtles
		self.turtle_names = [f'turtle{i}' for i in range(1, self.num_turtles + 1)] # Name the turtles 

		self.client_clear = self.create_client(Empty, '/clear')  # Client for /clear service
		self.client_teleport = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute') # Client for teleport service

		self.initialize_simulator() # Prep turtlesim and turtles

		# Create publishers for cmd_vel topics of each turtle
		self.turtle_publishers = {}
		for turtle_name in self.turtle_names:
			self.turtle_publishers[turtle_name] = self.create_publisher(
				Twist, f'/{turtle_name}/cmd_vel', 10
				)

		self.teleport_turtle()

	def initialize_simulator(self):
		self.client_clear.wait_for_service()
		
		req_clear = Empty.Request()
		self.client_clear.call_async(req_clear) # Clear Turtlesim
		self.get_logger().info('Simulator cleared.')

		os.system("ros2 param load /turtlesim background_params.txt") # Set background color to Aggie Maroon
		self.spawn_turtles() # Spawn Turtles
		for turtle in self.turtle_names:
			self.set_pen(turtle) # Set turtles' pen colors to white
		
	def spawn_turtles(self):
		for name in self.turtle_names:
			self.client_spawn = self.create_client(Spawn, 'spawn')
			req_spawn = Spawn.Request()
			req_spawn.name = name
			req_spawn.x = 5.544445
			req_spawn.y = 5.544445
			req_spawn.theta = 0.0
			self.client_spawn.call_async(req_spawn)

	def set_pen(self, turtle_name):
		# Set the pen color
		self.client_set_pen = self.create_client(SetPen, f"/{turtle_name}/set_pen") # Client for set_pen service
		self.client_set_pen.wait_for_service()
		req_pen = SetPen.Request()
		req_pen.r = 255
		req_pen.g = 255
		req_pen.b = 255
		req_pen.width = 2
		req_pen.off = 0
		self.client_set_pen.call_async(req_pen)
        
	def teleport_turtle(self):
		self.client_teleport.wait_for_service()
		
		for coord in self.coordinates:
			req = TeleportAbsolute.Request()
			req.x = coord.x
			req.y = coord.y
			req.theta = 0.0

			self.get_logger().info(f"Teleporting turtle to ({req.x}, {req.y})")
			future = self.client_teleport.call_async(req)
			rclpy.spin_until_future_complete(self, future)
			
			if future.result() is None:
				self.get_logger().error("Failed to call service teleport_absolute")
		self.remove_turtles()
				
	def remove_turtles(self):
		self.client_kill = self.create_client(Kill, "/kill") # Client for kill service
		for name in self.turtle_names:
			self.client_kill.wait_for_service()
			req_kill = Kill.Request()
			req_kill.name = name
			future_kill = self.client_kill.call_async(req_kill)
			rclpy.spin_until_future_complete(self, future_kill)
		
def main():
	rclpy.init()

	# List of coordinates to teleport to
	coordinates = tam.my_points
	segments = tam.my_lines
	num_turtles = 4
	
	teleported = TurtleTeleporter(coordinates, segments, num_turtles)
	#rclpy.spin(teleported)

	teleported.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()

