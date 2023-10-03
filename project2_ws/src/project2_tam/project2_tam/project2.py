import rclpy
from rclpy.node import Node
from turtlesim.srv import TeleportAbsolute
import tam_logo as tam
from turtlesim.srv import SetPen
from std_srvs.srv import Empty
from rcl_interfaces.srv import SetParameters
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import math
import os

class TurtleTeleporter(Node):
	def __init__(self, coordinates):
		super().__init__('turtle_teleporter')
		self.coordinates = coordinates
		self.num_turtles = self.declare_parameter('num_turtles', 2).value
		self.turtle_names = [f'turtle{i}' for i in range(1, self.num_turtles + 1)]
		self.client_teleport = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
		self.set_param_client = self.create_client(SetParameters, '/turtlesim/set_parameters')
		self.client_clear = self.create_client(Empty, '/clear')  # Create a client for the /clear service
		self.initialize_simulator()
		self.set_colors()
		self.teleport_turtle()

	def initialize_simulator(self):
		self.get_logger().info('Waiting for /clear service...')
		self.client_clear.wait_for_service()
		self.get_logger().info('/clear service available.')
		
		req_clear = Empty.Request()
		self.client_clear.call_async(req_clear)  # Call the /clear service
		self.set_background_color()
		self.get_logger().info('Simulator reset.')
		
	def set_background_color(self):
		# Use ros2 command line tool to set the parameters for turtlesim
		os.system("ros2 param load /turtlesim background_params.txt")
		
	def set_colors(self):
		# Set the pen color
		self.client_set_pen = self.create_client(SetPen, '/turtle1/set_pen')
		self.get_logger().info('Waiting for /turtle1/set_pen service...')
		self.client_set_pen.wait_for_service()
		req_pen = SetPen.Request()
		req_pen.r = 255
		req_pen.g = 255
		req_pen.b = 255
		req_pen.width = 2
		req_pen.off = 0
		self.client_set_pen.call_async(req_pen)
		self.get_logger().info('Pen color and width set.')

	def teleport_turtle(self):
		self.get_logger().info('Waiting for teleport_absolute service...')
		self.client_teleport.wait_for_service()
		self.get_logger().info('teleport_absolute service available.')
		
		for coord in self.coordinates:
			req = TeleportAbsolute.Request()
			req.x = coord.x
			req.y = coord.y
			req.theta = 0.0

			self.get_logger().info(f"Teleporting turtle to ({req.x}, {req.y})")
			future = self.client_teleport.call_async(req)
			rclpy.spin_until_future_complete(self, future)
			
			if future.result() is not None:
				self.get_logger().info("Teleportation successful!")
			else:
				self.get_logger().error("Failed to call service teleport_absolute")
		self.remove_turtles()
				
	def remove_turtles(self):
		for turtle_name in self.turtle_names:
			self.get_logger().info(f'Removing {turtle_name}')
			cmd = f'ros2 service call /kill turtlesim/srv/Kill "name: \'{turtle_name}\'"'
			os.system(cmd)
		
def main():
	rclpy.init()

	# List of coordinates to teleport to
	coordinates = tam.my_points
	
	teleported = TurtleTeleporter(coordinates)
	
	try:
		rclpy.spin(teleported)
	except KeyboardInterrupt:
		pass

	teleported.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()

