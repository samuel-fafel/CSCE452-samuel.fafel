import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
import math

class TamLogoDrawer(Node):
    def __init__(self):
        super().__init__('tam_logo_drawer')
        self.num_turtles = self.declare_parameter('num_turtles', 2).value
        self.turtle_names = [f'turtle{i}' for i in range(1, self.num_turtles + 1)]
        self.line_segments = []  # Fill this with your list of line segments

        # Initialize turtlesim
        self.clear_and_set_background()
        self.turtles_spawn()
        
        # Create publishers for cmd_vel topics of each turtle
        self.turtle_publishers = {}
        for turtle_name in self.turtle_names:
            self.turtle_publishers[turtle_name] = self.create_publisher(
                Twist, f'/{turtle_name}/cmd_vel', 10
            )
        
        self.draw_logo()

    def clear_and_set_background(self):
        clear = self.create_client(Empty, '/reset')
        while not clear.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service /clear not available, waiting again...')
        request = Empty.Request()
        future = clear.call_async(request)
        rclpy.spin_until_future_complete(self, future)

    def turtles_spawn(self):
        # Calculate the starting positions for turtles based on the number of turtles
        start_x = 5.0
        start_y = 5.0
        start_theta = 0.0
        angle_increment = 2 * math.pi / self.num_turtles

        for turtle_name in self.turtle_names:
            self.spawn_turtle(turtle_name, start_x, start_y, start_theta)
            start_theta += angle_increment

    def spawn_turtle(self, name, x, y, theta):
        self.get_logger().info(f'Spawning {name} at ({x}, {y}, {theta})')
        cmd = f'ros2 service call /spawn turtlesim/srv/Spawn "name: \'{name}\' x: {x} y: {y} theta: {theta}"'
        os.system(cmd)

    def draw_logo(self):
        # Calculate the number of line segments for each turtle
        num_segments_per_turtle = len(self.line_segments) // self.num_turtles
        remaining_segments = len(self.line_segments) % self.num_turtles

        # Distribute line segments among turtles
        segments_for_turtles = []
        for i in range(self.num_turtles):
            num_segments = num_segments_per_turtle + (1 if i < remaining_segments else 0)
            segments = self.line_segments[i * num_segments:(i + 1) * num_segments]
            segments_for_turtles.append(segments)

        # Calculate and publish cmd_vel messages for each turtle to draw the logo
        rate = self.create_rate(10)  # You may adjust the publishing rate as needed
        for turtle_name, segments in zip(self.turtle_names, segments_for_turtles):
            for segment in segments:
                # Calculate and publish cmd_vel messages to move the turtle
                # You need to implement the logic to control the turtle here
                pass

        # Remove turtles from the simulation
        self.remove_turtles()

    def remove_turtles(self):
        for turtle_name in self.turtle_names:
            self.get_logger().info(f'Removing {turtle_name}')
            cmd = f'ros2 service call /kill turtlesim/srv/Kill "name: \'{turtle_name}\'"'
            os.system(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = TamLogoDrawer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

