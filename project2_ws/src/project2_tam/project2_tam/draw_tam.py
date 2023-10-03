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
        
        self.draw_logo()

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

def main(args=None):
    rclpy.init(args=args)
    node = TamLogoDrawer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

