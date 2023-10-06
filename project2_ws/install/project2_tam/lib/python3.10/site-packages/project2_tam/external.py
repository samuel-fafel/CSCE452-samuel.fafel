import rclpy
from rclpy.node import Node
import GoToGoal

def main(args=None):
    rclpy.init(args=args)
    nodes = []
    number = 3
    for i in range(number):
        node = GoToGoal.TurtleGoToGoal()
        nodes.append(node)
        #node.turtle_name = "turtle1"
        node.goal.x = 0.0
        node.goal.y = 0.0
        node.goal.theta = 3.1415 / 4
        rclpy.spin(node)
        rclpy.shutdown()

if __name__ == '__main__':
    main()
