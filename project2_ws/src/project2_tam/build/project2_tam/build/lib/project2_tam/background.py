import rclpy
from turtlesim.msg import Color

def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node('change_background_color')

    publisher = node.create_publisher(Color, '/turtlesim/set_background_color', 10)

    msg = Color()
    msg.r = 80
    msg.g = 0
    msg.b = 0

    node.get_logger().info('Changing background color to maroon...')
    publisher.publish(msg)

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

