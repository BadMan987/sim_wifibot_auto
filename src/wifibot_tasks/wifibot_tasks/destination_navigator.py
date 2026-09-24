import rclpy
from rclpy.node import Node


class DestinationNavigator(Node):

    def __init__(self):
        super().__init__('destination_navigator')
        self.get_logger().info('Destination navigator started')


def main(args=None):
    rclpy.init(args=args)
    node = DestinationNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
