import rclpy
from rclpy.node import Node


class CmdVelAdapter(Node):

    def __init__(self):
        super().__init__('cmd_vel_adapter')
        self.get_logger().info('Wifibot cmd_vel adapter started')


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelAdapter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
