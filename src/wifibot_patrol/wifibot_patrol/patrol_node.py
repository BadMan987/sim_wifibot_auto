#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

import tf2_ros


def quaternion_to_yaw(x, y, z, w):

    siny_cosp = 2.0 * (w * z + x * y)

    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)

    return math.atan2(siny_cosp, cosy_cosp)


class PatrolNode(Node):

    def __init__(self):

        super().__init__('patrol_node')

        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.tf_buffer = tf2_ros.Buffer()

        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer,
            self
        )

        self.waypoints = [

            (0.4, 0.0),

            (0.4, -0.4),

            (-0.4, 0.0),

            (0.0, 0.0)

        ]

        self.current_goal = 0

        self.timer = self.create_timer(
            0.1,
            self.control_loop
        )

        self.get_logger().info(
            'Autonomous Patrol Started'
        )

    def control_loop(self):

        try:

            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time()
            )

        except Exception:
            return

        x = transform.transform.translation.x
        y = transform.transform.translation.y

        qx = transform.transform.rotation.x
        qy = transform.transform.rotation.y
        qz = transform.transform.rotation.z
        qw = transform.transform.rotation.w

        yaw = quaternion_to_yaw(
            qx,
            qy,
            qz,
            qw
        )

        goal_x, goal_y = self.waypoints[
            self.current_goal
        ]

        dx = goal_x - x
        dy = goal_y - y

        distance = math.sqrt(
            dx * dx +
            dy * dy
        )

        target_yaw = math.atan2(
            dy,
            dx
        )

        yaw_error = target_yaw - yaw

        while yaw_error > math.pi:
            yaw_error -= 2.0 * math.pi

        while yaw_error < -math.pi:
            yaw_error += 2.0 * math.pi

        cmd = Twist()

        #
        # arrive le pointe
        #
        if distance < 0.20:

            self.get_logger().info(
                f'Reached waypoint {self.current_goal}'
            )

            self.current_goal += 1

            if self.current_goal >= len(
                    self.waypoints):

                self.current_goal = 0

                self.get_logger().info(
                    'Restart Patrol Loop'
                )

            return

        #
        # tournee
        #
        if abs(yaw_error) > 0.30:

            cmd.linear.x = 0.0

            cmd.angular.z = (
                0.8 * yaw_error
            )

        #
        # 前进
        #
        else:

            cmd.linear.x = min(
                0.25,
                distance
            )

            cmd.angular.z = (
                0.5 * yaw_error
            )

        self.cmd_pub.publish(cmd)

        self.get_logger().info(
            f'Goal={self.current_goal} '
            f'Pos=({x:.2f},{y:.2f}) '
            f'Target=({goal_x:.2f},{goal_y:.2f}) '
            f'Distance={distance:.2f}'
        )


def main(args=None):

    rclpy.init(args=args)

    node = PatrolNode()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':

    main()
