#!/usr/bin/env python3

import time

import rclpy
import tf2_ros

from rclpy.node import Node


class PoseTest(Node):

    def __init__(self):

        super().__init__("pose_test")

        self.buffer = tf2_ros.Buffer()

        self.listener = tf2_ros.TransformListener(
            self.buffer,
            self
        )


def main():

    rclpy.init()

    node = PoseTest()

    print("Waiting TF...")

    time.sleep(5)

    while rclpy.ok():

        rclpy.spin_once(
            node,
            timeout_sec=0.1
        )

        try:

            trans = node.buffer.lookup_transform(
                "map",
                "base_footprint",
                rclpy.time.Time()
            )

            print("\nSUCCESS")

            print(
                trans.transform.translation.x,
                trans.transform.translation.y
            )

            break

        except Exception as e:

            print(type(e).__name__)
            
            print(repr(e))

            time.sleep(1)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
