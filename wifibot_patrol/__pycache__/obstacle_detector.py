#!/usr/bin/env python3

import numpy as np
import cv2
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image


class SafetyGuard(Node):

    def __init__(self):
        super().__init__(
            'safety_guard',
            parameter_overrides=[
                rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)
            ]
        )

        self.obstacle_stop_dist = 0.4   # 障碍物安全阈值 (米)
        self.front_min_distance = float('inf')
        self.bridge = CvBridge()
        self.latest_cmd = Twist()

        # 订阅 ZED 2i 深度图
        self.depth_sub = self.create_subscription(
            Image,
            '/zed/zed_node/depth/depth_registered',
            self.depth_callback,
            10
        )

        # 订阅跟车节点的原始速度指令
        self.cmd_sub = self.create_subscription(
            Twist,
            '/cmd_vel_raw',
            self.cmd_callback,
            10
        )

        # 发布到真实的底盘控制话题
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # 20Hz 仲裁与发布定时器
        self.timer = self.create_timer(0.05, self.safety_loop)
        self.get_logger().info("SafetyGuard node initialized and running.")

    def depth_callback(self, msg):
        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            h, w = depth_image.shape
            # 取正前方中间区域作为 ROI
            ymin, ymax = int(h * 0.4), int(h * 0.7)
            xmin, xmax = int(w * 0.35), int(w * 0.65)
            roi = depth_image[ymin:ymax, xmin:xmax]
            
            valid_depths = roi[~np.isnan(roi) & ~np.isinf(roi) & (roi > 0.0)]
            if valid_depths.size > 0:
                self.front_min_distance = float(np.min(valid_depths))
            else:
                self.front_min_distance = float('inf')
        except Exception as e:
            self.get_logger().warn(f"Failed to process depth image: {e}", throttle_duration_sec=2.0)

    def cmd_callback(self, msg):
        self.latest_cmd = msg

    def safety_loop(self):
        safe_cmd = Twist()

        # 安全仲裁逻辑
        if self.front_min_distance < self.obstacle_stop_dist:
            self.get_logger().warn(
                f"Safety Guard triggered! Obstacle at {self.front_min_distance:.2f}m. Stopping.",
                throttle_duration_sec=1.0
            )
            safe_cmd.linear.x = 0.0
            safe_cmd.angular.z = 0.0
        else:
            # 前方安全，直接转发跟车节点的指令
            safe_cmd = self.latest_cmd

        self.cmd_pub.publish(safe_cmd)


def main():
    rclpy.init()
    node = SafetyGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
