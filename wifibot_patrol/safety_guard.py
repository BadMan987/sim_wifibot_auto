#!/usr/bin/env python3

import subprocess
import os
import time
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

        self.obstacle_stop_dist = 0.30    
        self.front_min_distance = float('inf')
        self.bridge = CvBridge()
        self.latest_cmd = Twist()

        self.blocked_start_time = None
        self.is_replanning = False

        self.path_file = "/home/yz0000/wifibot_ws/src/wifibot_navigation/wifibot_navigation/world_path.npy"
        if not os.path.exists(self.path_file):
            self.path_file = "/home/yz0000/wifibot_ws/src/wifibot_patrol/wifibot_patrol/world_path.npy"
        
        self.load_original_goal()

        # Lancer path_follower.py
        follower_script = "/home/yz0000/wifibot_ws/src/wifibot_patrol/wifibot_patrol/path_follower.py"
        if not os.path.exists(follower_script):
            follower_script = "/home/yz0000/wifibot_ws/src/wifibot_navigation/wifibot_navigation/path_follower.py"

        self.get_logger().info(f"Launching path_follower from: {follower_script}")
        self.follower_process = subprocess.Popen(["python3", follower_script])

        # S'abonner à la carte de profondeur de la ZED 2i
        self.depth_sub = self.create_subscription(
            Image,
            '/camera/zed2i/depth/image_raw',
            self.depth_callback,
            10
        )

        self.cmd_sub = self.create_subscription(
            Twist,
            '/cmd_vel_raw',
            self.cmd_callback,
            10
        )

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.05, self.safety_loop)
        self.get_logger().info("Smart SafetyGuard with Dynamic Replanning initialized.")

    def load_original_goal(self):
        try:
            data = np.load(self.path_file, allow_pickle=True).item()
            path = data["path"]
            self.final_goal_x, self.final_goal_y = path[-1]
            self.final_goal_yaw = data.get("goal_yaw", 0.0)
        except Exception as e:
            self.get_logger().error(f"Failed to load original goal: {e}")
            self.final_goal_x, self.final_goal_y = 0.0, 0.0
            self.final_goal_yaw = 0.0

    def depth_callback(self, msg):
        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            h, w = depth_image.shape
            ymin, ymax = int(h * 0.60), int(h * 0.85)
            xmin, xmax = int(w * 0.40), int(w * 0.60)
            roi = depth_image[ymin:ymax, xmin:xmax]
            
            valid_depths = roi[~np.isnan(roi) & ~np.isinf(roi) & (roi > 0.0)]
            if valid_depths.size > 0:
                self.front_min_distance = float(np.percentile(valid_depths, 5))
            else:
                self.front_min_distance = float('inf')
        except Exception as e:
            self.get_logger().warn(f"Failed to process depth image: {e}", throttle_duration_sec=2.0)

    def cmd_callback(self, msg):
        self.latest_cmd = msg

    def trigger_dynamic_replan(self):
        if self.is_replanning:
            return
        self.is_replanning = True
        self.get_logger().warn(">>> Path blocked for too long! Triggering Dynamic Re-planning... <<<")

        stop_cmd = Twist()
        self.cmd_pub.publish(stop_cmd)

        astar_script = "/home/yz0000/wifibot_ws/src/wifibot_patrol/wifibot_patrol/astar.py"
        if not os.path.exists(astar_script):
            astar_script = "/home/yz0000/wifibot_ws/src/wifibot_navigation/wifibot_navigation/astar.py"

        try:
            cmd_str = f"python3 {astar_script} {self.final_goal_x} {self.final_goal_y} {self.final_goal_yaw}"
            result = subprocess.run(cmd_str, shell=True, timeout=20)
            
            if result.returncode == 0:
                self.get_logger().info(">>> Dynamic Re-planning succeeded! Resuming navigation. <<<")
                
                # Terminer en toute sécurité l'ancien processus follower
                if hasattr(self, 'follower_process') and self.follower_process.poll() is None:
                    self.follower_process.terminate()
                    self.follower_process.wait(timeout=2.0)
                
                # Relancer path_follower
                follower_script = os.path.dirname(astar_script) + "/path_follower.py"
                self.follower_process = subprocess.Popen(["python3", follower_script])
            else:
                self.get_logger().error(">>> Dynamic Re-planning failed! <<<")
        except Exception as e:
            self.get_logger().error(f"Re-plan execution error: {e}")

        self.blocked_start_time = None
        self.is_replanning = False

    def safety_loop(self):
        safe_cmd = Twist()
        is_blocked = (self.latest_cmd.linear.x > 0.01 and self.front_min_distance < self.obstacle_stop_dist)

        if is_blocked:
            self.get_logger().warn(
                f"Obstacle ahead! Dist: {self.front_min_distance:.2f}m. Emergency Stop!",
                throttle_duration_sec=1.0
            )
            safe_cmd.linear.x = 0.0
            safe_cmd.angular.z = 0.0

            if self.blocked_start_time is None:
                self.blocked_start_time = time.time()
            else:
                if time.time() - self.blocked_start_time > 3.0 and not self.is_replanning:
                    self.trigger_dynamic_replan()
        else:
            self.blocked_start_time = None
            safe_cmd = self.latest_cmd

        self.cmd_pub.publish(safe_cmd)

    def destroy_node(self):
        if hasattr(self, 'follower_process') and self.follower_process.poll() is None:
            self.follower_process.terminate()
            self.follower_process.wait()
        super().destroy_node()


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
