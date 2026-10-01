#!/usr/bin/env python3

import math
import os
import numpy as np
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped


class PathFollower(Node):

    def __init__(self):
        super().__init__(
            'path_follower',
            parameter_overrides=[
                rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)
            ]
        )

        # 1. Charger le fichier de chemin
        path_file = "/home/yz0000/wifibot_ws/src/wifibot_navigation/wifibot_navigation/world_path.npy"
        if not os.path.exists(path_file):
            path_file = "/home/yz0000/wifibot_ws/src/wifibot_patrol/wifibot_patrol/world_path.npy"
            if not os.path.exists(path_file):
                self.get_logger().error(f"Cannot find path file: {path_file}")
                raise FileNotFoundError(path_file)

        data = np.load(path_file, allow_pickle=True).item()
        self.path = np.array(data["path"])
        self.goal_yaw = data.get("goal_yaw", 0.0)  
        
        self.get_logger().info(f"Loaded world path: {len(self.path)} waypoints.")
        self.get_logger().info(f"Loaded Goal Yaw: {self.goal_yaw:.2f} rad ({math.degrees(self.goal_yaw):.2f}°)")

        # ==================== Paramètres de suivi local ====================
        self.lookahead_dist = 0.40      
        self.linear_speed = 0.35        
        self.max_angular_speed = 1.5    
        self.goal_tolerance = 0.15      
        # ====================================================================

        self.target_idx = 0
        self.is_finished = False
        self.aligning = True            
        self.is_aligning = False        

        self.rx = None
        self.ry = None
        self.yaw = None
        self.pose_received = False
        self.last_pose_time = self.get_clock().now()

        # Souscrire à la pose de localisation
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/rtabmap/localization_pose',
            self.pose_callback,
            10
        )

        # Publier sur le topic de vitesse de navigation intermédiaire
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_raw', 10)

        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info("Pure PathFollower node initialized.")

    def pose_callback(self, msg):
        self.rx = msg.pose.pose.position.x
        self.ry = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

        self.last_pose_time = self.get_clock().now()
        if not self.pose_received:
            self.get_logger().info(f"First pose received -> x: {self.rx:.3f}, y: {self.ry:.3f}")
            self.pose_received = True

    def control_loop(self):
        if self.is_finished:
            return

        if not self.pose_received or self.rx is None:
            self.get_logger().warn("Waiting for /rtabmap/localization_pose data...", throttle_duration_sec=2.0)
            return

        # Chien de garde de la localisation (Watchdog)
        elapsed_time = (self.get_clock().now() - self.last_pose_time).nanoseconds / 1e9
        if not self.is_aligning and elapsed_time > 1.5:
            self.get_logger().error(f"Localization lost! No pose update for {elapsed_time:.2f}s. Emergency Stop!")
            self.stop_robot()
            return

        rx, ry, yaw = self.rx, self.ry, self.yaw

        # Phase 0 : Alignement initial
        if self.aligning:
            p1 = self.path[0]
            p2 = self.path[min(5, len(self.path) - 1)]
            path_yaw = math.atan2(p2[1] - p1[1], p2[0] - p1[0])

            yaw_error = path_yaw - yaw
            while yaw_error > math.pi: yaw_error -= 2.0 * math.pi
            while yaw_error < -math.pi: yaw_error += 2.0 * math.pi

            cmd = Twist()
            if abs(yaw_error) > 0.1:
                cmd.angular.z = max(min(1.0 * yaw_error, self.max_angular_speed), -self.max_angular_speed)
                self.cmd_pub.publish(cmd)
            else:
                self.get_logger().info(">>> Initial alignment finished! Starting tracking. <<<")
                self.aligning = False
            return

        # Phase 2 : Alignement final
        if self.is_aligning:
            yaw_error = math.atan2(math.sin(self.goal_yaw - yaw), math.cos(self.goal_yaw - yaw))
            cmd = Twist()
            if abs(yaw_error) > 0.08:
                cmd.angular.z = max(min(0.8 * yaw_error, self.max_angular_speed), -self.max_angular_speed)
                self.cmd_pub.publish(cmd)
            else:
                self.get_logger().info(">>> Reached goal and aligned yaw! Shutting down. <<<")
                self.stop_robot()
                self.is_finished = True
            return

        # Phase 1 : Suivi de chemin Pure Pursuit
        goal_x, goal_y = self.path[-1]
        if math.hypot(goal_x - rx, goal_y - ry) < self.goal_tolerance:
            self.get_logger().info(">>> Reached goal position! Aligning orientation. <<<")
            self.is_aligning = True
            return

        # Distance de regard en avant (lookahead) adaptative
        current_lookahead = self.lookahead_dist
        if self.target_idx < len(self.path) - 5:
            p_cur = self.path[self.target_idx]
            p_next = self.path[min(self.target_idx + 4, len(self.path) - 1)]
            p_next2 = self.path[min(self.target_idx + 8, len(self.path) - 1)]
            v1 = (p_next[0] - p_cur[0], p_next[1] - p_cur[1])
            v2 = (p_next2[0] - p_next[0], p_next2[1] - p_next[1])
            dot = v1[0]*v2[0] + v1[1]*v2[1]
            norm_prod = math.hypot(v1[0], v1[1]) * math.hypot(v2[0], v2[1])
            if norm_prod > 1e-5:
                if (1.0 - max(-1.0, min(1.0, dot / norm_prod))) > 0.15:
                    current_lookahead = 0.22

        # Recherche du point de regard en avant (lookahead point)
        target_pt = None
        for i in range(self.target_idx, len(self.path)):
            px, py = self.path[i]
            dx, dy = px - rx, py - ry
            if (math.cos(yaw) * dx + math.sin(yaw) * dy) < 0:
                continue
            if math.hypot(dx, dy) >= current_lookahead:
                target_pt = (px, py)
                self.target_idx = i
                break

        if target_pt is None:
            target_pt = self.path[-1]

        dx, dy = target_pt[0] - rx, target_pt[1] - ry
        local_x = math.cos(yaw) * dx + math.sin(yaw) * dy
        local_y = -math.sin(yaw) * dx + math.cos(yaw) * dy

        cmd = Twist()
        target_heading_error = math.atan2(local_y, local_x)
        if abs(target_heading_error) > 0.7:
            cmd.angular.z = max(min(1.5 * target_heading_error, self.max_angular_speed), -self.max_angular_speed)
        else:
            l_sq = local_x**2 + local_y**2
            if l_sq > 1e-4:
                curvature = (2.0 * local_y) / l_sq
                base_linear_speed = self.linear_speed
                if abs(curvature) < 0.04 and current_lookahead > 0.35:
                    current_linear_speed = base_linear_speed
                else:
                    current_linear_speed = max(0.10, base_linear_speed / (1.0 + 3.5 * abs(curvature)))
                cmd.linear.x = current_linear_speed
                cmd.angular.z = max(min(current_linear_speed * curvature, self.max_angular_speed), -self.max_angular_speed)

        self.cmd_pub.publish(cmd)

    def stop_robot(self):
        cmd = Twist()
        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = PathFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_robot()
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
