#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node


class RobotPoseSubscriber(Node):

    def __init__(self):
        super().__init__("robot_pose_subscriber")
        self.pose = None
        # S'abonner directement au topic de localisation de RTAB-Map
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            "/rtabmap/localization_pose",
            self.pose_callback,
            10,
        )

    def pose_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.pose = (x, y)


def get_robot_pose(timeout_sec=10.0):
    """Fonction d'interface pratique fournie à l'extérieur : bloque jusqu'à l'obtention d'une position valide du robot"""
    if not rclpy.ok():
        rclpy.init()

    node = RobotPoseSubscriber()
    start_time = rclpy.clock.Clock().now().nanoseconds / 1e9

    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.pose is not None:
            pose = node.pose
            node.destroy_node()
            return pose

        # Vérification du dépassement de délai (timeout)
        current_time = rclpy.clock.Clock().now().nanoseconds / 1e9
        if current_time - start_time > timeout_sec:
            print("[ERROR] Timeout waiting for /rtabmap/localization_pose!")
            node.destroy_node()
            return None

    node.destroy_node()
    return None


# Point d'entrée pour le test indépendant
if __name__ == "__main__":
    print("Testing robot_pose.py standalone...")
    rclpy.init()
    pose = get_robot_pose(timeout_sec=5.0)
    if pose:
        print(f"\n>>> SUCCESS! Robot Pose -> x = {pose[0]:.3f}, y = {pose[1]:.3f} <<<\n")
    else:
        print("\n>>> FAILED to get pose. Is RTAB-Map running? <<<")
    rclpy.shutdown()
