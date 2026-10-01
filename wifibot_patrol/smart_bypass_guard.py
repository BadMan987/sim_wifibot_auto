#!/usr/bin/env python3

import subprocess
import os
import time
import math
import numpy as np
import cv2
import yaml
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from sensor_msgs.msg import Image
from nav_msgs.msg import OccupancyGrid


class SmartBypassGuard(Node):
    """
    Nœud ROS 2 gérant la garde de contournement intelligent (Smart Bypass Guard).
    Il surveille les obstacles dynamiques via la caméra de profondeur, filtre les murs statiques,
    évalue la sécurité des trajectoires et déclenche une ré-planification globale si nécessaire.
    """

    def __init__(self):
        """
        Initialise le nœud ROS 2, les abonnements, les publications, les variables d'état 
        et lance automatiquement le script de suivi de trajectoire en arrière-plan.
        """
        super().__init__(
            'smart_bypass_guard',
            parameter_overrides=[
                rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)
            ]
        )

        self.bridge = CvBridge()
        self.latest_cmd = Twist()

        # Position actuelle du robot
        self.rx = None
        self.ry = None
        self.yaw = None

        # Cache de la distance minimale avant de la caméra de profondeur
        self.center_dist = float('inf')

        # Cache des données de carte en temps réel RTAB-Map
        self.current_map = None
        self.map_resolution = 0.05
        self.map_origin_x = 0.0
        self.map_origin_y = 0.0
        self.map_width = 0
        self.map_height = 0

        # Variable de référence de la carte statique (utilisée pour filtrer les murs fixes)
        self.static_grid = None
        self.static_resolution = 0.05
        self.static_origin = [0.0, 0.0, 0.0]
        self.static_width = 0
        self.static_height = 0
        self.load_static_map_reference()

        # Dimensions physiques réelles du robot (longueur 0,32 m, largeur 0,37 m)
        self.robot_length = 0.32
        self.robot_width = 0.37

        # Variables de contrôle d'état
        self.is_replanning = False
        self.blocked_start_time = None
        self.last_replan_timestamp = 0.0

        # Obtenir le chemin du fichier de trajectoire globale
        self.path_file = "/home/yz0000/wifibot_ws/src/wifibot_navigation/wifibot_navigation/world_path.npy"
        if not os.path.exists(self.path_file):
            self.path_file = "/home/yz0000/wifibot_ws/src/wifibot_patrol/wifibot_patrol/world_path.npy"
        self.load_original_goal()

        # 1. Lancer automatiquement path_follower.py en arrière-plan
        follower_script = "/home/yz0000/wifibot_ws/src/wifibot_patrol/wifibot_patrol/path_follower.py"
        if not os.path.exists(follower_script):
            follower_script = "/home/yz0000/wifibot_ws/src/wifibot_navigation/wifibot_navigation/path_follower.py"

        self.get_logger().info(f"Launching path_follower from: {follower_script}")
        self.follower_process = subprocess.Popen(["python3", follower_script])

        # 2. Abonnements et publications ROS
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/rtabmap/localization_pose',
            self.pose_callback,
            10
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/rtabmap/map',
            self.map_callback,
            10
        )
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
        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info("🚀 Smart Bypass Guard (Optimized Threshold Strategy) initialized.")

    def load_static_map_reference(self):
        """
        Charge le fichier YAML et l'image de la carte statique de référence 
        afin de pouvoir distinguer les murs fixes des obstacles dynamiques.
        """
        yaml_file = "/home/yz0000/wifibot_ws/src/wifibot_navigation/maps/indoor/map.yaml"
        try:
            with open(yaml_file, "r") as file:
                config = yaml.safe_load(file)
            self.static_resolution = float(config["resolution"])
            self.static_origin = [float(v) for v in config["origin"]]
            
            image_path = os.path.join(os.path.dirname(yaml_file), config["image"])
            static_img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
            
            if static_img is not None:
                self.static_grid = np.zeros(static_img.shape, dtype=np.uint8)
                self.static_grid[static_img < 100] = 1
                self.static_grid[static_img >= 100] = 0
                self.static_height, self.static_width = static_img.shape
        except Exception as e:
            self.static_grid = None

    def load_original_goal(self):
        """
        Lit le fichier de trajectoire globale pour extraire la position finale de destination (goal)
        ainsi que l'orientation cible (yaw).
        """
        try:
            data = np.load(self.path_file, allow_pickle=True).item()
            path = data["path"]
            self.final_goal_x, self.final_goal_y = path[-1]
            self.final_goal_yaw = data.get("goal_yaw", 0.0)
        except Exception as e:
            self.final_goal_x, self.final_goal_y = 0.0, 0.0
            self.final_goal_yaw = 0.0

    def pose_callback(self, msg):
        """
        Fonction de rappel (callback) pour la pose de localisation du robot.
        Met à jour les coordonnées (rx, ry) et l'orientation (yaw).
        """
        self.rx = msg.pose.pose.position.x
        self.ry = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    def map_callback(self, msg):
        """
        Fonction de rappel (callback) pour la carte d'occupation en temps réel RTAB-Map.
        Met à jour les données de la grille d'occupation et ses métadonnées.
        """
        self.current_map = msg.data
        self.map_resolution = msg.info.resolution
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_origin_x = msg.info.origin.position.x
        self.map_origin_y = msg.info.origin.position.y

    def is_static_wall(self, wx, wy):
        """
        Vérifie si une coordonnée du monde (wx, wy) correspond à un mur statique connu
        en utilisant la grille de référence de la carte statique.
        """
        if self.static_grid is not None:
            s_mx = int((wx - self.static_origin[0]) / self.static_resolution)
            s_my = int(self.static_height - ((wy - self.static_origin[1]) / self.static_resolution))
            if 0 <= s_mx < self.static_width and 0 <= s_my < self.static_height:
                if self.static_grid[s_my, s_mx] == 1:
                    return True
        return False

    def check_trajectory_safety(self, test_v, test_w):
        """
        Simule l'empreinte et la trajectoire future du robot sur plusieurs pas de temps 
        et vérifie si elle entre en collision avec des obstacles sur la carte actuelle.
        Retourne True si la trajectoire est sûre, False sinon.
        """
        if self.current_map is None or self.rx is None or self.yaw is None:
            return True

        sim_time_steps = [0.2, 0.4, 0.6, 0.8]
        car_half_length = self.robot_length / 2.0
        car_half_width = self.robot_width / 2.0
        virtual_inflation_cells = 0

        for dt in sim_time_steps:
            if abs(test_w) > 1e-5:
                pred_yaw = self.yaw + test_w * dt
                pred_x = self.rx + (test_v / test_w) * (math.sin(pred_yaw) - math.sin(self.yaw))
                pred_y = self.ry - (test_v / test_w) * (math.cos(pred_yaw) - math.cos(self.yaw))
            else:
                pred_yaw = self.yaw
                pred_x = self.rx + test_v * dt * math.cos(self.yaw)
                pred_y = self.ry + test_v * dt * math.sin(self.yaw)

            corners = [
                ( car_half_length,  car_half_width),
                ( car_half_length, -car_half_width),
                (-car_half_length,  car_half_width),
                (-car_half_length, -car_half_width)
            ]

            for cx_local, cy_local in corners:
                wx = pred_x + cx_local * math.cos(pred_yaw) - cy_local * math.sin(pred_yaw)
                wy = pred_y + cx_local * math.sin(pred_yaw) + cy_local * math.cos(pred_yaw)

                base_mx = int((wx - self.map_origin_x) / self.map_resolution)
                base_my = int((wy - self.map_origin_y) / self.map_resolution)

                for dx in range(-virtual_inflation_cells, virtual_inflation_cells + 1):
                    for dy in range(-virtual_inflation_cells, virtual_inflation_cells + 1):
                        mx = base_mx + dx
                        my = base_my + dy

                        if 0 <= mx < self.map_width and 0 <= my < self.map_height:
                            index = my * self.map_width + mx
                            if self.current_map[index] > 50:
                                return False

        return True

    def depth_callback(self, msg):
        """
        Fonction de rappel (callback) pour l'image de profondeur de la caméra.
        Filtre les murs statiques pour ne conserver que les obstacles dynamiques proches 
        et met à jour la distance minimale centrale.
        """
        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            h, w = depth_image.shape
            ymin, ymax = int(h * 0.60), int(h * 0.85)
            w_third = w // 3
            roi = depth_image[ymin:ymax, w_third:2*w_third]

            if self.rx is None or self.yaw is None:
                self.center_dist = self.get_min_valid_depth(roi)
                return

            valid_dynamic_depths = []
            step_px = 4
            for r in range(0, roi.shape[0], step_px):
                for c in range(0, roi.shape[1], step_px):
                    d = roi[r, c]
                    if np.isnan(d) or np.isinf(d) or d <= 0.0:
                        continue

                    forward_dist = float(d)
                    if forward_dist < 2.5:
                        cam_offset_x = 0.15
                        wx = self.rx + (forward_dist + cam_offset_x) * math.cos(self.yaw)
                        wy = self.ry + (forward_dist + cam_offset_x) * math.sin(self.yaw)

                        if self.is_static_wall(wx, wy):
                            continue

                        valid_dynamic_depths.append(forward_dist)

            if len(valid_dynamic_depths) > 0:
                self.center_dist = float(np.percentile(valid_dynamic_depths, 5))
            else:
                self.center_dist = float('inf')
        except Exception as e:
            pass

    def get_min_valid_depth(self, roi):
        """
        Extrait la distance valide minimale (via percentile) d'une région d'intérêt (ROI) de profondeur.
        """
        valid = roi[~np.isnan(roi) & ~np.isinf(roi) & (roi > 0.0)]
        if valid.size > 0:
            return float(np.percentile(valid, 5))
        return float('inf')

    def cmd_callback(self, msg):
        """
        Fonction de rappel (callback) pour les commandes de vitesse brutes (`/cmd_vel_raw`).
        Met en cache la dernière commande reçue.
        """
        self.latest_cmd = msg

    def get_path_steer_command(self):
        """
        Calcule une commande de direction (vitesse angulaire w) basée sur la trajectoire globale 
        en utilisant une approche de type 'look-ahead' (poursuite de chemin).
        """
        try:
            if not os.path.exists(self.path_file):
                return 0.0
            data = np.load(self.path_file, allow_pickle=True).item()
            path = data["path"]
            if len(path) == 0:
                return 0.0

            min_dist = float('inf')
            closest_idx = 0
            for i, pt in enumerate(path):
                dist = math.hypot(pt[0] - self.rx, pt[1] - self.ry)
                if dist < min_dist:
                    min_dist = dist
                    closest_idx = i

            lookahead_dist = 0.5
            target_pt = path[-1]
            for i in range(closest_idx, len(path)):
                dist = math.hypot(path[i][0] - self.rx, path[i][1] - self.ry)
                if dist >= lookahead_dist:
                    target_pt = path[i]
                    break

            target_yaw = math.atan2(target_pt[1] - self.ry, target_pt[0] - self.rx)
            yaw_error = math.atan2(math.sin(target_yaw - self.yaw), math.cos(target_yaw - self.yaw))
            steer_w = max(-1.0, min(1.0, 1.5 * yaw_error))
            return steer_w
        except Exception as e:
            return 0.8

    def trigger_dynamic_replan(self):
        """
        Déclenche une ré-planification globale du chemin via Hybrid A* 
        et redémarre à chaud le processus de suivi de trajectoire.
        """
        current_time = time.time()
        if self.is_replanning or (current_time - self.last_replan_timestamp < 10.0):
            return

        self.is_replanning = True
        self.last_replan_timestamp = current_time
        self.get_logger().warn(">>> [Escape / Vision] Déclenchement de la ré-planification globale Hybrid A*... <<<")

        stop_cmd = Twist()
        self.cmd_pub.publish(stop_cmd)

        astar_script = "/home/yz0000/wifibot_ws/src/wifibot_patrol/wifibot_patrol/astar.py"
        if not os.path.exists(astar_script):
            astar_script = "/home/yz0000/wifibot_ws/src/wifibot_navigation/wifibot_navigation/astar.py"

        success = False
        try:
            cmd_str = f"python3 {astar_script} {self.final_goal_x} {self.final_goal_y} {self.final_goal_yaw}"
            result = subprocess.run(cmd_str, shell=True, timeout=6)
            if result.returncode == 0:
                success = True
            else:
                raise Exception("Goal unreachable.")
        except Exception:
            pass

        try:
            if success:
                if hasattr(self, 'follower_process') and self.follower_process.poll() is None:
                    self.follower_process.terminate()
                    self.follower_process.wait(timeout=2.0)
                follower_script = os.path.dirname(astar_script) + "/path_follower.py"
                self.follower_process = subprocess.Popen(["python3", follower_script])
                self.get_logger().info("🚀 Le suiveur de trajectoire a été redémarré à chaud !")
        except Exception as e:
            pass

        self.blocked_start_time = None
        self.is_replanning = False

    def control_loop(self):
        """
        Boucle de contrôle principale exécutée périodiquement par un timer.
        Évalue la distance aux obstacles, gère les comportements d'évitement, 
        d'urgence, de repli (recul/oscillation) et publie la commande finale sur `/cmd_vel`.
        """
        twist = Twist()
        current_time = time.time()
        
        desired_v = self.latest_cmd.linear.x
        desired_w = self.latest_cmd.angular.z

        # 🚀 Optimisation : Réduction de la distance d'alerte de sécurité pour éviter une sensibilité excessive à proximité des murs
        danger_distance = 0.25   # Réduit de 0,35 m à 0,25 m (ligne de défense déclenchée à 25 cm)
        warning_distance = 0.45  # Réduit de 0,60 m à 0,45 m (décélération à partir de 45 cm)

        if self.center_dist < danger_distance:
            self.get_logger().warn(f"🚨 [Vision Guard] Obstacle dynamique trop proche ({self.center_dist:.2f}m), calcul de la direction guidée par la trajectoire...", throttle_duration_sec=1.0)
            
            is_rear_safe = self.check_trajectory_safety(-0.12, 0.0)

            if is_rear_safe:
                steer_escape_w = self.get_path_steer_command()
                twist.linear.x = -0.12
                twist.angular.z = steer_escape_w
                self.get_logger().info(f"🟢 [Escape] Arrière sûr, exécution de [recul + direction guidée par la trajectoire] pour se dégager (w={steer_escape_w:.2f})", throttle_duration_sec=1.0)
            else:
                steer_escape_w = self.get_path_steer_command()
                twist.linear.x = 0.0
                twist.angular.z = steer_escape_w if abs(steer_escape_w) > 0.2 else 1.0
                self.get_logger().warn("🔴 [Escape] Bloqué à l'avant et à l'arrière, exécution de [oscillation sur place guidée par la trajectoire] pour se dégager...", throttle_duration_sec=1.0)

            self.cmd_pub.publish(twist)

            if not self.is_replanning:
                self.trigger_dynamic_replan()
            return

        if desired_v <= 0.01:
            self.cmd_pub.publish(self.latest_cmd)
            self.blocked_start_time = None
            return

        if self.center_dist < warning_distance:
            scale = max(0.2, (self.center_dist - danger_distance) / (warning_distance - danger_distance))
            desired_v *= scale

        candidates = [
            (desired_v, desired_w),
            (desired_v * 0.9, desired_w + 0.3),
            (desired_v * 0.9, desired_w - 0.3),
            (desired_v * 0.7, desired_w + 0.8),
            (desired_v * 0.7, desired_w - 0.8),
            (0.06, 0.3),  
            (0.0, 1.0),  
            (0.0, -1.0)  
        ]

        best_v = 0.0
        best_w = 0.0
        found_safe_candidate = False

        for cand_v, cand_w in candidates:
            if self.check_trajectory_safety(cand_v, cand_w):
                best_v = cand_v
                best_w = cand_w
                found_safe_candidate = True
                break

        is_breakout_mode = (current_time - self.last_replan_timestamp < 6.0)

        if found_safe_candidate:
            self.blocked_start_time = None
            twist.linear.x = best_v
            twist.angular.z = best_w
        elif is_breakout_mode:
            self.blocked_start_time = None
            twist.linear.x = 0.05
            twist.angular.z = desired_w * 0.5
            self.get_logger().info("🔥 [Breakout] Mode de percée forcée activé : Poussée vers l'avant forcée !", throttle_duration_sec=1.0)
        else:
            self.get_logger().warn("[Local Guard] Toutes les trajectoires candidates locales présentent un risque de collision ! Arrêt d'urgence...", throttle_duration_sec=1.0)
            twist.linear.x = 0.0
            twist.angular.z = 0.0

            if self.blocked_start_time is None:
                self.blocked_start_time = current_time
            else:
                if current_time - self.blocked_start_time > 1.0 and not self.is_replanning:
                    self.trigger_dynamic_replan()

        self.cmd_pub.publish(twist)

    def destroy_node(self):
        """
        Méthode de nettoyage appelée lors de la destruction du nœud.
        S'assure de terminer proprement le processus de suivi de trajectoire en arrière-plan.
        """
        if hasattr(self, 'follower_process') and self.follower_process.poll() is None:
            self.follower_process.terminate()
            self.follower_process.wait()
        super().destroy_node()


def main():
    """
    Fonction principale (entry point) du script.
    Initialise rclpy, instancie le nœud SmartBypassGuard, lance la boucle de spin 
    et gère l'arrêt propre lors d'une interruption clavier.
    """
    rclpy.init()
    node = SmartBypassGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
