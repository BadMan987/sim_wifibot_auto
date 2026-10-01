#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import os
import sys
import cv2
import numpy as np
import heapq
import yaml
import itertools

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from robot_pose import get_robot_pose


class HybridNode:
    """
    Classe représentant un nœud dans l'arbre de recherche Hybrid A*.
    Utilisée pour enregistrer la pose continue, les coûts, la direction de mouvement 
    et l'indice du nœud parent.
    """
    def __init__(self, x, y, yaw, g_cost, h_cost, parent_index, direction=1):
        self.x = x                 # Coordonnée x dans le système mondial (en mètres)
        self.y = y                 # Coordonnée y dans le système mondial (en mètres)
        self.yaw = yaw             # Cap / orientation (en radians)
        self.g_cost = g_cost       # Coût réel (coût cumulé depuis le départ)
        self.h_cost = h_cost       # Coût heuristique (estimation du coût restant jusqu'à l'objectif)
        self.f_cost = g_cost + h_cost # Coût total f = g + h
        self.parent_index = parent_index # Indice du nœud parent dans nodes_list pour la reconstruction du chemin
        self.direction = direction # Direction du mouvement (1: marche avant, -1: marche arrière)


class AStarPlanner(Node):
    """
    Nœud ROS 2 de planification de trajectoire basé sur Hybrid A*.
    Prend en charge le chargement de cartes statiques, la souscription dynamique à RTAB-Map,
    la vérification de collision par boîte englobante (footprint) et la visualisation OpenCV.
    """

    def __init__(self, yaml_file):
        super().__init__('astar_planner_node')
        self.yaml_file = yaml_file
        self.grid = None           # Grille de la carte (0: libre, 1: obstacle)

        self.resolution = 0.05     # Résolution de la carte (mètres/pixel)
        self.origin = [0.0, 0.0, 0.0] # Origine de la carte [x, y, yaw]
        self.height = 0            # Hauteur de la carte (lignes en pixels)
        self.width = 0             # Largeur de la carte (colonnes en pixels)

        # Dimensions physiques réelles du robot différentiel (longueur 0.32m, largeur 0.37m)
        self.robot_length = 0.32
        self.robot_width = 0.37

        # Paramètres cinématiques pour le Hybrid A*
        self.step_size = 0.12                    # Longueur d'un pas en avant (en mètres)
        self.steering_angles = [-0.4, -0.2, 0.0, 0.2, 0.4]  # Liste des angles de braquage candidats (en radians)

    def load_static_map(self):
        """
        Charge la configuration de la carte statique (.yaml) et l'image associée (.pgm/.png).
        """
        print(f"[INFO] Chargement de la configuration de la carte statique depuis : {self.yaml_file}")
        with open(self.yaml_file, "r") as file:
            config = yaml.safe_load(file)

        self.resolution = float(config["resolution"])
        self.origin = [float(v) for v in config["origin"]]

        # Récupère le chemin absolu de l'image de la carte et la lit en nuances de gris
        image_path = os.path.join(
            os.path.dirname(self.yaml_file), config["image"]
        )
        map_img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)

        if map_img is None:
            raise RuntimeError(f"Impossible de charger l'image de la carte depuis : {image_path}")

        self.height, self.width = map_img.shape
        self.grid = np.zeros(map_img.shape, dtype=np.uint8)
        # Seuil : les pixels < 100 sont considérés comme des obstacles (1), le reste comme libre (0)
        self.grid[map_img < 100] = 1
        self.grid[map_img >= 100] = 0
        print(f"[INFO] 🛡️ Carte statique chargée (sans distorsion, pas de dilatation agressive) !")

    def load_dynamic_rtab_map(self):
        """
        S'abonne dynamiquement au topic de grille de probabilité de RTAB-Map (/rtabmap/grid_prob_map).
        Utilise une politique QoS de type TRANSIENT_LOCAL pour s'assurer d'obtenir la dernière carte disponible.
        """
        print("[INFO] Souscription à la grille RTAB-Map avec la politique QoS TRANSIENT_LOCAL...")
        
        latest_map_msg = None

        def map_callback(msg):
            nonlocal latest_map_msg
            latest_map_msg = msg

        map_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )

        sub = self.create_subscription(OccupancyGrid, '/rtabmap/grid_prob_map', map_callback, map_qos)
        
        # Attente active du message de la carte avec un délai d'expiration (timeout) de 5 secondes
        start_time = self.get_clock().now().nanoseconds / 1e9
        while latest_map_msg is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            if (self.get_clock().now().nanoseconds / 1e9 - start_time) > 5.0:
                print("[ERROR] Délai d'attente de la carte dépassé ! Basculement sur la carte statique.")
                sub.destroy()
                self.load_static_map()
                return False

        sub.destroy() # Destruction de l'abonnement une fois la carte reçue

        # Mise à jour des métadonnées de la carte
        self.resolution = latest_map_msg.info.resolution
        self.origin = [
            latest_map_msg.info.origin.position.x,
            latest_map_msg.info.origin.position.y,
            0.0
        ]
        self.width = latest_map_msg.info.width
        self.height = latest_map_msg.info.height

        # Conversion des données de la grille 1D en tableau 2D et inversion verticale (adaptation au repère image)
        map_array = np.array(latest_map_msg.data, dtype=np.int8).reshape((self.height, self.width))
        map_array = np.flipud(map_array)

        self.grid = np.zeros((self.height, self.width), dtype=np.uint8)
        self.grid[map_array > 50] = 1   # Probabilité > 50 considérée comme un obstacle
        self.grid[map_array == -1] = 0  # Zones inconnues traitées comme libres
        self.grid[map_array == 0] = 0   # Zones libres

        print(f"[INFO] Grille RTAB-Map en direct chargée avec succès ! Taille : {self.width}x{self.height}")
        return True

    def world_to_pixel(self, world_x, world_y):
        """Convertit les coordonnées du monde (mètres) en coordonnées pixels de l'image."""
        pixel_x = int((world_x - self.origin[0]) / self.resolution)
        pixel_y = int(self.height - ((world_y - self.origin[1]) / self.resolution))
        return (pixel_x, pixel_y)

    def pixel_to_world(self, pixel_x, pixel_y):
        """Convertit les coordonnées pixels en coordonnées du monde (mètres), arrondies à 3 décimales."""
        world_x = self.origin[0] + pixel_x * self.resolution
        world_y = self.origin[1] + (self.height - pixel_y) * self.resolution
        return round(world_x, 3), round(world_y, 3)

    def check_footprint_safety(self, x, y, yaw):
        """
        Vérifie si la boîte englobante rectangulaire du robot en collisionne avec un obstacle
        pour une pose donnée. Calcule les 4 coins du robot dans le repère mondial en fonction
        de ses dimensions, puis les projette sur la grille de pixels.
        """
        car_half_l = self.robot_length / 2.0
        car_half_w = self.robot_width / 2.0
        
        # Les 4 coins dans le repère local du robot
        corners = [
            ( car_half_l,  car_half_w),
            ( car_half_l, -car_half_w),
            (-car_half_l,  car_half_w),
            (-car_half_l, -car_half_w)
        ]
        
        for cx, cy in corners:
            # Transformation par matrice de rotation et translation vers le repère mondial
            wx = x + cx * math.cos(yaw) - cy * math.sin(yaw)
            wy = y + cx * math.sin(yaw) + cy * math.cos(yaw)
            
            # Conversion en coordonnées pixels
            mx = int((wx - self.origin[0]) / self.resolution)
            my = int(self.height - ((wy - self.origin[1]) / self.resolution))
            
            # Vérification des limites de la carte : hors limites = non sécurisé
            if not (0 <= mx < self.width and 0 <= my < self.height):
                return False
            # Vérification des obstacles : valeur de grille à 1 = collision
            if self.grid[my, mx] == 1:
                return False
        return True

    def check_motion_safety(self, x0, y0, yaw0, x1, y1, yaw1):
        """
        Effectue une interpolation dense le long de l'arc de trajectoire pour la détection
        de collision, empêchant l'effet de "traversée de murs" (tunneling) dû à un pas trop grand.
        """
        dist = math.hypot(x1 - x0, y1 - y0)
        steps = max(int(dist / (self.resolution * 0.5)), 3) # Calcul dynamique du nombre de pas d'interpolation
        
        for i in range(steps + 1):
            t = i / float(steps)
            x = x0 * (1 - t) + x1 * t
            y = y0 * (1 - t) + y1 * t
            yaw = yaw0 * (1 - t) + yaw1 * t
            # Vérifie la sécurité de la boîte englobante sur chaque point interpolé
            if not self.check_footprint_safety(x, y, yaw):
                return False
        return True

    def find_nearest_free_cell(self, start_pixel, max_radius=30):
        """
        🚀 Recherche avancée d'un point de départ sécurisé :
        Exige non seulement que le pixel central soit libre, mais aussi que toute la boîte
        englobante du robot soit totalement exempte de collision à cet endroit !
        Si le point de départ est proche ou à l'intérieur d'un obstacle, utilise un parcours en
        largeur (BFS) pour trouver la cellule libre et valide la plus proche.
        """
        x, y = start_pixel
        wx, wy = self.pixel_to_world(x, y)
        if 0 <= x < self.width and 0 <= y < self.height:
            if self.check_footprint_safety(wx, wy, 0.0):
                return (x, y)

        queue = [(x, y)]
        visited = {(x, y)}
        while queue:
            cx, cy = queue.pop(0)
            for dx, dy in [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)]:
                nx, ny = cx + dx, cy + dy
                if 0 <= nx < self.width and 0 <= ny < self.height:
                    if (nx, ny) not in visited:
                        visited.add((nx, ny))
                        nwx, nwy = self.pixel_to_world(nx, ny)
                        
                        # Règle clé : la boîte englobante complète doit être sûre pour servir de départ
                        if self.check_footprint_safety(nwx, nwy, 0.0):
                            if math.hypot(nx - x, ny - y) <= max_radius:
                                return (nx, ny)
                        
                        queue.append((nx, ny))
        return (x, y)

    def select_goal(self):
        """
        Interface interactive par fenêtre OpenCV :
        - 1er clic du bouton gauche de la souris : sélection du point d'arrivée (Goal)
        - 2e clic du bouton gauche de la souris : sélection du point de référence d'orientation (Heading)
        Calcule et renvoie les coordonnées mondiales de l'objectif ainsi que l'angle de cap.
        """
        display_raw = np.where(self.grid == 1, 0, 255).astype(np.uint8)
        display_raw = cv2.cvtColor(display_raw, cv2.COLOR_GRAY2BGR)
        display = cv2.resize(display_raw, (self.width * 2, self.height * 2), interpolation=cv2.INTER_NEAREST)

        selected_points = []
        window_name = "Select GOAL (1st Click) & HEADING (2nd Click)"
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window_name, self.width * 2, self.height * 2)

        def mouse_callback(event, x, y, flags, param):
            if event == cv2.EVENT_LBUTTONDOWN:
                real_x, real_y = int(x / 2.0), int(y / 2.0)
                if len(selected_points) >= 2:
                    return
                if len(selected_points) == 0:
                    if 0 <= real_x < self.width and 0 <= real_y < self.height:
                        if self.grid[real_y, real_x] == 1:
                            print(f"[WARN] Le point sélectionné ({real_x}, {real_y}) se trouve dans un obstacle !")
                            return
                        selected_points.append((real_x, real_y))
                        print(f"[INFO] Objectif sélectionné au pixel : ({real_x}, {real_y})")
                        cv2.circle(display, (real_x * 2, real_y * 2), 10, (255, 0, 0), -1)
                        cv2.imshow(window_name, display)
                elif len(selected_points) == 1:
                    if 0 <= real_x < self.width and 0 <= real_y < self.height:
                        selected_points.append((real_x, real_y))
                        print(f"[INFO] Référence d'orientation sélectionnée au pixel : ({real_x}, {real_y})")
                        cv2.circle(display, (real_x * 2, real_y * 2), 8, (0, 255, 255), -1)
                        cv2.line(display, (selected_points[0][0]*2, selected_points[0][1]*2), (real_x * 2, real_y * 2), (0, 255, 0), 3)
                        cv2.imshow(window_name, display)

        print("[INFO] Veuillez cliquer sur la carte pour définir l'OBJECTIF (1er clic) et l'ORIENTATION (2e clic)... (Appuyez sur 'x' pour quitter)")
        cv2.imshow(window_name, display)
        cv2.setMouseCallback(window_name, mouse_callback)

        while len(selected_points) < 2:
            cv2.imshow(window_name, display)
            key = cv2.waitKey(50)
            if cv2.getWindowProperty(window_name, cv2.WND_PROP_VISIBLE) < 1:
                print("[INFO] Fenêtre fermée par l'utilisateur. Sortie...")
                cv2.destroyAllWindows()
                sys.exit(0)
            if key == 27: # Touche Échap (ESC) pour quitter
                cv2.destroyAllWindows()
                sys.exit(0)

        cv2.destroyAllWindows()

        if len(selected_points) == 2:
            p1_world = self.pixel_to_world(selected_points[0][0], selected_points[0][1])
            p2_world = self.pixel_to_world(selected_points[1][0], selected_points[1][1])
            # Calcul de l'angle de cap de l'objectif à partir des deux points
            goal_yaw = math.atan2(p2_world[1] - p1_world[1], p2_world[0] - p1_world[0])
            return selected_points[0], goal_yaw
        return None, None

    def hybrid_astar(self, start_pose, goal_pose):
        """
        Implémentation de l'algorithme principal Hybrid A* :
        Combine le modèle cinématique en espace continu (marche avant/arrière, angles de braquage multiples)
        et le hachage par grille discrétisée pour la gestion des doublons (closed_set).
        Utilise une file de priorité (Min-Heap) pour rechercher un chemin optimal et fluide.
        """
        print(f"[INFO] Démarrage de la planification Hybrid A* de {start_pose} vers {goal_pose}...")
        
        start_node = HybridNode(start_pose[0], start_pose[1], start_pose[2], 0.0, 0.0, None, 1)
        goal_node = HybridNode(goal_pose[0], goal_pose[1], goal_pose[2], 0.0, 0.0, None, 1)

        # Vérification de la sécurité du véhicule au départ
        if not self.check_footprint_safety(start_node.x, start_node.y, start_node.yaw):
            print("[ERROR] La position de départ est en collision avec un obstacle !")
            return None

        open_set = []
        counter = itertools.count() # Compteur pour éviter les erreurs de comparaison heapq si f_cost est identique
        nodes_list = [start_node]
        heapq.heappush(open_set, (start_node.f_cost, next(counter), 0))

        closed_set = set() # Ensemble fermé (closed set) des états discrétisés

        def get_discrete_index(node):
            """Discrétise l'état continu pour éviter l'explosion de l'espace d'états dans le closed_set."""
            gx = int((node.x - self.origin[0]) / (self.resolution * 2))
            gy = int((node.y - self.origin[1]) / (self.resolution * 2))
            g_yaw = int(node.yaw / (math.pi / 6)) # Tranche de cap de 30 degrés par compartiment
            return (gx, gy, g_yaw)

        while open_set:
            _, _, current_idx = heapq.heappop(open_set)
            current = nodes_list[current_idx]

            disc_idx = get_discrete_index(current)
            if disc_idx in closed_set:
                continue
            closed_set.add(disc_idx)

            # Condition d'atteinte de l'objectif : erreur de position < 0.35m et différence de cap < 25 degrés
            dist_to_goal = math.hypot(current.x - goal_node.x, current.y - goal_node.y)
            yaw_diff = abs(math.atan2(math.sin(goal_node.yaw - current.yaw), math.cos(goal_node.yaw - current.yaw)))
            
            if dist_to_goal < 0.35 and yaw_diff < math.radians(25):
                print(f"[INFO] Chemin Hybrid A* trouvé ! Nœuds totaux explorés : {len(nodes_list)}")
                path = []
                curr = current
                # Remontée des parents pour construire le chemin complet
                while curr is not None:
                    path.append((curr.x, curr.y))
                    if curr.parent_index is not None:
                        curr = nodes_list[curr.parent_index]
                    else:
                        break
                path.reverse()
                return path

            # Expansion des nœuds enfants : parcours de tous les angles de braquage et directions
            for steer in self.steering_angles:
                for direction in [1, -1]:
                    step = self.step_size * direction
                    new_yaw = current.yaw + steer * direction
                    new_x = current.x + step * math.cos(new_yaw)
                    new_y = current.y + step * math.sin(new_yaw)

                    # Vérification de la sécurité du mouvement et de l'interpolation
                    if not self.check_motion_safety(current.x, current.y, current.yaw, new_x, new_y, new_yaw):
                        continue

                    new_disc = get_discrete_index(HybridNode(new_x, new_y, new_yaw, 0, 0, 0))
                    if new_disc in closed_set:
                        continue

                    # Calcul du coût : coût de distance de base + pénalité de braquage. Pénalité supplémentaire en marche arrière.
                    g_cost = current.g_cost + abs(step) + abs(steer) * 0.2
                    if direction == -1:
                        g_cost += 1.5
                    
                    h_cost = math.hypot(new_x - goal_node.x, new_y - goal_node.y)
                    
                    next_node = HybridNode(new_x, new_y, new_yaw, g_cost, h_cost, current_idx, direction)
                    nodes_list.append(next_node)
                    next_idx = len(nodes_list) - 1

                    heapq.heappush(open_set, (next_node.f_cost, next(counter), next_idx))

        print("[WARN] Le Hybrid A* n'a pas pu trouver de chemin valide vers l'objectif !")
        return None

    def draw_path_blocking(self, path, rtab_pixel, window_title="Planned Path"):
        """Affiche le chemin global planifié dans une fenêtre graphique bloquante."""
        display_raw = np.where(self.grid == 1, 0, 255).astype(np.uint8)
        display_raw = cv2.cvtColor(display_raw, cv2.COLOR_GRAY2BGR)
        display = cv2.resize(display_raw, (self.width * 2, self.height * 2), interpolation=cv2.INTER_NEAREST)

        if len(path) >= 2:
            pixel_pts = [self.world_to_pixel(pt[0], pt[1]) for pt in path]
            pts = np.array([[pt[0] * 2, pt[1] * 2] for pt in pixel_pts], np.int32)
            cv2.polylines(display, [pts], isClosed=False, color=(0, 0, 255), thickness=3)

        start_px = self.world_to_pixel(path[0][0], path[0][1])
        goal_px = self.world_to_pixel(path[-1][0], path[-1][1])
        cv2.circle(display, (start_px[0] * 2, start_px[1] * 2), 8, (0, 255, 0), -1)   # Départ en vert
        cv2.circle(display, (goal_px[0] * 2, goal_px[1] * 2), 8, (255, 0, 0), -1)    # Arrivée en bleu
        cv2.circle(display, (rtab_pixel[0] * 2, rtab_pixel[1] * 2), 8, (0, 255, 255), -1) # Position actuelle du robot en jaune

        cv2.namedWindow(window_title, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window_title, self.width * 2, self.height * 2)
        
        print(f"[INFO] Affichage de la fenêtre GUI '{window_title}'. Cliquez sur 'x' pour quitter et fermer...")
        
        while True:
            cv2.imshow(window_title, display)
            key = cv2.waitKey(50)
            if cv2.getWindowProperty(window_title, cv2.WND_PROP_VISIBLE) < 1:
                break
            if key != -1:
                break
                
        cv2.destroyAllWindows()

    def save_replan_image(self, path, rtab_pixel, filename="replan_result.png"):
        """Enregistre le résultat du nouveau calcul de trajectoire sous forme d'image."""
        display_raw = np.where(self.grid == 1, 0, 255).astype(np.uint8)
        display_raw = cv2.cvtColor(display_raw, cv2.COLOR_GRAY2BGR)
        display = cv2.resize(display_raw, (self.width * 2, self.height * 2), interpolation=cv2.INTER_NEAREST)

        if len(path) >= 2:
            pixel_pts = [self.world_to_pixel(pt[0], pt[1]) for pt in path]
            pts = np.array([[pt[0] * 2, pt[1] * 2] for pt in pixel_pts], np.int32)
            cv2.polylines(display, [pts], isClosed=False, color=(0, 0, 255), thickness=3)

        start_px = self.world_to_pixel(path[0][0], path[0][1])
        goal_px = self.world_to_pixel(path[-1][0], path[-1][1])
        cv2.circle(display, (start_px[0] * 2, start_px[1] * 2), 8, (0, 255, 0), -1)
        cv2.circle(display, (goal_px[0] * 2, goal_px[1] * 2), 8, (255, 0, 0), -1)
        cv2.circle(display, (rtab_pixel[0] * 2, rtab_pixel[1] * 2), 8, (0, 255, 255), -1)  

        save_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), filename)
        cv2.imwrite(save_path, display)
        print(f"[INFO] Visualisation du replan enregistrée vers {save_path}")

    def plan_path_to_goal_headless(self, goal_world_x, goal_world_y, goal_yaw=0.0):
        """
        Pipeline de replanification dynamique en mode headless (sans interface graphique) :
        1. Charge la carte dynamique en direct depuis RTAB-Map
        2. Récupère la pose actuelle du robot
        3. Exécute la planification Hybrid A*
        4. Sauvegarde le chemin dans world_path.npy et génère l'image de visualisation
        """
        print("[INFO] Déclenchement du replan dynamique piloté par la carte RTAB-Map (Hybrid A*)...")
        
        if not self.load_dynamic_rtab_map():
            print("[ERROR] Échec du chargement de la carte dynamique depuis RTAB-Map !")
            return False

        robot_pose = get_robot_pose(timeout_sec=3.0)
        if robot_pose is None:
            print("[ERROR] Échec de l'obtention de la pose du robot pour la replanification !")
            return False

        robot_x, robot_y = robot_pose
        px, py = self.world_to_pixel(robot_x, robot_y)
        start_pixel = self.find_nearest_free_cell((px, py))
        start_world = self.pixel_to_world(start_pixel[0], start_pixel[1])
        
        start_pose = (start_world[0], start_world[1], 0.0)
        goal_pose = (goal_world_x, goal_world_y, goal_yaw)

        world_path = self.hybrid_astar(start_pose, goal_pose)
        if world_path is None:
            print("[ERROR] Échec de la replanification Hybrid A* !")
            return False

        # Sauvegarde des données du chemin pour lecture par le contrôleur de bas niveau
        save_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "world_path.npy")
        np.save(save_path, {"path": world_path, "goal_yaw": goal_yaw}, allow_pickle=True)
        print(f"[INFO] 🗺️ Chemin lissé anti-collision basé sur Hybrid A* généré et sauvegardé !")
        
        self.save_replan_image(world_path, (px, py), filename="replan_result.png")
        return True


if __name__ == "__main__":
    yaml_file = "/home/yz0000/wifibot_ws/src/wifibot_navigation/maps/indoor/map.yaml"
    
    if not rclpy.ok():
        rclpy.init()

    planner = AStarPlanner(yaml_file)

    # Mode 1 : Si des arguments de ligne de commande sont fournis (x, y, [yaw]), exécute en arrière-plan (mode headless)
    if len(sys.argv) >= 3:
        goal_wx = float(sys.argv[1])
        goal_wy = float(sys.argv[2])
        goal_yw = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0
        
        success = planner.plan_path_to_goal_headless(goal_wx, goal_wy, goal_yw)
        
        if rclpy.ok():
            rclpy.shutdown()
        sys.exit(0 if success else 1)
    
    # Mode 2 : Aucun argument fourni, exécute le mode interactif avec interface graphique (carte statique + sélection manuelle)
    else:
        planner.load_static_map()
        
        robot_pose = get_robot_pose(timeout_sec=10.0)
        if robot_pose is None:
            print("[ERROR] Échec de l'obtention de la pose initiale du robot !")
            rclpy.shutdown()
            sys.exit(1)

        px, py = planner.world_to_pixel(robot_pose[0], robot_pose[1])
        start_pixel = planner.find_nearest_free_cell((px, py))
        start_world = planner.pixel_to_world(start_pixel[0], start_pixel[1])
        
        # Ouvre une fenêtre pour sélectionner manuellement l'objectif et l'orientation
        goal_pixel, goal_yaw = planner.select_goal()
        if goal_pixel is None:
            print("[ERROR] Aucun objectif sélectionné ou sélection annulée !")
            rclpy.shutdown()
            sys.exit(1)

        goal_world_x, goal_world_y = planner.pixel_to_world(goal_pixel[0], goal_pixel[1])

        start_pose = (start_world[0], start_world[1], 0.0)
        goal_pose = (goal_world_x, goal_world_y, goal_yaw)

        world_path = self.hybrid_astar(start_pose, goal_pose) if 'self' in locals() else planner.hybrid_astar(start_pose, goal_pose)
        if world_path is None:
            rclpy.shutdown()
            sys.exit(1)

        # Enregistre le chemin de planification initial
        save_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "world_path.npy")
        np.save(save_path, {"path": world_path, "goal_yaw": goal_yaw}, allow_pickle=True)
        print(f"[INFO] Chemin initial Hybrid A* sauvegardé dans {save_path} avec {len(world_path)} waypoints.")
        
        # Affiche la fenêtre de visualisation bloquante du chemin initial
        planner.draw_path_blocking(world_path, (px, py), window_title="Initial Hybrid A* Path")
        rclpy.shutdown()
        sys.exit(0)
