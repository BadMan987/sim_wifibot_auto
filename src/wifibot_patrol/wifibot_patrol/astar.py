#!/usr/bin/env python3

import heapq
import itertools
import math
import os
import sys
import cv2
import numpy as np
import yaml

# Dépendances de base ROS2
import rclpy
from robot_pose import get_robot_pose


class AStarPlanner:

    def __init__(self, yaml_file):
        self.yaml_file = yaml_file
        self.grid = None
        self.distance_field = None

        self.resolution = 0.05
        self.origin = [0.0, 0.0, 0.0]
        self.height = 0
        self.width = 0

        self.load_map()

    def load_map(self):
        """Recharge la carte statique pure à chaque appel, effaçant complètement tous les résidus d'obstacles virtuels historiques"""
        print(f"[INFO] Loading map configuration from: {self.yaml_file}")
        with open(self.yaml_file, "r") as file:
            config = yaml.safe_load(file)

        self.resolution = float(config["resolution"])
        self.origin = [float(v) for v in config["origin"]]

        image_path = os.path.join(
            os.path.dirname(self.yaml_file), config["image"]
        )
        map_img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)

        if map_img is None:
            raise RuntimeError(f"Cannot load map image from: {image_path}")

        self.height, self.width = map_img.shape
        print(f"[INFO] Map loaded successfully! Size: {self.width}x{self.height}, Resolution: {self.resolution} m/pixel")

        self.grid = np.zeros(map_img.shape, dtype=np.uint8)
        self.grid[map_img < 100] = 1
        self.grid[map_img >= 100] = 0

        # Dilatation de la carte (Inflation)
        robot_radius = 0.24
        inflate_pixels = int(robot_radius / self.resolution)
        print(f"[INFO] Applying map inflation with robot radius {robot_radius}m ({inflate_pixels} pixels)...")
        kernel = np.ones((inflate_pixels * 2 + 1, inflate_pixels * 2 + 1), np.uint8)
        self.grid = cv2.dilate(self.grid, kernel, iterations=1)

        # Champ de transformation de distance
        self.distance_field = cv2.distanceTransform(1 - self.grid, cv2.DIST_L2, 5)

    def pixel_to_world(self, pixel_x, pixel_y):
        world_x = self.origin[0] + pixel_x * self.resolution
        world_y = self.origin[1] + (self.height - pixel_y) * self.resolution
        return round(world_x, 3), round(world_y, 3)

    def world_to_pixel(self, world_x, world_y):
        pixel_x = int((world_x - self.origin[0]) / self.resolution)
        pixel_y = int(self.height - ((world_y - self.origin[1]) / self.resolution))
        return (pixel_x, pixel_y)

    def heuristic(self, a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def neighbors(self, node):
        x, y = node
        directions = [
            (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
            (1, 1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (-1, -1, 1.414),
        ]
        result = []
        for dx, dy, cost in directions:
            nx, ny = x + dx, y + dy
            if 0 <= nx < self.width and 0 <= ny < self.height:
                if self.grid[ny, nx] == 0:
                    if dx != 0 and dy != 0:
                        if self.grid[y, nx] == 1 and self.grid[ny, x] == 1:
                            continue
                    result.append(((nx, ny), (dx, dy), cost))
        return result

    def line_of_sight(self, p1, p2):
        x0, y0 = p1
        x1, y1 = p2
        dx, dy = abs(x1 - x0), abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        x, y = x0, y0
        while True:
            if not (0 <= x < self.width and 0 <= y < self.height):
                return False
            if self.grid[y, x] == 1:
                return False
            if x == x1 and y == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy
        return True

    def find_nearest_free_cell(self, start_pixel, max_radius=30):
        x, y = start_pixel
        if 0 <= x < self.width and 0 <= y < self.height:
            if self.grid[y, x] == 0:
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
                        if self.grid[ny, nx] == 0:
                            if math.hypot(nx - x, ny - y) <= max_radius:
                                return (nx, ny)
                        queue.append((nx, ny))
        return None

    def select_goal(self):
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
                            print(f"[WARN] Selected point ({real_x}, {real_y}) is inside an obstacle!")
                            return
                        selected_points.append((real_x, real_y))
                        print(f"[INFO] Goal selected at pixel: ({real_x}, {real_y}) -> World: {self.pixel_to_world(real_x, real_y)}")
                        cv2.circle(display, (real_x * 2, real_y * 2), 10, (255, 0, 0), -1)
                        cv2.imshow(window_name, display)
                elif len(selected_points) == 1:
                    if 0 <= real_x < self.width and 0 <= real_y < self.height:
                        selected_points.append((real_x, real_y))
                        print(f"[INFO] Heading reference selected at pixel: ({real_x}, {real_y})")
                        cv2.circle(display, (real_x * 2, real_y * 2), 8, (0, 255, 255), -1)
                        cv2.line(display, (selected_points[0][0]*2, selected_points[0][1]*2), (real_x * 2, real_y * 2), (0, 255, 0), 3)
                        cv2.imshow(window_name, display)

        print("[INFO] Please click on the map to set GOAL (1st click) and HEADING (2nd click)...")
        cv2.imshow(window_name, display)
        cv2.setMouseCallback(window_name, mouse_callback)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

        if len(selected_points) == 2:
            p1_world = self.pixel_to_world(selected_points[0][0], selected_points[0][1])
            p2_world = self.pixel_to_world(selected_points[1][0], selected_points[1][1])
            goal_yaw = math.atan2(p2_world[1] - p1_world[1], p2_world[0] - p1_world[0])
            return selected_points[0], goal_yaw
        elif len(selected_points) == 1:
            return selected_points[0], 0.0
        return None, None

    def astar(self, start, goal):
        print(f"[INFO] Starting fast A* planning from {start} to {goal}...")
        if self.grid[start[1], start[0]] == 1 or self.grid[goal[1], goal[0]] == 1:
            print("[ERROR] Start or Goal position is blocked by obstacles!")
            return None

        counter = itertools.count()
        open_set = []
        heapq.heappush(open_set, (0.0, next(counter), start, None))
        
        came_from = {start: start}
        g_score = {start: 0.0}
        node_dir = {start: None}

        while open_set:
            _, _, current, last_dir = heapq.heappop(open_set)

            if current == goal:
                path = []
                while current != came_from[current]:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                path.reverse()
                print(f"[INFO] A* path found with {len(path)} waypoints.")
                return path

            parent = came_from[current]

            for neighbor, move_dir, step_cost in self.neighbors(current):
                if neighbor not in g_score:
                    g_score[neighbor] = float('inf')
                    came_from[neighbor] = None
                    node_dir[neighbor] = None

                def compute_turn_penalty(d_from, d_to):
                    if d_from is None:
                        return 0.0
                    dot_product = d_from[0] * d_to[0] + d_from[1] * d_to[1]
                    norm_a = math.hypot(d_from[0], d_from[1])
                    norm_b = math.hypot(d_to[0], d_to[1])
                    if norm_a == 0 or norm_b == 0:
                        return 0.0
                    cos_theta = max(-1.0, min(1.0, dot_product / (norm_a * norm_b)))
                    return math.acos(cos_theta) * 1.5

                if self.line_of_sight(parent, neighbor):
                    direct_dist = math.hypot(neighbor[0] - parent[0], neighbor[1] - parent[1])
                    parent_dir = node_dir.get(parent, None)
                    direct_move_dir = (neighbor[0] - parent[0], neighbor[1] - parent[1])
                    
                    turn_pen = compute_turn_penalty(parent_dir, direct_move_dir)
                    tentative_g = g_score[parent] + direct_dist + turn_pen

                    if tentative_g < g_score[neighbor]:
                        came_from[neighbor] = parent
                        g_score[neighbor] = tentative_g
                        node_dir[neighbor] = direct_move_dir
                        f_score = tentative_g + self.heuristic(neighbor, goal)
                        heapq.heappush(open_set, (f_score, next(counter), neighbor, direct_move_dir))
                else:
                    turn_pen = compute_turn_penalty(last_dir, move_dir)
                    tentative_g = g_score[current] + step_cost + turn_pen

                    if tentative_g < g_score[neighbor]:
                        came_from[neighbor] = current
                        g_score[neighbor] = tentative_g
                        node_dir[neighbor] = move_dir
                        f_score = tentative_g + self.heuristic(neighbor, goal)
                        heapq.heappush(open_set, (f_score, next(counter), neighbor, move_dir))
        
        print("[WARN] A* failed to find a valid path to the goal!")
        return None

    def smooth_path(self, path):
        if len(path) < 3:
            return path

        def interpolate_segment(p1, p2):
            x1, y1 = p1
            x2, y2 = p2
            seg_pts = []
            steps = max(abs(x2 - x1), abs(y2 - y1))
            if steps == 0:
                return [p1]
            for step in range(steps + 1):
                t = step / steps
                x = int(round(x1 * (1 - t) + x2 * t))
                y = int(round(y1 * (1 - t) + y2 * t))
                if 0 <= x < self.width and 0 <= y < self.height:
                    if self.grid[y, x] == 0:
                        seg_pts.append((x, y))
            return seg_pts

        smoothed_pixel_path = []
        for i in range(len(path) - 1):
            for pt in interpolate_segment(path[i], path[i+1]):
                if not smoothed_pixel_path or pt != smoothed_pixel_path[-1]:
                    smoothed_pixel_path.append(pt)
                print(f"[INFO] Path smoothed: {len(path)} -> {len(smoothed_pixel_path)} waypoints.")
        return smoothed_pixel_path if len(smoothed_pixel_path) >= 2 else path

    def draw_path_blocking(self, path, rtab_pixel, window_title="Planned Path"):
        display_raw = np.where(self.grid == 1, 0, 255).astype(np.uint8)
        display_raw = cv2.cvtColor(display_raw, cv2.COLOR_GRAY2BGR)
        display = cv2.resize(display_raw, (self.width * 2, self.height * 2), interpolation=cv2.INTER_NEAREST)

        if len(path) >= 2:
            pts = np.array([[pt[0] * 2, pt[1] * 2] for pt in path], np.int32)
            cv2.polylines(display, [pts], isClosed=False, color=(0, 0, 255), thickness=3)

        cv2.circle(display, (path[0][0] * 2, path[0][1] * 2), 8, (0, 255, 0), -1)
        cv2.circle(display, (path[-1][0] * 2, path[-1][1] * 2), 8, (255, 0, 0), -1)
        cv2.circle(display, (rtab_pixel[0] * 2, rtab_pixel[1] * 2), 8, (0, 255, 255), -1)  

        cv2.namedWindow(window_title, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window_title, self.width * 2, self.height * 2)
        cv2.imshow(window_title, display)
        print(f"[INFO] Displaying GUI window '{window_title}'. Press any key in the window to continue...")
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    def save_replan_image(self, path, rtab_pixel, filename="replan_result.png"):
        display_raw = np.where(self.grid == 1, 0, 255).astype(np.uint8)
        display_raw = cv2.cvtColor(display_raw, cv2.COLOR_GRAY2BGR)
        display = cv2.resize(display_raw, (self.width * 2, self.height * 2), interpolation=cv2.INTER_NEAREST)

        if len(path) >= 2:
            pts = np.array([[pt[0] * 2, pt[1] * 2] for pt in path], np.int32)
            cv2.polylines(display, [pts], isClosed=False, color=(0, 0, 255), thickness=3)

        cv2.circle(display, (path[0][0] * 2, path[0][1] * 2), 8, (0, 255, 0), -1)
        cv2.circle(display, (path[-1][0] * 2, path[-1][1] * 2), 8, (255, 0, 0), -1)
        cv2.circle(display, (rtab_pixel[0] * 2, rtab_pixel[1] * 2), 8, (0, 255, 255), -1)  

        save_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), filename)
        cv2.imwrite(save_path, display)
        print(f"[INFO] Replan route visualization saved to {save_path}")

    def plan_path_to_goal_headless(self, goal_world_x, goal_world_y, goal_yaw=0.0):
        """Replanification dynamique headless (sans interface) : la direction pointe toujours vers le point cible, évitant complètement la dérive d'orientation des obstacles et l'invalidité de zone"""
        print("[INFO] Triggering Headless Dynamic Re-planning with Stable Goal-Vector...")
        
        # 1. Recharge de force la carte statique pure avant chaque replanification
        self.load_map()

        robot_pose = get_robot_pose(timeout_sec=3.0)
        if robot_pose is None:
            print("[ERROR] Failed to get robot pose for re-planning!")
            return False

        robot_x, robot_y = robot_pose
        px, py = self.world_to_pixel(robot_x, robot_y)
        print(f"[INFO] Current robot pose -> World: ({robot_x}, {robot_y}), Pixel: ({px}, {py})")

        # 2. [CORRECTION PRINCIPALE] La direction d'avancement pointe toujours vers « la direction allant de la position actuelle du robot au point cible final (Goal) »
        # Ne lit absolument pas l'ancien chemin courbé précédent, garantissant que l'obstacle est toujours précisément verrouillé juste en face, et ne deviendra jamais une zone navigable lors de la deuxième planification !
        dir_x, dir_y = 1.0, 0.0
        vx = goal_world_x - robot_x
        vy = goal_world_y - robot_y
        v_len = math.hypot(vx, vy)
        if v_len > 1e-3:
            dir_x, dir_y = vx / v_len, vy / v_len

        # 3. Injection précise d'un obstacle en forme de fine cloison devant l'avant du véhicule, avec ajout d'une [zone de protection de départ] anti-blocage
        print("[INFO] Injecting obstacle block with start-zone protection...")
        start_px, start_py = px, py

        for step_m in np.arange(0.2, 0.35, 0.05):
            obs_wx = robot_x + dir_x * step_m
            obs_wy = robot_y + dir_y * step_m
            obs_px, obs_py = self.world_to_pixel(obs_wx, obs_wy)
            
            for dx in range(-2, 3):
                for dy in range(-2, 3):
                    nx, ny = obs_px + dx, obs_py + dy
                    if 0 <= nx < self.width and 0 <= ny < self.height:
                        # [PROTECTION PRINCIPALE] Ne jamais noircir les 3 pixels autour de la position actuelle du petit robot !
                        if math.hypot(nx - start_px, ny - start_py) > 3:
                            self.grid[ny, nx] = 1

        start_pixel = self.find_nearest_free_cell((px, py))
        if start_pixel is None:
            print("[ERROR] Could not find nearest free cell for start pose!")
            return False

        gx, gy = self.world_to_pixel(goal_world_x, goal_world_y)
        goal_pixel = self.find_nearest_free_cell((gx, gy))
        if goal_pixel is None:
            print("[ERROR] Could not find nearest free cell for goal pose!")
            return False
        print(f"[INFO] Target goal pixel -> ({gx}, {gy}), World goal -> ({goal_world_x}, {goal_world_y})")

        # 4. Recherche de chemin A* efficace et standard
        pixel_path = self.astar(start_pixel, goal_pixel)
        if pixel_path is None:
            print("[ERROR] Re-planning A* failed!")
            return False

        pixel_path = self.smooth_path(pixel_path)
        world_path = [self.pixel_to_world(px_val, py_val) for px_val, py_val in pixel_path]
        if len(world_path) > 10:
            world_path = world_path[::2]

        save_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "world_path.npy")
        np.save(save_path, {"path": world_path, "goal_yaw": goal_yaw}, allow_pickle=True)
        print(f"[INFO] New re-planned path saved to {save_path} with {len(world_path)} waypoints.")
        
        print("[INFO] J'ai planifié l'itinéraire d'évitement d'obstacles.")
        self.save_replan_image(pixel_path, (px, py), filename="replan_result.png")
        return True


if __name__ == "__main__":
    yaml_file = "/home/yz0000/wifibot_ws/src/wifibot_navigation/maps/indoor/map.yaml"
    planner = AStarPlanner(yaml_file)

    if len(sys.argv) >= 3:
        goal_wx = float(sys.argv[1])
        goal_wy = float(sys.argv[2])
        goal_yw = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0
        
        if not rclpy.ok():
            rclpy.init()
            
        success = planner.plan_path_to_goal_headless(goal_wx, goal_wy, goal_yw)
        
        if rclpy.ok():
            rclpy.shutdown()
            
        sys.exit(0 if success else 1)
    else:
        rclpy.init()
        robot_pose = get_robot_pose(timeout_sec=10.0)
        if robot_pose is None:
            print("[ERROR] Failed to obtain initial robot pose!")
            rclpy.shutdown()
            sys.exit(1)

        px, py = planner.world_to_pixel(robot_pose[0], robot_pose[1])
        start_pixel = planner.find_nearest_free_cell((px, py))
        goal_pixel, goal_yaw = planner.select_goal()
        if goal_pixel is None:
            print("[ERROR] No goal selected or selection canceled!")
            rclpy.shutdown()
            sys.exit(1)

        pixel_path = planner.astar(start_pixel, goal_pixel)
        if pixel_path is None:
            rclpy.shutdown()
            sys.exit(1)

        pixel_path = planner.smooth_path(pixel_path)
        world_path = [planner.pixel_to_world(px_val, py_val) for px_val, py_val in pixel_path]
        if len(world_path) > 10:
            world_path = world_path[::2]

        save_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "world_path.npy")
        np.save(save_path, {"path": world_path, "goal_yaw": goal_yaw}, allow_pickle=True)
        print(f"[INFO] Initial path saved to {save_path} with {len(world_path)} waypoints.")
        planner.draw_path_blocking(pixel_path, (px, py), window_title="Initial Planned Path")
        rclpy.shutdown()
