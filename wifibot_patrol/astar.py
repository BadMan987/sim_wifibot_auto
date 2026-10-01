#!/usr/bin/env python3

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
    def __init__(self, x, y, yaw, g_cost, h_cost, parent_index, direction=1):
        self.x = x             # 世界坐标系下 x (米)
        self.y = y             # 世界坐标系下 y (米)
        self.yaw = yaw         # 航向角 (弧度)
        self.g_cost = g_cost   # 实际代价
        self.h_cost = h_cost   # 启发式代价
        self.f_cost = g_cost + h_cost
        self.parent_index = parent_index
        self.direction = direction  # 1: 前进, -1: 后退


class AStarPlanner(Node):

    def __init__(self, yaml_file):
        super().__init__('astar_planner_node')
        self.yaml_file = yaml_file
        self.grid = None

        self.resolution = 0.05
        self.origin = [0.0, 0.0, 0.0]
        self.height = 0
        self.width = 0

        # 差速小车真实物理尺寸 (长 0.32m, 宽 0.37m)
        self.robot_length = 0.32
        self.robot_width = 0.37

        # Hybrid A* 运动学参数
        self.step_size = 0.12                  # 单次前进步长 (米)
        self.steering_angles = [-0.4, -0.2, 0.0, 0.2, 0.4]  # 转向弧度候选

    def load_static_map(self):
        print(f"[INFO] Loading static map configuration from: {self.yaml_file}")
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
        self.grid = np.zeros(map_img.shape, dtype=np.uint8)
        self.grid[map_img < 100] = 1
        self.grid[map_img >= 100] = 0
        print(f"[INFO] 🛡️ 静态地图已加载（保持零畸变，不进行硬膨胀）！")

    def load_dynamic_rtab_map(self):
        print("[INFO] Subscribing to RTAB-Map grid with TRANSIENT_LOCAL QoS...")
        
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
        
        start_time = self.get_clock().now().nanoseconds / 1e9
        while latest_map_msg is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            if (self.get_clock().now().nanoseconds / 1e9 - start_time) > 5.0:
                print("[ERROR] Timeout waiting for map! Falling back to static map.")
                sub.destroy()
                self.load_static_map()
                return False

        sub.destroy()

        self.resolution = latest_map_msg.info.resolution
        self.origin = [
            latest_map_msg.info.origin.position.x,
            latest_map_msg.info.origin.position.y,
            0.0
        ]
        self.width = latest_map_msg.info.width
        self.height = latest_map_msg.info.height

        map_array = np.array(latest_map_msg.data, dtype=np.int8).reshape((self.height, self.width))
        map_array = np.flipud(map_array)

        self.grid = np.zeros((self.height, self.width), dtype=np.uint8)
        self.grid[map_array > 50] = 1    
        self.grid[map_array == -1] = 0  
        self.grid[map_array == 0] = 0   

        print(f"[INFO] Successfully loaded live RTAB-Map grid! Size: {self.width}x{self.height}")
        return True

    def world_to_pixel(self, world_x, world_y):
        pixel_x = int((world_x - self.origin[0]) / self.resolution)
        pixel_y = int(self.height - ((world_y - self.origin[1]) / self.resolution))
        return (pixel_x, pixel_y)

    def pixel_to_world(self, pixel_x, pixel_y):
        world_x = self.origin[0] + pixel_x * self.resolution
        world_y = self.origin[1] + (self.height - pixel_y) * self.resolution
        return round(world_x, 3), round(world_y, 3)

    def check_footprint_safety(self, x, y, yaw):
        """检查单个位姿下车体矩形包围盒是否碰撞"""
        car_half_l = self.robot_length / 2.0
        car_half_w = self.robot_width / 2.0
        
        corners = [
            ( car_half_l,  car_half_w),
            ( car_half_l, -car_half_w),
            (-car_half_l,  car_half_w),
            (-car_half_l, -car_half_w)
        ]
        
        for cx, cy in corners:
            wx = x + cx * math.cos(yaw) - cy * math.sin(yaw)
            wy = y + cx * math.sin(yaw) + cy * math.cos(yaw)
            
            mx = int((wx - self.origin[0]) / self.resolution)
            my = int(self.height - ((wy - self.origin[1]) / self.resolution))
            
            if not (0 <= mx < self.width and 0 <= my < self.height):
                return False
            if self.grid[my, mx] == 1:
                return False
        return True

    def check_motion_safety(self, x0, y0, yaw0, x1, y1, yaw1):
        """沿运动弧线进行密集插值碰撞检测，杜绝穿墙"""
        dist = math.hypot(x1 - x0, y1 - y0)
        steps = max(int(dist / (self.resolution * 0.5)), 3)
        
        for i in range(steps + 1):
            t = i / float(steps)
            x = x0 * (1 - t) + x1 * t
            y = y0 * (1 - t) + y1 * t
            yaw = yaw0 * (1 - t) + yaw1 * t
            if not self.check_footprint_safety(x, y, yaw):
                return False
        return True

    def find_nearest_free_cell(self, start_pixel, max_radius=30):
        """
        🚀 升级版起点安全搜寻：不仅要求像素中心空白，
        更要求以该像素为中心时，整个小车矩形包围盒完全不撞墙！
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
                        
                        # 核心：整个车体包围盒安全才允许作为起点
                        if self.check_footprint_safety(nwx, nwy, 0.0):
                            if math.hypot(nx - x, ny - y) <= max_radius:
                                return (nx, ny)
                        
                        queue.append((nx, ny))
        return (x, y)

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
                        print(f"[INFO] Goal selected at pixel: ({real_x}, {real_y})")
                        cv2.circle(display, (real_x * 2, real_y * 2), 10, (255, 0, 0), -1)
                        cv2.imshow(window_name, display)
                elif len(selected_points) == 1:
                    if 0 <= real_x < self.width and 0 <= real_y < self.height:
                        selected_points.append((real_x, real_y))
                        print(f"[INFO] Heading reference selected at pixel: ({real_x}, {real_y})")
                        cv2.circle(display, (real_x * 2, real_y * 2), 8, (0, 255, 255), -1)
                        cv2.line(display, (selected_points[0][0]*2, selected_points[0][1]*2), (real_x * 2, real_y * 2), (0, 255, 0), 3)
                        cv2.imshow(window_name, display)

        print("[INFO] Please click on the map to set GOAL (1st click) and HEADING (2nd click)... (Click 'x' to exit)")
        cv2.imshow(window_name, display)
        cv2.setMouseCallback(window_name, mouse_callback)

        while len(selected_points) < 2:
            cv2.imshow(window_name, display)
            key = cv2.waitKey(50)
            if cv2.getWindowProperty(window_name, cv2.WND_PROP_VISIBLE) < 1:
                print("[INFO] Window closed by user. Exiting...")
                cv2.destroyAllWindows()
                sys.exit(0)
            if key == 27:
                cv2.destroyAllWindows()
                sys.exit(0)

        cv2.destroyAllWindows()

        if len(selected_points) == 2:
            p1_world = self.pixel_to_world(selected_points[0][0], selected_points[0][1])
            p2_world = self.pixel_to_world(selected_points[1][0], selected_points[1][1])
            goal_yaw = math.atan2(p2_world[1] - p1_world[1], p2_world[0] - p1_world[0])
            return selected_points[0], goal_yaw
        return None, None

    def hybrid_astar(self, start_pose, goal_pose):
        print(f"[INFO] Starting Hybrid A* planning from {start_pose} to {goal_pose}...")
        
        start_node = HybridNode(start_pose[0], start_pose[1], start_pose[2], 0.0, 0.0, None, 1)
        goal_node = HybridNode(goal_pose[0], goal_pose[1], goal_pose[2], 0.0, 0.0, None, 1)

        if not self.check_footprint_safety(start_node.x, start_node.y, start_node.yaw):
            print("[ERROR] Start position is in collision with obstacle!")
            return None

        open_set = []
        counter = itertools.count()
        nodes_list = [start_node]
        heapq.heappush(open_set, (start_node.f_cost, next(counter), 0))

        closed_set = set()

        def get_discrete_index(node):
            gx = int((node.x - self.origin[0]) / (self.resolution * 2))
            gy = int((node.y - self.origin[1]) / (self.resolution * 2))
            g_yaw = int(node.yaw / (math.pi / 6))
            return (gx, gy, g_yaw)

        while open_set:
            _, _, current_idx = heapq.heappop(open_set)
            current = nodes_list[current_idx]

            disc_idx = get_discrete_index(current)
            if disc_idx in closed_set:
                continue
            closed_set.add(disc_idx)

            dist_to_goal = math.hypot(current.x - goal_node.x, current.y - goal_node.y)
            yaw_diff = abs(math.atan2(math.sin(goal_node.yaw - current.yaw), math.cos(goal_node.yaw - current.yaw)))
            
            if dist_to_goal < 0.35 and yaw_diff < math.radians(25):
                print(f"[INFO] Hybrid A* path found! Total nodes expanded: {len(nodes_list)}")
                path = []
                curr = current
                while curr is not None:
                    path.append((curr.x, curr.y))
                    if curr.parent_index is not None:
                        curr = nodes_list[curr.parent_index]
                    else:
                        break
                path.reverse()
                return path

            for steer in self.steering_angles:
                for direction in [1, -1]:
                    step = self.step_size * direction
                    new_yaw = current.yaw + steer * direction
                    new_x = current.x + step * math.cos(new_yaw)
                    new_y = current.y + step * math.sin(new_yaw)

                    if not self.check_motion_safety(current.x, current.y, current.yaw, new_x, new_y, new_yaw):
                        continue

                    new_disc = get_discrete_index(HybridNode(new_x, new_y, new_yaw, 0, 0, 0))
                    if new_disc in closed_set:
                        continue

                    g_cost = current.g_cost + abs(step) + abs(steer) * 0.2
                    if direction == -1:
                        g_cost += 1.5
                    
                    h_cost = math.hypot(new_x - goal_node.x, new_y - goal_node.y)
                    
                    next_node = HybridNode(new_x, new_y, new_yaw, g_cost, h_cost, current_idx, direction)
                    nodes_list.append(next_node)
                    next_idx = len(nodes_list) - 1

                    heapq.heappush(open_set, (next_node.f_cost, next(counter), next_idx))

        print("[WARN] Hybrid A* failed to find a valid path to the goal!")
        return None

    def draw_path_blocking(self, path, rtab_pixel, window_title="Planned Path"):
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

        cv2.namedWindow(window_title, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window_title, self.width * 2, self.height * 2)
        
        print(f"[INFO] Displaying GUI window '{window_title}'. Click 'x' to exit and terminate...")
        
        while True:
            cv2.imshow(window_title, display)
            key = cv2.waitKey(50)
            if cv2.getWindowProperty(window_title, cv2.WND_PROP_VISIBLE) < 1:
                break
            if key != -1:
                break
                
        cv2.destroyAllWindows()

    def save_replan_image(self, path, rtab_pixel, filename="replan_result.png"):
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
        print(f"[INFO] Replan route visualization saved to {save_path}")

    def plan_path_to_goal_headless(self, goal_world_x, goal_world_y, goal_yaw=0.0):
        print("[INFO] Triggering RTAB-Map Map-Driven Dynamic Re-planning (Hybrid A*)...")
        
        if not self.load_dynamic_rtab_map():
            print("[ERROR] Failed to load dynamic map from RTAB-Map!")
            return False

        robot_pose = get_robot_pose(timeout_sec=3.0)
        if robot_pose is None:
            print("[ERROR] Failed to get robot pose for re-planning!")
            return False

        robot_x, robot_y = robot_pose
        px, py = self.world_to_pixel(robot_x, robot_y)
        start_pixel = self.find_nearest_free_cell((px, py))
        start_world = self.pixel_to_world(start_pixel[0], start_pixel[1])
        
        start_pose = (start_world[0], start_world[1], 0.0)
        goal_pose = (goal_world_x, goal_world_y, goal_yaw)

        world_path = self.hybrid_astar(start_pose, goal_pose)
        if world_path is None:
            print("[ERROR] Hybrid A* re-planning failed!")
            return False

        save_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "world_path.npy")
        np.save(save_path, {"path": world_path, "goal_yaw": goal_yaw}, allow_pickle=True)
        print(f"[INFO] 🗺️ 基于 Hybrid A* 的防穿墙平滑路径已生成并保存！")
        
        self.save_replan_image(world_path, (px, py), filename="replan_result.png")
        return True


if __name__ == "__main__":
    yaml_file = "/home/yz0000/wifibot_ws/src/wifibot_navigation/maps/indoor/map.yaml"
    
    if not rclpy.ok():
        rclpy.init()

    planner = AStarPlanner(yaml_file)

    if len(sys.argv) >= 3:
        goal_wx = float(sys.argv[1])
        goal_wy = float(sys.argv[2])
        goal_yw = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0
        
        success = planner.plan_path_to_goal_headless(goal_wx, goal_wy, goal_yw)
        
        if rclpy.ok():
            rclpy.shutdown()
        sys.exit(0 if success else 1)
    else:
        planner.load_static_map()
        
        robot_pose = get_robot_pose(timeout_sec=10.0)
        if robot_pose is None:
            print("[ERROR] Failed to obtain initial robot pose!")
            rclpy.shutdown()
            sys.exit(1)

        px, py = planner.world_to_pixel(robot_pose[0], robot_pose[1])
        start_pixel = planner.find_nearest_free_cell((px, py))
        start_world = planner.pixel_to_world(start_pixel[0], start_pixel[1])
        
        goal_pixel, goal_yaw = planner.select_goal()
        if goal_pixel is None:
            print("[ERROR] No goal selected or selection canceled!")
            rclpy.shutdown()
            sys.exit(1)

        goal_world_x, goal_world_y = planner.pixel_to_world(goal_pixel[0], goal_pixel[1])

        start_pose = (start_world[0], start_world[1], 0.0)
        goal_pose = (goal_world_x, goal_world_y, goal_yaw)

        world_path = planner.hybrid_astar(start_pose, goal_pose)
        if world_path is None:
            rclpy.shutdown()
            sys.exit(1)

        save_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "world_path.npy")
        np.save(save_path, {"path": world_path, "goal_yaw": goal_yaw}, allow_pickle=True)
        print(f"[INFO] Initial Hybrid A* path saved to {save_path} with {len(world_path)} waypoints.")
        
        planner.draw_path_blocking(world_path, (px, py), window_title="Initial Hybrid A* Path")
        rclpy.shutdown()
        sys.exit(0)
