#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import heapq
import math
import threading

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path


class AStarGlobalPlanner:
    def __init__(self):
        rospy.init_node("astar_global_planner")

        self.map_topic = rospy.get_param("~map_topic", "/occupancy_map")
        self.odom_topic = rospy.get_param("~odom_topic", "/odom")
        self.goal_topic = rospy.get_param("~goal_topic", "/move_base_simple/goal")
        self.path_topic = rospy.get_param("~path_topic", "/global_path")

        self.occupancy_threshold = rospy.get_param("~occupancy_threshold", 50)
        self.unknown_is_obstacle = rospy.get_param("~unknown_is_obstacle", False)
        self.replan_rate = rospy.get_param("~replan_rate", 1.0)
        self.goal_tolerance = rospy.get_param("~goal_tolerance", 0.20)

        self.lock = threading.Lock()
        self.map_msg = None
        self.odom_msg = None
        self.goal_msg = None
        self.goal_active = False

        self.path_pub = rospy.Publisher(self.path_topic, Path, queue_size=1, latch=True)
        rospy.Subscriber(self.map_topic, OccupancyGrid, self.map_callback, queue_size=1)
        rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=1)
        rospy.Subscriber(self.goal_topic, PoseStamped, self.goal_callback, queue_size=1)

        self.timer = rospy.Timer(rospy.Duration(1.0 / max(self.replan_rate, 0.1)), self.on_timer)

        rospy.loginfo("astar_global_planner started")
        rospy.loginfo("map_topic: %s", self.map_topic)
        rospy.loginfo("goal_topic: %s", self.goal_topic)
        rospy.loginfo("path_topic: %s", self.path_topic)

    def map_callback(self, msg):
        with self.lock:
            self.map_msg = msg

    def odom_callback(self, msg):
        with self.lock:
            self.odom_msg = msg

    def goal_callback(self, msg):
        with self.lock:
            self.goal_msg = msg
            self.goal_active = True
        rospy.loginfo("New goal received: (%.2f, %.2f)", msg.pose.position.x, msg.pose.position.y)

    def on_timer(self, _):
        with self.lock:
            map_msg = self.map_msg
            odom_msg = self.odom_msg
            goal_msg = self.goal_msg
            goal_active = self.goal_active

        if not goal_active or map_msg is None or odom_msg is None or goal_msg is None:
            return

        sx, sy = odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y
        gx, gy = goal_msg.pose.position.x, goal_msg.pose.position.y

        if math.hypot(gx - sx, gy - sy) <= self.goal_tolerance:
            with self.lock:
                self.goal_active = False
            self.path_pub.publish(Path(header=map_msg.header))
            rospy.loginfo_throttle(2.0, "Goal reached, stop replanning.")
            return

        start = self.world_to_grid(map_msg, sx, sy)
        goal = self.world_to_grid(map_msg, gx, gy)
        if start is None or goal is None:
            rospy.logwarn_throttle(1.0, "Start/goal out of map bounds.")
            return

        path_cells = self.astar(map_msg, start, goal)
        if not path_cells:
            rospy.logwarn_throttle(1.0, "A* failed to find path.")
            return

        path_msg = self.cells_to_path(map_msg, path_cells)
        self.path_pub.publish(path_msg)

    def astar(self, map_msg, start, goal):
        width = map_msg.info.width
        height = map_msg.info.height
        data = map_msg.data

        def in_bounds(cell):
            x, y = cell
            return 0 <= x < width and 0 <= y < height

        def is_free(cell):
            x, y = cell
            occ = data[y * width + x]
            if occ < 0:
                return not self.unknown_is_obstacle
            return occ < self.occupancy_threshold

        def h(cell):
            return math.hypot(goal[0] - cell[0], goal[1] - cell[1])

        neighbors = [
            (-1, -1), (0, -1), (1, -1),
            (-1, 0),           (1, 0),
            (-1, 1),  (0, 1),  (1, 1),
        ]

        if not is_free(start) or not is_free(goal):
            return None

        open_heap = []
        heapq.heappush(open_heap, (h(start), 0.0, start))
        g_score = {start: 0.0}
        came_from = {}
        closed = set()

        while open_heap:
            _, current_g, current = heapq.heappop(open_heap)
            if current in closed:
                continue
            closed.add(current)

            if current == goal:
                return self.reconstruct_path(came_from, current)

            cx, cy = current
            for dx, dy in neighbors:
                nxt = (cx + dx, cy + dy)
                if not in_bounds(nxt) or not is_free(nxt):
                    continue

                step_cost = math.sqrt(2.0) if dx != 0 and dy != 0 else 1.0
                tentative_g = current_g + step_cost
                if tentative_g < g_score.get(nxt, float("inf")):
                    g_score[nxt] = tentative_g
                    came_from[nxt] = current
                    f = tentative_g + h(nxt)
                    heapq.heappush(open_heap, (f, tentative_g, nxt))

        return None

    @staticmethod
    def reconstruct_path(came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

    @staticmethod
    def world_to_grid(map_msg, wx, wy):
        ox = map_msg.info.origin.position.x
        oy = map_msg.info.origin.position.y
        res = map_msg.info.resolution
        width = map_msg.info.width
        height = map_msg.info.height

        gx = int((wx - ox) / res)
        gy = int((wy - oy) / res)
        if 0 <= gx < width and 0 <= gy < height:
            return gx, gy
        return None

    @staticmethod
    def grid_to_world(map_msg, gx, gy):
        ox = map_msg.info.origin.position.x
        oy = map_msg.info.origin.position.y
        res = map_msg.info.resolution
        wx = ox + (gx + 0.5) * res
        wy = oy + (gy + 0.5) * res
        return wx, wy

    def cells_to_path(self, map_msg, cells):
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = map_msg.header.frame_id

        for gx, gy in cells:
            wx, wy = self.grid_to_world(map_msg, gx, gy)
            ps = PoseStamped()
            ps.header = path_msg.header
            ps.pose.position.x = wx
            ps.pose.position.y = wy
            ps.pose.orientation.w = 1.0
            path_msg.poses.append(ps)
        return path_msg


if __name__ == "__main__":
    try:
        AStarGlobalPlanner()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
