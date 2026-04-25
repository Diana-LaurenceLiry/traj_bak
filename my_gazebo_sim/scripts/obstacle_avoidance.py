#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import threading

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from tf.transformations import euler_from_quaternion


class ObstacleAvoidance:
    def __init__(self):
        rospy.init_node("obstacle_avoidance")

        self.enabled = rospy.get_param("~enabled", True)

        self.cmd_in_topic = rospy.get_param("~cmd_in_topic", "/cmd_vel_raw")
        self.cmd_out_topic = rospy.get_param("~cmd_out_topic", "/cmd_vel")
        self.map_topic = rospy.get_param("~map_topic", "/occupancy_map")
        self.odom_topic = rospy.get_param("~odom_topic", "/odom")

        self.forward_distance = rospy.get_param("~forward_distance", 0.9)
        self.corridor_half_width = rospy.get_param("~corridor_half_width", 0.35)
        self.critical_distance = rospy.get_param("~critical_distance", 0.35)
        self.slowdown_distance = rospy.get_param("~slowdown_distance", 0.9)
        self.occupancy_threshold = rospy.get_param("~occupancy_threshold", 50)
        self.unknown_is_obstacle = rospy.get_param("~unknown_is_obstacle", False)

        self.min_turn_rate = rospy.get_param("~min_turn_rate", 0.35)
        self.max_turn_rate = rospy.get_param("~max_turn_rate", 1.2)
        self.max_output_linear = rospy.get_param("~max_output_linear", 0.8)

        self.last_turn_sign = 1.0
        self.lock = threading.Lock()

        self.latest_map = None
        self.latest_odom = None

        self.cmd_pub = rospy.Publisher(self.cmd_out_topic, Twist, queue_size=1)
        rospy.Subscriber(self.cmd_in_topic, Twist, self.cmd_callback, queue_size=1)
        rospy.Subscriber(self.map_topic, OccupancyGrid, self.map_callback, queue_size=1)
        rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=1)

        rospy.loginfo("obstacle_avoidance started")
        rospy.loginfo("cmd_in_topic: %s", self.cmd_in_topic)
        rospy.loginfo("cmd_out_topic: %s", self.cmd_out_topic)
        rospy.loginfo("map_topic: %s", self.map_topic)
        rospy.loginfo("odom_topic: %s", self.odom_topic)

    def map_callback(self, msg):
        with self.lock:
            self.latest_map = msg

    def odom_callback(self, msg):
        with self.lock:
            self.latest_odom = msg

    def cmd_callback(self, cmd_in):
        if not self.enabled:
            self.cmd_pub.publish(cmd_in)
            return

        with self.lock:
            grid = self.latest_map
            odom = self.latest_odom

        if grid is None or odom is None:
            self.cmd_pub.publish(cmd_in)
            return

        cmd_out = Twist()
        cmd_out.linear.x = cmd_in.linear.x
        cmd_out.angular.z = cmd_in.angular.z

        nearest, left_hits, right_hits = self.scan_front_corridor(grid, odom)

        if math.isinf(nearest):
            cmd_out.linear.x = self.clamp(cmd_out.linear.x, -self.max_output_linear, self.max_output_linear)
            self.cmd_pub.publish(cmd_out)
            return

        if nearest <= self.critical_distance:
            slowdown_ratio = 0.0
        elif nearest >= self.slowdown_distance:
            slowdown_ratio = 1.0
        else:
            span = max(self.slowdown_distance - self.critical_distance, 1e-3)
            slowdown_ratio = (nearest - self.critical_distance) / span

        if cmd_in.linear.x > 0.0:
            cmd_out.linear.x = cmd_in.linear.x * slowdown_ratio
            if cmd_out.linear.x < 0.02:
                cmd_out.linear.x = 0.0

        turn_sign = self.select_turn_sign(left_hits, right_hits)
        proximity = 1.0 - slowdown_ratio
        if proximity > 0.0:
            turn_bias = turn_sign * (self.min_turn_rate + (self.max_turn_rate - self.min_turn_rate) * proximity)
            cmd_out.angular.z = cmd_in.angular.z + turn_bias

        cmd_out.linear.x = self.clamp(cmd_out.linear.x, -self.max_output_linear, self.max_output_linear)
        cmd_out.angular.z = self.clamp(cmd_out.angular.z, -self.max_turn_rate, self.max_turn_rate)
        self.cmd_pub.publish(cmd_out)

    def scan_front_corridor(self, grid, odom):
        origin_x = grid.info.origin.position.x
        origin_y = grid.info.origin.position.y
        resolution = grid.info.resolution
        width = grid.info.width
        height = grid.info.height

        px = odom.pose.pose.position.x
        py = odom.pose.pose.position.y
        q = odom.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

        step_x = max(0.05, resolution * 2.0)
        step_y = max(0.05, resolution * 2.0)

        nearest = float("inf")
        left_hits = 0
        right_hits = 0

        x = step_x
        while x <= self.forward_distance:
            y = -self.corridor_half_width
            while y <= self.corridor_half_width:
                wx = px + x * math.cos(yaw) - y * math.sin(yaw)
                wy = py + x * math.sin(yaw) + y * math.cos(yaw)

                gx = int((wx - origin_x) / resolution)
                gy = int((wy - origin_y) / resolution)

                if 0 <= gx < width and 0 <= gy < height:
                    idx = gy * width + gx
                    occ = grid.data[idx]
                    occupied = occ >= self.occupancy_threshold or (self.unknown_is_obstacle and occ < 0)

                    if occupied:
                        if x < nearest:
                            nearest = x
                        if y >= 0.0:
                            left_hits += 1
                        else:
                            right_hits += 1

                y += step_y
            x += step_x

        return nearest, left_hits, right_hits

    def select_turn_sign(self, left_hits, right_hits):
        if left_hits > right_hits:
            self.last_turn_sign = -1.0
        elif right_hits > left_hits:
            self.last_turn_sign = 1.0
        return self.last_turn_sign

    @staticmethod
    def clamp(value, low, high):
        return max(low, min(high, value))


if __name__ == "__main__":
    try:
        ObstacleAvoidance()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
