#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import threading

import rospy
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud2


class ObstacleAvoidance3D:
    def __init__(self):
        rospy.init_node("obstacle_avoidance_3d")

        self.enabled = rospy.get_param("~enabled", True)
        self.cmd_in_topic = rospy.get_param("~cmd_in_topic", "/cmd_vel_raw")
        self.cmd_out_topic = rospy.get_param("~cmd_out_topic", "/cmd_vel")
        self.cloud_topic = rospy.get_param("~cloud_topic", "/depth_camera/depth/points")

        self.forward_distance = rospy.get_param("~forward_distance", 2.0)
        self.lateral_half_width = rospy.get_param("~lateral_half_width", 0.6)
        self.vertical_half_height = rospy.get_param("~vertical_half_height", 0.6)
        self.min_range = rospy.get_param("~min_range", 0.2)
        self.max_range = rospy.get_param("~max_range", 10.0)

        self.critical_distance = rospy.get_param("~critical_distance", 0.8)
        self.slowdown_distance = rospy.get_param("~slowdown_distance", 1.8)
        self.min_turn_rate = rospy.get_param("~min_turn_rate", 0.3)
        self.max_turn_rate = rospy.get_param("~max_turn_rate", 1.2)
        self.max_vertical_speed = rospy.get_param("~max_vertical_speed", 0.6)
        self.max_linear_speed = rospy.get_param("~max_linear_speed", 1.0)
        self.safe_ceiling_clearance = rospy.get_param("~safe_ceiling_clearance", 0.7)
        self.safe_floor_clearance = rospy.get_param("~safe_floor_clearance", 0.5)

        self.lock = threading.Lock()
        self.last_turn_sign = 1.0
        self.latest_cloud = None

        self.cmd_pub = rospy.Publisher(self.cmd_out_topic, Twist, queue_size=1)
        rospy.Subscriber(self.cmd_in_topic, Twist, self.cmd_callback, queue_size=1)
        rospy.Subscriber(self.cloud_topic, PointCloud2, self.cloud_callback, queue_size=1)

        rospy.loginfo("obstacle_avoidance_3d started")
        rospy.loginfo("cmd_in_topic: %s", self.cmd_in_topic)
        rospy.loginfo("cmd_out_topic: %s", self.cmd_out_topic)
        rospy.loginfo("cloud_topic: %s", self.cloud_topic)

    def cloud_callback(self, msg):
        with self.lock:
            self.latest_cloud = msg

    def cmd_callback(self, cmd_in):
        if not self.enabled:
            self.cmd_pub.publish(cmd_in)
            return

        with self.lock:
            cloud = self.latest_cloud

        if cloud is None:
            self.cmd_pub.publish(cmd_in)
            return

        nearest_front, left_hits, right_hits, min_up_clearance, min_down_clearance = self.scan_cloud(cloud)

        cmd_out = Twist()
        cmd_out.linear.x = cmd_in.linear.x
        cmd_out.linear.y = cmd_in.linear.y
        cmd_out.linear.z = cmd_in.linear.z
        cmd_out.angular.z = cmd_in.angular.z

        if math.isfinite(nearest_front):
            slowdown_ratio = self.compute_slowdown_ratio(nearest_front)

            if cmd_in.linear.x > 0.0:
                cmd_out.linear.x = cmd_in.linear.x * slowdown_ratio
                if cmd_out.linear.x < 0.02:
                    cmd_out.linear.x = 0.0

            proximity = 1.0 - slowdown_ratio
            if proximity > 0.0:
                turn_sign = self.select_turn_sign(left_hits, right_hits)
                turn_bias = turn_sign * (
                    self.min_turn_rate + (self.max_turn_rate - self.min_turn_rate) * proximity
                )
                cmd_out.angular.z = cmd_in.angular.z + turn_bias

                climb_cmd = self.compute_vertical_bias(min_up_clearance, min_down_clearance, proximity)
                cmd_out.linear.z = cmd_in.linear.z + climb_cmd

        cmd_out.linear.x = self.clamp(cmd_out.linear.x, -self.max_linear_speed, self.max_linear_speed)
        cmd_out.linear.z = self.clamp(cmd_out.linear.z, -self.max_vertical_speed, self.max_vertical_speed)
        cmd_out.angular.z = self.clamp(cmd_out.angular.z, -self.max_turn_rate, self.max_turn_rate)
        self.cmd_pub.publish(cmd_out)

    def scan_cloud(self, cloud_msg):
        nearest_front = float("inf")
        left_hits = 0
        right_hits = 0
        min_up_clearance = float("inf")
        min_down_clearance = float("inf")

        for p in pc2.read_points(cloud_msg, field_names=("x", "y", "z"), skip_nans=True):
            x, y, z = p

            r = math.sqrt(x * x + y * y + z * z)
            if r < self.min_range or r > self.max_range:
                continue

            if x <= 0.0 or x > self.forward_distance:
                continue
            if abs(y) > self.lateral_half_width:
                continue
            if abs(z) > self.vertical_half_height:
                continue

            if x < nearest_front:
                nearest_front = x

            if y >= 0.0:
                left_hits += 1
            else:
                right_hits += 1

            if z > 0.0 and z < min_up_clearance:
                min_up_clearance = z
            if z < 0.0 and abs(z) < min_down_clearance:
                min_down_clearance = abs(z)

        return nearest_front, left_hits, right_hits, min_up_clearance, min_down_clearance

    def compute_slowdown_ratio(self, nearest_front):
        if nearest_front <= self.critical_distance:
            return 0.0
        if nearest_front >= self.slowdown_distance:
            return 1.0
        span = max(self.slowdown_distance - self.critical_distance, 1e-3)
        return (nearest_front - self.critical_distance) / span

    def select_turn_sign(self, left_hits, right_hits):
        if left_hits > right_hits:
            self.last_turn_sign = -1.0
        elif right_hits > left_hits:
            self.last_turn_sign = 1.0
        return self.last_turn_sign

    def compute_vertical_bias(self, up_clearance, down_clearance, proximity):
        # Prefer moving to the side with more clearance.
        up_safe = up_clearance > self.safe_ceiling_clearance
        down_safe = down_clearance > self.safe_floor_clearance

        if up_safe and not down_safe:
            return 0.6 * self.max_vertical_speed * proximity
        if down_safe and not up_safe:
            return -0.6 * self.max_vertical_speed * proximity
        if up_safe and down_safe:
            return 0.3 * self.max_vertical_speed * proximity
        return 0.0

    @staticmethod
    def clamp(value, lo, hi):
        return max(lo, min(hi, value))


if __name__ == "__main__":
    try:
        ObstacleAvoidance3D()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
