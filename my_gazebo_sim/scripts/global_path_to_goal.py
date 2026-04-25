#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import threading

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path


class GlobalPathToGoal:
    def __init__(self):
        rospy.init_node("global_path_to_goal")

        self.path_topic = rospy.get_param("~path_topic", "/planner/global_path")
        self.odom_topic = rospy.get_param("~odom_topic", "/odom")
        self.user_goal_topic = rospy.get_param("~user_goal_topic", "/move_base_simple/goal")
        self.goal_out_topic = rospy.get_param("~goal_out_topic", "/planner/ego_goal")

        self.lookahead_distance = rospy.get_param("~lookahead_distance", 3.0)
        self.switch_distance = rospy.get_param("~switch_distance", 1.0)
        self.publish_interval = rospy.get_param("~publish_interval", 0.8)
        self.min_target_spacing = rospy.get_param("~min_target_spacing", 1.2)
        self.fallback_to_user_goal = rospy.get_param("~fallback_to_user_goal", True)

        self.lock = threading.Lock()
        self.latest_path = None
        self.latest_odom = None
        self.user_goal = None
        self.last_target = None

        self.goal_pub = rospy.Publisher(self.goal_out_topic, PoseStamped, queue_size=1)
        rospy.Subscriber(self.path_topic, Path, self.path_cb, queue_size=1)
        rospy.Subscriber(self.odom_topic, Odometry, self.odom_cb, queue_size=1)
        rospy.Subscriber(self.user_goal_topic, PoseStamped, self.user_goal_cb, queue_size=1)

        self.timer = rospy.Timer(rospy.Duration(max(self.publish_interval, 0.2)), self.on_timer)

        rospy.loginfo(
            "global_path_to_goal started path=%s odom=%s user_goal=%s out_goal=%s",
            self.path_topic, self.odom_topic, self.user_goal_topic, self.goal_out_topic
        )

    def path_cb(self, msg):
        with self.lock:
            self.latest_path = msg

    def odom_cb(self, msg):
        with self.lock:
            self.latest_odom = msg

    def user_goal_cb(self, msg):
        with self.lock:
            self.user_goal = msg
            # Force refresh on new user goal.
            self.last_target = None

    def on_timer(self, _):
        with self.lock:
            path = self.latest_path
            odom = self.latest_odom
            user_goal = self.user_goal
            last_target = self.last_target

        if odom is None:
            return

        target = self.select_target(path, odom, user_goal, last_target)
        if target is None:
            return

        if last_target is not None:
            if self.dist_xy(last_target.pose.position, target.pose.position) < self.min_target_spacing:
                target = last_target

        self.goal_pub.publish(target)
        with self.lock:
            self.last_target = target

    def select_target(self, path, odom, user_goal, last_target):
        if path is None or len(path.poses) == 0:
            if self.fallback_to_user_goal and user_goal is not None:
                return user_goal
            return None

        rx = odom.pose.pose.position.x
        ry = odom.pose.pose.position.y

        poses = path.poses
        nearest_idx = self.find_nearest_index(poses, rx, ry)
        pick_idx = nearest_idx

        for i in range(nearest_idx, len(poses)):
            px = poses[i].pose.position.x
            py = poses[i].pose.position.y
            d = math.hypot(px - rx, py - ry)
            if d >= self.lookahead_distance:
                pick_idx = i
                break
        else:
            pick_idx = len(poses) - 1

        ps = PoseStamped()
        ps.header.stamp = rospy.Time.now()
        ps.header.frame_id = poses[pick_idx].header.frame_id or path.header.frame_id or "odom"
        ps.pose = poses[pick_idx].pose

        # Keep z from user goal when available, otherwise use path z.
        if user_goal is not None:
            ps.pose.position.z = user_goal.pose.position.z if abs(user_goal.pose.position.z) > 1e-3 else 1.0
        elif abs(ps.pose.position.z) < 1e-3:
            ps.pose.position.z = 1.0

        # Only update to a new target if we've progressed near the old one.
        if last_target is not None:
            d_old = self.dist_xy(odom.pose.pose.position, last_target.pose.position)
            if d_old > self.switch_distance:
                return last_target

        return ps

    @staticmethod
    def find_nearest_index(poses, rx, ry):
        best_i = 0
        best_d = float("inf")
        for i, p in enumerate(poses):
            px = p.pose.position.x
            py = p.pose.position.y
            d = (px - rx) * (px - rx) + (py - ry) * (py - ry)
            if d < best_d:
                best_d = d
                best_i = i
        return best_i

    @staticmethod
    def dist_xy(p1, p2):
        return math.hypot(p1.x - p2.x, p1.y - p2.y)


if __name__ == "__main__":
    try:
        GlobalPathToGoal()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

