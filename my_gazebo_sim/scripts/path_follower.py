#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import threading

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from tf.transformations import euler_from_quaternion


class PathFollower:
    def __init__(self):
        rospy.init_node("path_follower")

        self.path_topic = rospy.get_param("~path_topic", "/global_path")
        self.odom_topic = rospy.get_param("~odom_topic", "/odom")
        self.cmd_topic = rospy.get_param("~cmd_topic", "/cmd_vel_raw")

        self.lookahead_dist = rospy.get_param("~lookahead_dist", 0.45)
        self.goal_tolerance = rospy.get_param("~goal_tolerance", 0.20)
        self.max_linear = rospy.get_param("~max_linear", 0.45)
        self.max_angular = rospy.get_param("~max_angular", 1.2)
        self.k_angular = rospy.get_param("~k_angular", 1.8)
        self.rotate_in_place_thresh = rospy.get_param("~rotate_in_place_thresh", 0.75)
        self.control_rate = rospy.get_param("~control_rate", 15.0)

        self.lock = threading.Lock()
        self.path_msg = None
        self.odom_msg = None

        self.cmd_pub = rospy.Publisher(self.cmd_topic, Twist, queue_size=1)
        rospy.Subscriber(self.path_topic, Path, self.path_callback, queue_size=1)
        rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=1)

        self.timer = rospy.Timer(rospy.Duration(1.0 / max(self.control_rate, 1.0)), self.on_timer)

        rospy.loginfo("path_follower started")
        rospy.loginfo("path_topic: %s", self.path_topic)
        rospy.loginfo("cmd_topic: %s", self.cmd_topic)

    def path_callback(self, msg):
        with self.lock:
            self.path_msg = msg

    def odom_callback(self, msg):
        with self.lock:
            self.odom_msg = msg

    def on_timer(self, _):
        with self.lock:
            path_msg = self.path_msg
            odom_msg = self.odom_msg

        cmd = Twist()

        if odom_msg is None or path_msg is None or len(path_msg.poses) == 0:
            self.cmd_pub.publish(cmd)
            return

        rx = odom_msg.pose.pose.position.x
        ry = odom_msg.pose.pose.position.y
        q = odom_msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

        gx = path_msg.poses[-1].pose.position.x
        gy = path_msg.poses[-1].pose.position.y
        goal_dist = math.hypot(gx - rx, gy - ry)
        if goal_dist <= self.goal_tolerance:
            self.cmd_pub.publish(cmd)
            return

        target = self.pick_lookahead_target(path_msg, rx, ry)
        tx, ty = target

        target_yaw = math.atan2(ty - ry, tx - rx)
        yaw_err = self.normalize_angle(target_yaw - yaw)
        abs_err = abs(yaw_err)

        if abs_err > self.rotate_in_place_thresh:
            cmd.linear.x = 0.0
            cmd.angular.z = self.clamp(self.k_angular * yaw_err, -self.max_angular, self.max_angular)
        else:
            turn_factor = max(0.0, 1.0 - abs_err / self.rotate_in_place_thresh)
            cmd.linear.x = self.max_linear * turn_factor
            cmd.angular.z = self.clamp(self.k_angular * yaw_err, -self.max_angular, self.max_angular)

        self.cmd_pub.publish(cmd)

    def pick_lookahead_target(self, path_msg, rx, ry):
        best = (path_msg.poses[-1].pose.position.x, path_msg.poses[-1].pose.position.y)
        for ps in path_msg.poses:
            px = ps.pose.position.x
            py = ps.pose.position.y
            if math.hypot(px - rx, py - ry) >= self.lookahead_dist:
                return px, py
        return best

    @staticmethod
    def normalize_angle(a):
        while a > math.pi:
            a -= 2.0 * math.pi
        while a < -math.pi:
            a += 2.0 * math.pi
        return a

    @staticmethod
    def clamp(v, lo, hi):
        return max(lo, min(hi, v))


if __name__ == "__main__":
    try:
        PathFollower()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
