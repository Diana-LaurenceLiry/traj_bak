#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import threading

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32


class GoalReachFeedback:
    def __init__(self):
        rospy.init_node("goal_reach_feedback")

        self.odom_topic = rospy.get_param("~odom_topic", "/visual_slam/odom")
        self.goal_topic = rospy.get_param("~goal_topic", "/move_base_simple/goal")
        self.use_2d_goal = rospy.get_param("~use_2d_goal", True)
        self.default_goal_z = rospy.get_param("~default_goal_z", 1.0)
        self.xy_tolerance = rospy.get_param("~xy_tolerance", 0.35)
        self.z_tolerance = rospy.get_param("~z_tolerance", 0.30)
        self.hold_time = rospy.get_param("~hold_time", 0.6)
        self.rate = rospy.get_param("~rate", 10.0)

        self.lock = threading.Lock()
        self.latest_odom = None
        self.goal = None
        self.goal_active = False
        self.arrive_since = None
        self.last_reported_reached = False

        self.reached_pub = rospy.Publisher("~reached", Bool, queue_size=1)
        self.dist_pub = rospy.Publisher("~distance", Float32, queue_size=1)

        rospy.Subscriber(self.odom_topic, Odometry, self.odom_cb, queue_size=1)
        rospy.Subscriber(self.goal_topic, PoseStamped, self.goal_cb, queue_size=1)

        self.timer = rospy.Timer(rospy.Duration(1.0 / max(self.rate, 1.0)), self.on_timer)
        rospy.loginfo("goal_reach_feedback started, odom=%s, goal=%s", self.odom_topic, self.goal_topic)

    def odom_cb(self, msg):
        with self.lock:
            self.latest_odom = msg

    def goal_cb(self, msg):
        with self.lock:
            gz = msg.pose.position.z
            if self.use_2d_goal and abs(gz) < 1e-3:
                gz = self.default_goal_z
            self.goal = (msg.pose.position.x, msg.pose.position.y, gz)
            self.goal_active = True
            self.arrive_since = None
            self.last_reported_reached = False
        rospy.loginfo("New goal received: (%.2f, %.2f, %.2f)", self.goal[0], self.goal[1], self.goal[2])

    def on_timer(self, _):
        with self.lock:
            odom = self.latest_odom
            goal = self.goal
            active = self.goal_active

        if odom is None or goal is None or not active:
            self.reached_pub.publish(Bool(data=False))
            return

        px = odom.pose.pose.position.x
        py = odom.pose.pose.position.y
        pz = odom.pose.pose.position.z
        gx, gy, gz = goal

        dxy = math.hypot(px - gx, py - gy)
        dz = abs(pz - gz)
        dist3d = math.sqrt((px - gx) ** 2 + (py - gy) ** 2 + (pz - gz) ** 2)
        self.dist_pub.publish(Float32(data=dist3d))

        within = (dxy <= self.xy_tolerance) and (dz <= self.z_tolerance)
        reached = False
        now = rospy.Time.now()

        with self.lock:
            if within:
                if self.arrive_since is None:
                    self.arrive_since = now
                elif (now - self.arrive_since).to_sec() >= self.hold_time:
                    reached = True
                    self.goal_active = False
            else:
                self.arrive_since = None

            if reached and not self.last_reported_reached:
                rospy.loginfo(
                    "Goal reached. Current=(%.2f, %.2f, %.2f), Goal=(%.2f, %.2f, %.2f), dxy=%.2f dz=%.2f",
                    px, py, pz, gx, gy, gz, dxy, dz,
                )
                self.last_reported_reached = True

        self.reached_pub.publish(Bool(data=reached))


if __name__ == "__main__":
    try:
        GoalReachFeedback()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
