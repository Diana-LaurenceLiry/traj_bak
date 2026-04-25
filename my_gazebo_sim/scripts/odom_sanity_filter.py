#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import threading
import copy

import rospy
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry


class OdomSanityFilter:
    def __init__(self):
        rospy.init_node("odom_sanity_filter")

        self.input_topic = rospy.get_param("~input_topic", "/visual_slam/odom")
        self.fallback_topic = rospy.get_param("~fallback_topic", "/odom")
        self.output_topic = rospy.get_param("~output_topic", "/visual_slam/odom_safe")
        self.model_states_topic = rospy.get_param("~model_states_topic", "/gazebo/model_states")
        self.model_name = rospy.get_param("~model_name", "simple_uav")

        self.use_fallback = rospy.get_param("~use_fallback", True)
        self.min_z = rospy.get_param("~min_z", 0.0)
        self.max_z = rospy.get_param("~max_z", 5.0)
        self.max_jump_xy = rospy.get_param("~max_jump_xy", 1.0)
        self.max_jump_z = rospy.get_param("~max_jump_z", 0.8)
        self.max_abs_speed = rospy.get_param("~max_abs_speed", 4.0)
        self.jump_ref_timeout = rospy.get_param("~jump_ref_timeout", 0.8)
        self.relock_reject_count = int(rospy.get_param("~relock_reject_count", 25))
        self.stale_timeout = rospy.get_param("~stale_timeout", 0.5)
        self.blocked_timeout = rospy.get_param("~blocked_timeout", 0.8)
        self.allow_raw_when_blocked = rospy.get_param("~allow_raw_when_blocked", True)
        self.accept_z_outlier_with_clamp = rospy.get_param("~accept_z_outlier_with_clamp", True)
        self.rate = rospy.get_param("~rate", 50.0)

        self.lock = threading.Lock()
        self.latest_input = None
        self.latest_input_stamp = rospy.Time(0)
        self.last_good = None
        self.last_good_stamp = rospy.Time(0)
        self.latest_fallback = None
        self.latest_fallback_stamp = rospy.Time(0)
        self.latest_model_odom = None
        self.latest_model_odom_stamp = rospy.Time(0)
        self.model_index = None
        self.consecutive_rejects = 0

        self.pub = rospy.Publisher(self.output_topic, Odometry, queue_size=20)
        rospy.Subscriber(self.input_topic, Odometry, self.input_cb, queue_size=50)
        if self.use_fallback and self.fallback_topic:
            rospy.Subscriber(self.fallback_topic, Odometry, self.fallback_cb, queue_size=20)
        if self.use_fallback and self.model_states_topic:
            rospy.Subscriber(self.model_states_topic, ModelStates, self.model_states_cb, queue_size=20)
        self.timer = rospy.Timer(rospy.Duration(1.0 / max(self.rate, 1.0)), self.on_timer)

        rospy.loginfo(
            "odom_sanity_filter started: in=%s fallback=%s model_states=%s out=%s",
            self.input_topic,
            self.fallback_topic,
            self.model_states_topic,
            self.output_topic,
        )

    @staticmethod
    def finite_pose(msg):
        p = msg.pose.pose.position
        return all(math.isfinite(v) for v in [p.x, p.y, p.z])

    def is_reasonable(self, msg, now):
        if not self.finite_pose(msg):
            return False, "non_finite_pose"

        p = msg.pose.pose.position
        if p.z < self.min_z or p.z > self.max_z:
            return False, "z_out_of_range"

        tw = msg.twist.twist
        if (
            abs(tw.linear.x) > self.max_abs_speed
            or abs(tw.linear.y) > self.max_abs_speed
            or abs(tw.linear.z) > self.max_abs_speed
        ):
            return False, "speed_out_of_range"

        if self.last_good is not None:
            ref_age = (now - self.last_good_stamp).to_sec()
            # If reference sample is stale, do not keep hard jump constraints.
            if ref_age <= self.jump_ref_timeout:
                lp = self.last_good.pose.pose.position
                dxy = math.hypot(p.x - lp.x, p.y - lp.y)
                dz = abs(p.z - lp.z)
                if dxy > self.max_jump_xy or dz > self.max_jump_z:
                    return False, "jump_too_large"

        return True, "ok"

    def input_cb(self, msg):
        now = rospy.Time.now()
        with self.lock:
            self.latest_input = msg
            self.latest_input_stamp = now
            ok, reason = self.is_reasonable(msg, now)
            if ok:
                self.last_good = msg
                self.last_good_stamp = now
                self.consecutive_rejects = 0
            else:
                # Common ORB init/drift issue: z goes out of range while xy is still usable.
                # Accept this sample after clamping z to keep navigation chain alive.
                if reason == "z_out_of_range" and self.accept_z_outlier_with_clamp:
                    zfixed = copy.deepcopy(msg)
                    p = zfixed.pose.pose.position
                    p.z = max(self.min_z, min(self.max_z, p.z))
                    zfixed.twist.twist.linear.z = 0.0
                    self.last_good = zfixed
                    self.last_good_stamp = now
                    self.consecutive_rejects = 0
                    rospy.logwarn_throttle(
                        1.0,
                        "odom_sanity_filter accepted ORB with z clamp: raw_z=%.2f -> %.2f",
                        msg.pose.pose.position.z,
                        p.z,
                    )
                    return

                self.consecutive_rejects += 1
                p = msg.pose.pose.position
                tw = msg.twist.twist
                # Auto-relock to current ORB pose if jump rejects persist.
                if reason == "jump_too_large" and self.consecutive_rejects >= self.relock_reject_count:
                    self.last_good = msg
                    self.last_good_stamp = now
                    self.consecutive_rejects = 0
                    rospy.logwarn(
                        "odom_sanity_filter relocked on ORB odom after repeated jump rejects at pos=(%.2f, %.2f, %.2f)",
                        p.x,
                        p.y,
                        p.z,
                    )
                    return
                rospy.logwarn_throttle(
                    1.0,
                    "odom_sanity_filter rejected ORB odom sample: reason=%s pos=(%.2f, %.2f, %.2f) vel=(%.2f, %.2f, %.2f) rej=%d",
                    reason,
                    p.x,
                    p.y,
                    p.z,
                    tw.linear.x,
                    tw.linear.y,
                    tw.linear.z,
                    self.consecutive_rejects,
                )

    def fallback_cb(self, msg):
        with self.lock:
            self.latest_fallback = msg
            self.latest_fallback_stamp = rospy.Time.now()

    def model_states_cb(self, msg):
        with self.lock:
            if self.model_index is None:
                for i, name in enumerate(msg.name):
                    if name == self.model_name:
                        self.model_index = i
                        break
            if self.model_index is None:
                return
            if self.model_index >= len(msg.pose) or self.model_index >= len(msg.twist):
                return

            od = Odometry()
            od.header.stamp = rospy.Time.now()
            od.header.frame_id = "odom"
            od.child_frame_id = "base_link"
            od.pose.pose = msg.pose[self.model_index]
            od.twist.twist = msg.twist[self.model_index]
            self.latest_model_odom = od
            self.latest_model_odom_stamp = rospy.Time.now()

    def on_timer(self, _):
        out = None
        now = rospy.Time.now()
        with self.lock:
            if self.last_good is not None and (now - self.last_good_stamp).to_sec() <= self.stale_timeout:
                out = self.last_good
            elif self.use_fallback and self.latest_fallback is not None:
                out = self.latest_fallback
                rospy.logwarn_throttle(1.0, "odom_sanity_filter using fallback odom")
            elif self.use_fallback and self.latest_model_odom is not None:
                out = self.latest_model_odom
                rospy.logwarn_throttle(1.0, "odom_sanity_filter using model_states fallback odom")
            elif (
                self.allow_raw_when_blocked
                and self.latest_input is not None
                and (now - self.latest_input_stamp).to_sec() <= self.blocked_timeout
            ):
                out = self.latest_input
                rospy.logwarn_throttle(1.0, "odom_sanity_filter temporarily forwarding raw ORB odom")
            elif self.last_good is not None:
                # Last resort: keep previous valid sample only when no fresh raw/fallback is available.
                out = self.last_good
                rospy.logwarn_throttle(1.0, "odom_sanity_filter holding last good odom (no fresh input)")

        if out is not None:
            out_msg = copy.deepcopy(out)
            p = out_msg.pose.pose.position
            # Final guard: never publish z outside planner-valid range.
            if p.z < self.min_z:
                p.z = self.min_z
                out_msg.twist.twist.linear.z = 0.0
            elif p.z > self.max_z:
                p.z = self.max_z
                out_msg.twist.twist.linear.z = 0.0
            out_msg.header.stamp = now
            self.pub.publish(out_msg)


if __name__ == "__main__":
    try:
        OdomSanityFilter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
