#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import threading

import numpy as np
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_matrix, quaternion_from_matrix


def pose_to_mat(pos, quat):
    m = quaternion_matrix([quat.x, quat.y, quat.z, quat.w])
    m[0, 3] = pos.x
    m[1, 3] = pos.y
    m[2, 3] = pos.z
    return m


def mat_to_pose(m):
    q = quaternion_from_matrix(m)
    p = m[:3, 3]
    return p, q


class OrbOdomAligner:
    def __init__(self):
        rospy.init_node("orb_odom_aligner")

        self.input_topic = rospy.get_param("~input_topic", "/visual_slam/odom_raw")
        self.ref_topic = rospy.get_param("~ref_topic", "/odom")
        self.output_topic = rospy.get_param("~output_topic", "/visual_slam/odom_aligned")
        self.align_on_first_pair = bool(rospy.get_param("~align_on_first_pair", True))
        self.freeze_after_align = bool(rospy.get_param("~freeze_after_align", True))
        self.min_stamp_dt = float(rospy.get_param("~min_stamp_dt", 0.2))
        # Avoid blocking the whole navigation chain when ref odom is missing.
        # If True, publish raw odom on output before alignment is established.
        self.pass_through_before_align = bool(rospy.get_param("~pass_through_before_align", True))

        self.lock = threading.Lock()
        self.latest_in = None
        self.latest_ref = None
        self.latest_in_stamp = rospy.Time(0)
        self.latest_ref_stamp = rospy.Time(0)

        self.aligned = False
        self.t_ref_from_in = np.eye(4)
        self.warned_no_ref = False

        self.pub = rospy.Publisher(self.output_topic, Odometry, queue_size=50)
        rospy.Subscriber(self.input_topic, Odometry, self.in_cb, queue_size=100)
        rospy.Subscriber(self.ref_topic, Odometry, self.ref_cb, queue_size=50)
        self.timer = rospy.Timer(rospy.Duration(0.02), self.on_timer)  # 50Hz

        rospy.loginfo(
            "orb_odom_aligner started: in=%s ref=%s out=%s",
            self.input_topic,
            self.ref_topic,
            self.output_topic,
        )

    def in_cb(self, msg):
        with self.lock:
            self.latest_in = msg
            self.latest_in_stamp = rospy.Time.now()

    def ref_cb(self, msg):
        with self.lock:
            self.latest_ref = msg
            self.latest_ref_stamp = rospy.Time.now()

    def try_align(self):
        if self.aligned and self.freeze_after_align:
            return
        if self.latest_in is None or self.latest_ref is None:
            return

        t_now = rospy.Time.now()
        dt_in = (t_now - self.latest_in_stamp).to_sec()
        dt_ref = (t_now - self.latest_ref_stamp).to_sec()
        if dt_in > self.min_stamp_dt or dt_ref > self.min_stamp_dt:
            return

        in_pose = self.latest_in.pose.pose
        ref_pose = self.latest_ref.pose.pose
        t_in = pose_to_mat(in_pose.position, in_pose.orientation)
        t_ref = pose_to_mat(ref_pose.position, ref_pose.orientation)

        # T_ref_from_in maps ORB odom frame -> reference odom frame.
        self.t_ref_from_in = np.dot(t_ref, np.linalg.inv(t_in))
        self.aligned = True
        rospy.logwarn(
            "orb_odom_aligner: aligned ORB frame to reference frame (freeze=%s)",
            str(self.freeze_after_align),
        )

    def on_timer(self, _):
        with self.lock:
            if self.latest_in is None:
                return
            self.try_align()
            if not self.aligned:
                if not self.pass_through_before_align:
                    return
                if not self.warned_no_ref:
                    rospy.logwarn(
                        "orb_odom_aligner: ref odom not ready, passthrough raw odom to %s",
                        self.output_topic,
                    )
                    self.warned_no_ref = True
                out = Odometry()
                out.header.stamp = rospy.Time.now()
                out.header.frame_id = self.latest_in.header.frame_id
                out.child_frame_id = self.latest_in.child_frame_id
                out.pose = self.latest_in.pose
                out.twist = self.latest_in.twist
                self.pub.publish(out)
                return

            src = self.latest_in
            t_in_base = pose_to_mat(src.pose.pose.position, src.pose.pose.orientation)
            t_ref_base = np.dot(self.t_ref_from_in, t_in_base)
            p, q = mat_to_pose(t_ref_base)

            out = Odometry()
            out.header.stamp = rospy.Time.now()
            out.header.frame_id = self.latest_ref.header.frame_id if self.latest_ref is not None else src.header.frame_id
            out.child_frame_id = src.child_frame_id
            out.pose.pose.position.x = float(p[0])
            out.pose.pose.position.y = float(p[1])
            out.pose.pose.position.z = float(p[2])
            out.pose.pose.orientation.x = float(q[0])
            out.pose.pose.orientation.y = float(q[1])
            out.pose.pose.orientation.z = float(q[2])
            out.pose.pose.orientation.w = float(q[3])
            out.pose.covariance = src.pose.covariance
            out.twist = src.twist

            self.pub.publish(out)


if __name__ == "__main__":
    try:
        OrbOdomAligner()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
