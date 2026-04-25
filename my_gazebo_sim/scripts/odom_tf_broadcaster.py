#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import tf2_ros
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry


class OdomTfBroadcaster:
    def __init__(self):
        rospy.init_node("odom_tf_broadcaster")

        self.odom_topic = rospy.get_param("~odom_topic", "/odom")
        self.model_states_topic = rospy.get_param("~model_states_topic", "/gazebo/model_states")
        self.model_name = rospy.get_param("~model_name", "simple_uav")
        self.parent_frame = rospy.get_param("~parent_frame", "odom")
        self.child_frame = rospy.get_param("~child_frame", "base_link")
        self.stale_timeout = float(rospy.get_param("~stale_timeout", 0.5))
        self.use_model_states_fallback = bool(rospy.get_param("~use_model_states_fallback", True))

        self.last_odom_time = rospy.Time(0)

        self.br = tf2_ros.TransformBroadcaster()
        self.odom_pub = rospy.Publisher(self.odom_topic, Odometry, queue_size=10)
        rospy.Subscriber(self.odom_topic, Odometry, self.odom_cb, queue_size=50)
        rospy.Subscriber(self.model_states_topic, ModelStates, self.model_states_cb, queue_size=20)
        rospy.loginfo(
            "odom_tf_broadcaster started: odom_topic=%s model_states=%s model=%s %s->%s",
            self.odom_topic,
            self.model_states_topic,
            self.model_name,
            self.parent_frame,
            self.child_frame,
        )

    def odom_cb(self, msg):
        self.last_odom_time = rospy.Time.now()
        self.publish_tf_from_odom(msg)

    def model_states_cb(self, msg):
        # Use model_states as fallback when /odom is missing/stale.
        if not self.use_model_states_fallback:
            return
        if (rospy.Time.now() - self.last_odom_time).to_sec() <= self.stale_timeout:
            return

        try:
            idx = msg.name.index(self.model_name)
        except ValueError:
            rospy.logwarn_throttle(2.0, "odom_tf_broadcaster: model '%s' not found in model_states", self.model_name)
            return

        od = Odometry()
        od.header.stamp = rospy.Time.now()
        od.header.frame_id = self.parent_frame
        od.child_frame_id = self.child_frame
        od.pose.pose = msg.pose[idx]
        od.twist.twist = msg.twist[idx]
        self.odom_pub.publish(od)
        self.publish_tf_from_odom(od)
        rospy.logwarn_throttle(2.0, "odom_tf_broadcaster: using model_states fallback for /odom")

    def publish_tf_from_odom(self, odom_msg):
        t = TransformStamped()
        t.header.stamp = odom_msg.header.stamp if odom_msg.header.stamp != rospy.Time(0) else rospy.Time.now()
        t.header.frame_id = self.parent_frame
        t.child_frame_id = self.child_frame
        t.transform.translation.x = odom_msg.pose.pose.position.x
        t.transform.translation.y = odom_msg.pose.pose.position.y
        t.transform.translation.z = odom_msg.pose.pose.position.z
        t.transform.rotation = odom_msg.pose.pose.orientation
        self.br.sendTransform(t)


if __name__ == "__main__":
    try:
        OdomTfBroadcaster()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
