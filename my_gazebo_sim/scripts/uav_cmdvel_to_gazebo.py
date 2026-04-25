#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import threading

import rospy
from gazebo_msgs.msg import ModelState, ModelStates
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion


class UavCmdVelToGazebo:
    def __init__(self):
        rospy.init_node("uav_cmdvel_to_gazebo")

        self.model_name = rospy.get_param("~model_name", "simple_uav")
        self.cmd_topic = rospy.get_param("~cmd_topic", "/cmd_vel")
        self.model_states_topic = rospy.get_param("~model_states_topic", "/gazebo/model_states")
        self.set_state_service = rospy.get_param("~set_state_service", "/gazebo/set_model_state")
        self.control_rate = rospy.get_param("~control_rate", 30.0)
        self.cmd_timeout = rospy.get_param("~cmd_timeout", 0.5)

        self.max_xy_speed = rospy.get_param("~max_xy_speed", 1.5)
        self.max_z_speed = rospy.get_param("~max_z_speed", 1.0)
        self.max_yaw_rate = rospy.get_param("~max_yaw_rate", 1.5)

        self.lock = threading.Lock()
        self.latest_cmd = Twist()
        self.latest_cmd_stamp = rospy.Time(0)
        self.latest_pose = None
        self.model_index = None

        rospy.Subscriber(self.cmd_topic, Twist, self.cmd_callback, queue_size=1)
        rospy.Subscriber(self.model_states_topic, ModelStates, self.model_states_callback, queue_size=1)

        rospy.loginfo("waiting for %s ...", self.set_state_service)
        rospy.wait_for_service(self.set_state_service)
        self.set_state = rospy.ServiceProxy(self.set_state_service, SetModelState)

        self.timer = rospy.Timer(rospy.Duration(1.0 / max(self.control_rate, 1.0)), self.on_timer)
        rospy.loginfo("uav_cmdvel_to_gazebo started, model=%s, cmd=%s", self.model_name, self.cmd_topic)

    def cmd_callback(self, msg):
        with self.lock:
            self.latest_cmd = msg
            self.latest_cmd_stamp = rospy.Time.now()

    def model_states_callback(self, msg):
        with self.lock:
            if self.model_index is None:
                for i, name in enumerate(msg.name):
                    if name == self.model_name:
                        self.model_index = i
                        break
            if self.model_index is not None and self.model_index < len(msg.pose):
                self.latest_pose = msg.pose[self.model_index]

    def on_timer(self, _):
        with self.lock:
            pose = self.latest_pose
            cmd = self.latest_cmd
            cmd_age = (rospy.Time.now() - self.latest_cmd_stamp).to_sec()

        if pose is None:
            return

        vx_body = 0.0 if cmd_age > self.cmd_timeout else cmd.linear.x
        vy_body = 0.0 if cmd_age > self.cmd_timeout else cmd.linear.y
        vz = 0.0 if cmd_age > self.cmd_timeout else cmd.linear.z
        wz = 0.0 if cmd_age > self.cmd_timeout else cmd.angular.z

        vx_body = self.clamp(vx_body, -self.max_xy_speed, self.max_xy_speed)
        vy_body = self.clamp(vy_body, -self.max_xy_speed, self.max_xy_speed)
        vz = self.clamp(vz, -self.max_z_speed, self.max_z_speed)
        wz = self.clamp(wz, -self.max_yaw_rate, self.max_yaw_rate)

        q = pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

        vx_world = vx_body * math.cos(yaw) - vy_body * math.sin(yaw)
        vy_world = vx_body * math.sin(yaw) + vy_body * math.cos(yaw)

        req = ModelState()
        req.model_name = self.model_name
        req.pose = pose
        req.reference_frame = "world"
        req.twist.linear.x = vx_world
        req.twist.linear.y = vy_world
        req.twist.linear.z = vz
        req.twist.angular.x = 0.0
        req.twist.angular.y = 0.0
        req.twist.angular.z = wz

        try:
            self.set_state(req)
        except rospy.ServiceException as e:
            rospy.logwarn_throttle(1.0, "set_model_state failed: %s", str(e))

    @staticmethod
    def clamp(v, lo, hi):
        return max(lo, min(hi, v))


if __name__ == "__main__":
    try:
        UavCmdVelToGazebo()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
