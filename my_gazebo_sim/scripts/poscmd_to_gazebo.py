#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import threading

import rospy
from gazebo_msgs.msg import ModelState, ModelStates
from gazebo_msgs.srv import SetModelState
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from quadrotor_msgs.msg import PositionCommand


class PosCmdToGazebo:
    def __init__(self):
        rospy.init_node("poscmd_to_gazebo")

        self.model_name = rospy.get_param("~model_name", "simple_uav")
        self.cmd_topic = rospy.get_param("~cmd_topic", "planning/pos_cmd")
        self.model_states_topic = rospy.get_param("~model_states_topic", "/gazebo/model_states")
        self.set_state_service = rospy.get_param("~set_state_service", "/gazebo/set_model_state")
        self.control_rate = rospy.get_param("~control_rate", 50.0)
        self.cmd_timeout = rospy.get_param("~cmd_timeout", 0.5)

        self.kp_xy = rospy.get_param("~kp_xy", 1.2)
        self.kp_z = rospy.get_param("~kp_z", 1.5)
        self.kp_yaw = rospy.get_param("~kp_yaw", 1.2)
        self.use_yaw_control = rospy.get_param("~use_yaw_control", True)
        self.use_cmd_yaw_dot = rospy.get_param("~use_cmd_yaw_dot", False)
        self.align_heading_before_move = rospy.get_param("~align_heading_before_move", True)
        self.yaw_start_threshold = rospy.get_param("~yaw_start_threshold", 0.18)
        self.yaw_realign_threshold = rospy.get_param("~yaw_realign_threshold", 0.35)
        self.lock_yaw_while_moving = rospy.get_param("~lock_yaw_while_moving", True)
        self.yaw_follow_goal_dist = rospy.get_param("~yaw_follow_goal_dist", 0.30)
        self.ready_to_move = False

        self.max_xy_speed = rospy.get_param("~max_xy_speed", 2.0)
        self.max_z_speed = rospy.get_param("~max_z_speed", 1.2)
        self.max_yaw_rate = rospy.get_param("~max_yaw_rate", 1.2)
        self.z_min = rospy.get_param("~z_min", 0.4)
        self.z_max = rospy.get_param("~z_max", 2.0)
        self.enable_dynamic_avoidance = rospy.get_param("~enable_dynamic_avoidance", True)
        self.obstacle_prefix = rospy.get_param("~obstacle_prefix", "dyn_obs")
        self.slow_distance = rospy.get_param("~slow_distance", 3.0)
        self.safety_distance = rospy.get_param("~safety_distance", 1.5)
        self.repel_gain = rospy.get_param("~repel_gain", 1.2)
        self.max_repel_speed = rospy.get_param("~max_repel_speed", 1.6)
        self.keep_level_attitude = rospy.get_param("~keep_level_attitude", True)

        self.lock = threading.Lock()
        self.latest_cmd = None
        self.latest_cmd_stamp = rospy.Time(0)
        self.latest_pose = None
        self.model_index = None
        self.dynamic_obstacles = []

        rospy.Subscriber(self.cmd_topic, PositionCommand, self.cmd_callback, queue_size=1)
        rospy.Subscriber(self.model_states_topic, ModelStates, self.model_states_callback, queue_size=1)

        rospy.loginfo("waiting for %s ...", self.set_state_service)
        rospy.wait_for_service(self.set_state_service)
        self.set_state = rospy.ServiceProxy(self.set_state_service, SetModelState)

        self.timer = rospy.Timer(
            rospy.Duration(1.0 / max(self.control_rate, 1.0)),
            self.on_timer,
        )
        rospy.loginfo("poscmd_to_gazebo started, model=%s, cmd=%s", self.model_name, self.cmd_topic)

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
            if self.enable_dynamic_avoidance:
                obs = []
                for i, name in enumerate(msg.name):
                    if name.startswith(self.obstacle_prefix) and i < len(msg.pose):
                        p = msg.pose[i].position
                        obs.append((p.x, p.y))
                self.dynamic_obstacles = obs

    def on_timer(self, _):
        with self.lock:
            pose = self.latest_pose
            cmd = self.latest_cmd
            cmd_age = (rospy.Time.now() - self.latest_cmd_stamp).to_sec()
            dynamic_obstacles = list(self.dynamic_obstacles)

        if pose is None:
            return

        vx = 0.0
        vy = 0.0
        vz = 0.0
        wz = 0.0

        if cmd is not None and cmd_age <= self.cmd_timeout:
            des_z = self.clamp(cmd.position.z, self.z_min, self.z_max)
            ex = cmd.position.x - pose.position.x
            ey = cmd.position.y - pose.position.y
            ez = des_z - pose.position.z

            vx = cmd.velocity.x + self.kp_xy * ex
            vy = cmd.velocity.y + self.kp_xy * ey
            vz = cmd.velocity.z + self.kp_z * ez

            yaw_err = 0.0
            if self.use_yaw_control:
                q = pose.orientation
                _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
                # Prefer facing toward the current target position to avoid
                # aggressive spinning caused by discontinuous cmd.yaw.
                if math.hypot(ex, ey) > self.yaw_follow_goal_dist:
                    yaw_target = math.atan2(ey, ex)
                else:
                    yaw_target = cmd.yaw
                yaw_err = self.wrap_pi(yaw_target - yaw)
                wz = self.kp_yaw * yaw_err
                if self.use_cmd_yaw_dot:
                    wz += cmd.yaw_dot
            else:
                wz = 0.0

            # Soft heading-priority behavior: reduce XY speed when heading
            # error is large, but do not force in-place spinning.
            if self.use_yaw_control and self.align_heading_before_move:
                if abs(yaw_err) > self.yaw_realign_threshold:
                    self.ready_to_move = False
                elif abs(yaw_err) < self.yaw_start_threshold:
                    self.ready_to_move = True

                if not self.ready_to_move:
                    # Keep at least a small forward component to avoid
                    # long rotate-in-place behavior at waypoint switches.
                    err = abs(yaw_err)
                    scale = 1.0 - (err / math.pi)
                    scale = self.clamp(scale, 0.20, 0.80)
                    vx *= scale
                    vy *= scale
                elif self.lock_yaw_while_moving:
                    wz = 0.0
        else:
            self.ready_to_move = False

        if pose.position.z > self.z_max:
            vz = min(vz, -0.3)
        elif pose.position.z < self.z_min:
            vz = max(vz, 0.3)

        if self.enable_dynamic_avoidance and dynamic_obstacles:
            vx, vy = self.apply_dynamic_avoidance(vx, vy, pose.position.x, pose.position.y, dynamic_obstacles)

        vx = self.clamp(vx, -self.max_xy_speed, self.max_xy_speed)
        vy = self.clamp(vy, -self.max_xy_speed, self.max_xy_speed)
        vz = self.clamp(vz, -self.max_z_speed, self.max_z_speed)
        wz = self.clamp(wz, -self.max_yaw_rate, self.max_yaw_rate)

        req = ModelState()
        req.model_name = self.model_name
        req.pose = pose
        if self.keep_level_attitude:
            q = req.pose.orientation
            _, _, yaw_now = euler_from_quaternion([q.x, q.y, q.z, q.w])
            qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, yaw_now)
            req.pose.orientation.x = qx
            req.pose.orientation.y = qy
            req.pose.orientation.z = qz
            req.pose.orientation.w = qw
        req.reference_frame = "world"
        req.twist.linear.x = vx
        req.twist.linear.y = vy
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

    @staticmethod
    def wrap_pi(a):
        while a > math.pi:
            a -= 2.0 * math.pi
        while a < -math.pi:
            a += 2.0 * math.pi
        return a

    def apply_dynamic_avoidance(self, vx, vy, x, y, obstacles_xy):
        nearest_dist = None
        nearest_dx = 0.0
        nearest_dy = 0.0

        for ox, oy in obstacles_xy:
            dx = x - ox
            dy = y - oy
            d = math.hypot(dx, dy)
            if nearest_dist is None or d < nearest_dist:
                nearest_dist = d
                nearest_dx = dx
                nearest_dy = dy

        if nearest_dist is None:
            return vx, vy

        if nearest_dist < self.slow_distance:
            # Gradually reduce commanded speed as obstacles approach.
            ratio = max(0.15, min(1.0, (nearest_dist - self.safety_distance) / max(self.slow_distance - self.safety_distance, 1e-3)))
            vx *= ratio
            vy *= ratio

        if nearest_dist < self.safety_distance:
            # Add repulsive velocity to push UAV away from closest dynamic obstacle.
            d = max(nearest_dist, 0.05)
            ux = nearest_dx / d
            uy = nearest_dy / d
            repel = min(self.max_repel_speed, self.repel_gain * (self.safety_distance - nearest_dist + 0.1))
            vx += repel * ux
            vy += repel * uy
            rospy.logwarn_throttle(0.5, "dynamic obstacle too close (%.2fm), applying avoidance", nearest_dist)

        return vx, vy


if __name__ == "__main__":
    try:
        PosCmdToGazebo()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
