#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand


class AutoGoalPatrol:
    @staticmethod
    def as_bool(v):
        if isinstance(v, bool):
            return v
        if isinstance(v, (int, float)):
            return v != 0
        if isinstance(v, str):
            return v.strip().lower() in ("1", "true", "yes", "on")
        return bool(v)

    def __init__(self):
        rospy.init_node("auto_goal_patrol")

        self.goal_topic = rospy.get_param("~goal_topic", "/move_base_simple/goal")
        self.odom_topic = rospy.get_param("~odom_topic", "/visual_slam/odom_safe_filtered")
        self.cmd_topic = rospy.get_param("~cmd_topic", "planning/pos_cmd")
        self.frame_id = rospy.get_param("~frame_id", "world")
        self.switch_distance = float(rospy.get_param("~switch_distance", 0.9))
        self.switch_cooldown = float(rospy.get_param("~switch_cooldown", 0.6))
        self.resend_same_goal = self.as_bool(rospy.get_param("~resend_same_goal", False))
        self.resend_interval = float(rospy.get_param("~resend_interval", 1.0))
        self.start_delay = float(rospy.get_param("~start_delay", 2.0))
        self.start_min_distance = float(rospy.get_param("~start_min_distance", 2.0))
        self.goal_timeout = float(rospy.get_param("~goal_timeout", 10.0))
        self.no_progress_timeout = float(rospy.get_param("~no_progress_timeout", self.goal_timeout))
        self.progress_epsilon = float(rospy.get_param("~progress_epsilon", 0.15))
        self.timeout_min_age = float(rospy.get_param("~timeout_min_age", 4.0))
        self.enable_no_progress_timeout = self.as_bool(rospy.get_param("~enable_no_progress_timeout", True))
        self.cmd_stale_retrigger_sec = float(rospy.get_param("~cmd_stale_retrigger_sec", 1.2))
        self.cmd_retrigger_interval = float(rospy.get_param("~cmd_retrigger_interval", 1.5))
        self.loop_route = self.as_bool(rospy.get_param("~loop_route", True))

        # Route intentionally crosses dynamic obstacle lanes.
        default_route = [
            [7.5, -2.0, 1.0],
            [14.0, -2.0, 1.0],
            [14.0, 6.0, 1.0],
            [20.0, -8.0, 1.0],
            [24.0, -2.0, 1.0],
            [14.0, 2.0, 1.0],
            [7.5, -2.0, 1.0],
        ]
        raw_route = rospy.get_param("~route_points", default_route)
        self.route = self._normalize_route(raw_route)
        if not self.route:
            rospy.logerr("auto_goal_patrol: route_points is empty, shutdown")
            rospy.signal_shutdown("empty route")
            return

        self.current_index = 0
        self.current_goal = None
        self.last_goal_pub_t = 0.0
        self.goal_set_t = 0.0
        self.last_published_index = -1
        self.last_switch_t = 0.0
        self.latest_pos = None
        self.best_dxy = None
        self.last_progress_t = 0.0
        self.last_cmd_t = 0.0
        self.last_cmd_retrigger_t = 0.0

        self.goal_pub = rospy.Publisher(self.goal_topic, PoseStamped, queue_size=1, latch=True)
        rospy.Subscriber(self.odom_topic, Odometry, self.odom_cb, queue_size=1)
        rospy.Subscriber(self.cmd_topic, PositionCommand, self.cmd_cb, queue_size=1)
        rospy.loginfo("auto_goal_patrol odom_topic=%s goal_topic=%s cmd_topic=%s", self.odom_topic, self.goal_topic, self.cmd_topic)

        rospy.sleep(max(0.0, self.start_delay))
        self.wait_initial_odom(timeout_sec=3.0)
        self.skip_nearby_waypoints(min_xy_dist=self.start_min_distance)
        self.publish_current_goal(force=True)
        self.timer = rospy.Timer(rospy.Duration(0.1), self.on_timer)
        rospy.loginfo("auto_goal_patrol started, %d waypoints, loop=%s", len(self.route), str(self.loop_route))

    def _normalize_route(self, raw_route):
        route = []
        for p in raw_route:
            if not isinstance(p, (list, tuple)) or len(p) < 2:
                continue
            x = float(p[0])
            y = float(p[1])
            z = float(p[2]) if len(p) >= 3 else 1.0
            route.append((x, y, z))
        return route

    def odom_cb(self, msg):
        self.latest_pos = msg.pose.pose.position

    def cmd_cb(self, _msg):
        self.last_cmd_t = rospy.Time.now().to_sec()

    def wait_initial_odom(self, timeout_sec=3.0):
        end_t = rospy.Time.now().to_sec() + max(0.1, timeout_sec)
        r = rospy.Rate(20)
        while not rospy.is_shutdown() and self.latest_pos is None and rospy.Time.now().to_sec() < end_t:
            r.sleep()
        if self.latest_pos is None:
            rospy.logwarn("auto_goal_patrol: no odom before start, continue with default first waypoint")

    def xy_dist_to_waypoint(self, idx):
        if self.latest_pos is None:
            return None
        x, y, _ = self.route[idx]
        return math.hypot(self.latest_pos.x - x, self.latest_pos.y - y)

    def skip_nearby_waypoints(self, min_xy_dist):
        if self.latest_pos is None:
            return
        for _ in range(len(self.route)):
            d = self.xy_dist_to_waypoint(self.current_index)
            if d is None or d >= min_xy_dist:
                return
            self.current_index = (self.current_index + 1) % len(self.route)
        rospy.logwarn("auto_goal_patrol: all waypoints are near current position, keep index=%d", self.current_index)

    
    def publish_current_goal(self, force=False):
        now = rospy.Time.now().to_sec()
        # Always publish immediately when waypoint index changes.
        index_changed = (self.current_index != self.last_published_index)
        if not force and not index_changed:
            if not self.resend_same_goal:
                return
            if now - self.last_goal_pub_t < self.resend_interval:
                return
        x, y, z = self.route[self.current_index]
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame_id
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.w = 1.0
        self.goal_pub.publish(msg)
        self.current_goal = (x, y, z)
        self.last_goal_pub_t = now
        # Only reset timeout baseline when waypoint index actually changes.
        if index_changed:
            self.goal_set_t = now
            self.best_dxy = None
            self.last_progress_t = now
        self.last_published_index = self.current_index
        rospy.loginfo("auto_goal_patrol -> wp[%d/%d]: (%.2f, %.2f, %.2f)",
                      self.current_index + 1, len(self.route), x, y, z)
    

    def on_timer(self, _):
        if self.current_goal is None:
            self.publish_current_goal(force=True)
            return

        # If current goal is too close (e.g. restart near old waypoint), skip it.
        prev_idx = self.current_index
        self.skip_nearby_waypoints(min_xy_dist=max(self.switch_distance * 1.2, 0.8))
        if self.current_index != prev_idx:
            self.publish_current_goal(force=True)
        self.publish_current_goal(force=False)
        if self.latest_pos is None:
            rospy.logwarn_throttle(2.0, "auto_goal_patrol: waiting odom on %s", self.odom_topic)
            return

        gx, gy, gz = self.current_goal
        dx = self.latest_pos.x - gx
        dy = self.latest_pos.y - gy
        dxy = math.hypot(dx, dy)
        now = rospy.Time.now().to_sec()
        if self.best_dxy is None:
            self.best_dxy = dxy
            self.last_progress_t = now
        elif dxy < self.best_dxy - self.progress_epsilon:
            self.best_dxy = dxy
            self.last_progress_t = now

        no_progress = (now - self.last_progress_t) if self.last_progress_t > 0.0 else 0.0
        rospy.loginfo_throttle(
            1.0,
            "auto_goal_patrol status: wp=%d dxy=%.3f best=%.3f no_prog=%.1fs pos=(%.2f,%.2f) goal=(%.2f,%.2f)",
            self.current_index + 1, dxy, self.best_dxy, no_progress, self.latest_pos.x, self.latest_pos.y, gx, gy
        )

        # If planner command stream is stale, retrigger current goal to pull FSM out of WAIT_TARGET.
        cmd_gap = now - self.last_cmd_t if self.last_cmd_t > 0.0 else 1e9
        if (
            cmd_gap > self.cmd_stale_retrigger_sec
            and (now - self.last_cmd_retrigger_t) > self.cmd_retrigger_interval
            and dxy > self.switch_distance
        ):
            self.last_cmd_retrigger_t = now
            rospy.logwarn(
                "auto_goal_patrol: pos_cmd stale %.2fs, retrigger current goal wp[%d]",
                cmd_gap,
                self.current_index + 1,
            )
            self.publish_current_goal(force=True)

        # Pure multi-waypoint mode: switch by XY distance threshold.
        if dxy <= self.switch_distance and (now - self.last_switch_t) >= self.switch_cooldown:
            self.last_switch_t = now
            self.advance_waypoint()
            return

        # Timeout fallback: skip only if there has been no progress for a while.
        if (
            self.enable_no_progress_timeout
            and self.goal_set_t > 0.0
            and (now - self.goal_set_t) >= self.timeout_min_age
            and no_progress > self.no_progress_timeout
        ):
            rospy.logwarn(
                "auto_goal_patrol: no-progress timeout, skip wp[%d] dxy=%.3f best=%.3f no_prog=%.1fs",
                self.current_index + 1,
                dxy,
                self.best_dxy if self.best_dxy is not None else dxy,
                no_progress,
            )
            self.last_switch_t = now
            self.advance_waypoint()

    def advance_waypoint(self):
        if self.current_index + 1 < len(self.route):
            self.current_index += 1
        elif self.loop_route:
            self.current_index = 0
        else:
            rospy.loginfo("auto_goal_patrol finished route")
            rospy.signal_shutdown("route completed")
            return
        rospy.loginfo("auto_goal_patrol: switch to wp[%d/%d]", self.current_index + 1, len(self.route))
        self.publish_current_goal(force=True)


if __name__ == "__main__":
    try:
        AutoGoalPatrol()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
