#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math

import rospy
from gazebo_msgs.msg import ModelState, ModelStates
from gazebo_msgs.srv import DeleteModel, SetModelState, SpawnModel
from geometry_msgs.msg import Pose


def make_tree_sdf(trunk_radius, trunk_height, canopy_radius, canopy_height, mass):
    canopy_z = trunk_height + canopy_height * 0.45
    collision_height = trunk_height + canopy_height * 0.45
    collision_radius = max(trunk_radius * 1.15, canopy_radius * 0.42)
    return f"""<?xml version='1.0'?>
<sdf version='1.6'>
  <model name='dynamic_tree'>
    <static>false</static>
    <link name='link'>
      <inertial>
        <mass>{mass}</mass>
        <inertia>
          <ixx>1.0</ixx><ixy>0.0</ixy><ixz>0.0</ixz>
          <iyy>1.0</iyy><iyz>0.0</iyz><izz>1.0</izz>
        </inertia>
      </inertial>
      <collision name='collision'>
        <pose>0 0 {collision_height / 2.0} 0 0 0</pose>
        <geometry>
          <cylinder>
            <radius>{collision_radius}</radius>
            <length>{collision_height}</length>
          </cylinder>
        </geometry>
      </collision>
      <visual name='trunk'>
        <pose>0 0 {trunk_height / 2.0} 0 0 0</pose>
        <geometry>
          <cylinder>
            <radius>{trunk_radius}</radius>
            <length>{trunk_height}</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>0.32 0.19 0.08 1</ambient>
          <diffuse>0.36 0.22 0.10 1</diffuse>
        </material>
      </visual>
      <visual name='canopy'>
        <pose>0 0 {canopy_z} 0 0 0</pose>
        <geometry>
          <sphere>
            <radius>{canopy_radius}</radius>
          </sphere>
        </geometry>
        <material>
          <ambient>0.08 0.33 0.10 1</ambient>
          <diffuse>0.10 0.45 0.12 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""


class DynamicObstaclesManager:
    def __init__(self):
        rospy.init_node("dynamic_obstacles_manager")

        self.prefix = rospy.get_param("~prefix", "dyn_obs")
        self.rate_hz = float(rospy.get_param("~rate", 20.0))
        self.z_base = float(rospy.get_param("~z_base", 0.0))
        self.tree_trunk_radius = float(rospy.get_param("~tree_trunk_radius", 0.14))
        self.tree_trunk_height = float(rospy.get_param("~tree_trunk_height", 0.80))
        self.tree_canopy_radius = float(rospy.get_param("~tree_canopy_radius", 0.45))
        self.tree_canopy_height = float(rospy.get_param("~tree_canopy_height", 0.70))
        self.tree_mass = float(rospy.get_param("~tree_mass", 8.0))
        self.safe_clearance = float(rospy.get_param("~safe_clearance", 4.0))
        self.search_radius_max = float(rospy.get_param("~search_radius_max", 15.0))
        self.search_radius_step = float(rospy.get_param("~search_radius_step", 1.5))
        self.angular_samples = int(rospy.get_param("~angular_samples", 24))
        self.min_spawn_distance_to_origin = float(rospy.get_param("~min_spawn_distance_to_origin", 5.0))

        # Corridor-near dynamic obstacles: keep them on/near the default
        # 5-waypoint route while leaving enough surrounding space.
        self.obstacles = [
            # Segment wp1->wp2 (crossing motion in y).
            {"name": f"{self.prefix}_0", "cx": 7.3, "cy": -1.7, "ax": 0.0, "ay": 0.9, "w": 0.18, "ph": 0.0},
            # Segment wp2->wp3 (crossing motion in y).
            {"name": f"{self.prefix}_1", "cx": 10.2, "cy": -2.9, "ax": 0.0, "ay": 0.8, "w": 0.20, "ph": 1.0},
            # Segment wp4->wp5 (crossing motion in x).
            {"name": f"{self.prefix}_2", "cx": 12.6, "cy": -1.3, "ax": 0.8, "ay": 0.0, "w": 0.17, "ph": 2.0},
        ]

        self.model_names = set()
        self.model_xy = {}
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.model_states_cb, queue_size=1)

        rospy.loginfo("waiting for /gazebo/spawn_sdf_model, /gazebo/set_model_state and /gazebo/delete_model ...")
        rospy.wait_for_service("/gazebo/spawn_sdf_model")
        rospy.wait_for_service("/gazebo/set_model_state")
        rospy.wait_for_service("/gazebo/delete_model")
        self.spawn_model = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
        self.set_model_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        self.delete_model = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)

        rospy.sleep(0.5)
        self.delete_legacy_prefixed_models()
        self.adjust_centers_to_avoid_collisions()
        self.spawn_all_once()
        self.start_t = rospy.Time.now().to_sec()
        self.timer = rospy.Timer(rospy.Duration(1.0 / max(self.rate_hz, 1.0)), self.on_timer)
        rospy.loginfo("dynamic_obstacles_manager started with %d obstacles", len(self.obstacles))

    def model_states_cb(self, msg):
        self.model_names = set(msg.name)
        self.model_xy = {}
        for i, name in enumerate(msg.name):
            if i < len(msg.pose):
                p = msg.pose[i].position
                self.model_xy[name] = (p.x, p.y)

    def delete_legacy_prefixed_models(self):
        to_delete = [name for name in self.model_names if name.startswith(self.prefix + "_")]
        for name in to_delete:
            try:
                self.delete_model(name)
                rospy.loginfo("deleted legacy dynamic model: %s", name)
            except rospy.ServiceException as e:
                rospy.logwarn("delete_model failed for %s: %s", name, str(e))

    def nearest_external_distance(self, x, y):
        nearest = float("inf")
        for name, (mx, my) in self.model_xy.items():
            if name.startswith(self.prefix + "_"):
                continue
            if name in ("ground_plane", "sun"):
                continue
            d = math.hypot(mx - x, my - y)
            if d < nearest:
                nearest = d
        return nearest

    def adjust_centers_to_avoid_collisions(self):
        selected = []
        for cfg in self.obstacles:
            best_x = cfg["cx"]
            best_y = cfg["cy"]
            best_d = self.nearest_external_distance(best_x, best_y)

            # Spiral-search around candidate center to avoid static scene objects.
            r = 0.0
            while r <= self.search_radius_max:
                found = False
                for k in range(max(self.angular_samples, 8)):
                    ang = 2.0 * math.pi * k / float(max(self.angular_samples, 8))
                    x = cfg["cx"] + r * math.cos(ang)
                    y = cfg["cy"] + r * math.sin(ang)
                    if math.hypot(x, y) < self.min_spawn_distance_to_origin:
                        continue
                    if any(math.hypot(x - sx, y - sy) < self.safe_clearance * 0.8 for sx, sy in selected):
                        continue
                    d = self.nearest_external_distance(x, y)
                    if d > best_d:
                        best_d = d
                        best_x = x
                        best_y = y
                    if d >= self.safe_clearance:
                        best_x = x
                        best_y = y
                        found = True
                        break
                if found:
                    break
                r += self.search_radius_step

            cfg["cx"] = best_x
            cfg["cy"] = best_y
            selected.append((best_x, best_y))
            rospy.loginfo("dynamic tree %s center set to (%.2f, %.2f), nearest obstacle %.2fm",
                          cfg["name"], best_x, best_y, best_d)

    def spawn_all_once(self):
        sdf = make_tree_sdf(
            self.tree_trunk_radius,
            self.tree_trunk_height,
            self.tree_canopy_radius,
            self.tree_canopy_height,
            self.tree_mass,
        )
        for cfg in self.obstacles:
            name = cfg["name"]
            if name in self.model_names:
                continue
            pose = Pose()
            pose.position.x = cfg["cx"]
            pose.position.y = cfg["cy"]
            pose.position.z = self.z_base
            pose.orientation.w = 1.0
            try:
                self.spawn_model(name, sdf, "", pose, "world")
                rospy.loginfo("spawned dynamic moving tree: %s", name)
            except rospy.ServiceException as e:
                rospy.logwarn("spawn_sdf_model failed for %s: %s", name, str(e))

    def on_timer(self, _):
        t = rospy.Time.now().to_sec() - self.start_t
        for cfg in self.obstacles:
            x = cfg["cx"] + cfg["ax"] * math.sin(cfg["w"] * t + cfg["ph"])
            y = cfg["cy"] + cfg["ay"] * math.sin(cfg["w"] * t + cfg["ph"])
            vx = cfg["ax"] * cfg["w"] * math.cos(cfg["w"] * t + cfg["ph"])
            vy = cfg["ay"] * cfg["w"] * math.cos(cfg["w"] * t + cfg["ph"])
            yaw = math.atan2(vy, vx) if (abs(vx) + abs(vy)) > 1e-3 else 0.0

            state = ModelState()
            state.model_name = cfg["name"]
            state.reference_frame = "world"
            state.pose.position.x = x
            state.pose.position.y = y
            state.pose.position.z = self.z_base
            state.pose.orientation.z = math.sin(0.5 * yaw)
            state.pose.orientation.w = math.cos(0.5 * yaw)
            state.twist.linear.x = vx
            state.twist.linear.y = vy
            state.twist.linear.z = 0.0
            state.twist.angular.x = 0.0
            state.twist.angular.y = 0.0
            state.twist.angular.z = 0.0

            try:
                self.set_model_state(state)
            except rospy.ServiceException as e:
                rospy.logwarn_throttle(1.0, "set_model_state failed for %s: %s", cfg["name"], str(e))


if __name__ == "__main__":
    try:
        DynamicObstaclesManager()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
