#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math

import rospy
from gazebo_msgs.msg import ModelState, ModelStates
from gazebo_msgs.srv import DeleteModel, SetModelState, SpawnModel
from geometry_msgs.msg import Pose


def make_checker_cylinder_sdf(radius, height, mass):
    zc = height * 0.5
    return f"""<?xml version='1.0'?>
<sdf version='1.6'>
  <model name='dynamic_checker_cylinder'>
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
        <pose>0 0 {zc} 0 0 0</pose>
        <geometry>
          <cylinder>
            <radius>{radius}</radius>
            <length>{height}</length>
          </cylinder>
        </geometry>
      </collision>
      <visual name='checker_body'>
        <pose>0 0 {zc} 0 0 0</pose>
        <geometry>
          <cylinder>
            <radius>{radius}</radius>
            <length>{height}</length>
          </cylinder>
        </geometry>
        <material>
          <script>
            <uri>file://media/materials/scripts/gazebo.material</uri>
            <name>Gazebo/Checkerboard</name>
          </script>
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
        self.cyl_radius = float(rospy.get_param("~cyl_radius", 0.30))
        self.cyl_height = float(rospy.get_param("~cyl_height", 1.20))
        self.cyl_mass = float(rospy.get_param("~cyl_mass", 4.0))
        self.safe_clearance = float(rospy.get_param("~safe_clearance", 4.0))
        self.search_radius_max = float(rospy.get_param("~search_radius_max", 15.0))
        self.search_radius_step = float(rospy.get_param("~search_radius_step", 1.5))
        self.angular_samples = int(rospy.get_param("~angular_samples", 24))
        self.min_spawn_distance_to_origin = float(rospy.get_param("~min_spawn_distance_to_origin", 5.0))
        self.auto_relocate = bool(rospy.get_param("~auto_relocate", False))

        # Two dynamic obstacles placed on the current route's key corridors.
        # Motion is set perpendicular to local route direction:
        # 1) near horizontal segment -> vertical crossing motion
        # 2) near vertical segment   -> horizontal crossing motion
        self.obstacles = [
            # Lower area: vertical crossing through y~4 route.
            {"name": f"{self.prefix}_0", "cx": 26.0, "cy": 4.0, "ax": 0.0, "ay": 2.6, "w": 0.15, "ph": 0.0},
            # Right area: horizontal crossing through x~34 route.
            {"name": f"{self.prefix}_1", "cx": 34.0, "cy": 17.0, "ax": 3.8, "ay": 0.0, "w": 0.14, "ph": 1.1},
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
        if self.auto_relocate:
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
        sdf = make_checker_cylinder_sdf(self.cyl_radius, self.cyl_height, self.cyl_mass)
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
                rospy.loginfo("spawned dynamic checker cylinder: %s", name)
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
