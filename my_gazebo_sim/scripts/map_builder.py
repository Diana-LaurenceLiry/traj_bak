#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import numpy as np
import rospy
import tf2_ros
import sensor_msgs.point_cloud2 as pc2

from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid
from tf.transformations import quaternion_matrix


class MapBuilder:
    def __init__(self):
        rospy.init_node("map_builder")

        # topics
        self.cloud_topic = rospy.get_param("~cloud_topic", "/points2")
        self.map_topic = rospy.get_param("~map_topic", "/occupancy_map")
        self.map_frame = rospy.get_param("~map_frame", "odom")

        # map params
        self.resolution = rospy.get_param("~resolution", 0.05)   # m/cell
        self.width = rospy.get_param("~width", 400)              # cells
        self.height = rospy.get_param("~height", 400)            # cells
        self.origin_x = rospy.get_param("~origin_x", -10.0)      # m
        self.origin_y = rospy.get_param("~origin_y", -10.0)      # m

        # filter params
        self.min_range = rospy.get_param("~min_range", 0.2)
        self.max_range = rospy.get_param("~max_range", 8.0)

        # 只保留会挡住小车的点
        self.min_z = rospy.get_param("~min_z", 0.02)
        self.max_z = rospy.get_param("~max_z", 1.0)

        # 可选：基于当前机器人高度做动态切片，避免把地面误当障碍
        self.use_relative_height_filter = rospy.get_param("~use_relative_height_filter", False)
        self.robot_frame = rospy.get_param("~robot_frame", "base_link")
        self.rel_min_z = rospy.get_param("~rel_min_z", -0.10)
        self.rel_max_z = rospy.get_param("~rel_max_z", 0.80)

        # 障碍膨胀半径
        self.inflation_radius = rospy.get_param("~inflation_radius", 0.20)

        # 每帧重建地图，先做最小版
        self.reset_each_cloud = rospy.get_param("~reset_each_cloud", True)

        self.grid = np.zeros((self.height, self.width), dtype=np.int8)

        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.map_pub = rospy.Publisher(self.map_topic, OccupancyGrid, queue_size=1)

        rospy.Subscriber(self.cloud_topic, PointCloud2, self.cloud_callback, queue_size=1)

        rospy.loginfo("map_builder started")
        rospy.loginfo("cloud_topic: %s", self.cloud_topic)
        rospy.loginfo("map_topic: %s", self.map_topic)

    def cloud_callback(self, msg):
        if self.reset_each_cloud:
            self.grid.fill(0)

        try:
            tf_msg = self.tf_buffer.lookup_transform(
                self.map_frame,
                msg.header.frame_id,
                msg.header.stamp,
                rospy.Duration(0.2)
            )
        except Exception as e:
            rospy.logwarn_throttle(1.0, "TF lookup failed: %s", str(e))
            return

        points = self.read_points(msg)
        if points.shape[0] == 0:
            self.publish_map(msg.header.stamp)
            return

        points_map = self.transform_points(points, tf_msg)

        # For relative-height filtering, evaluate points in robot frame to
        # avoid odom drift / map tilt causing ground points to leak through.
        points_robot = None
        if self.use_relative_height_filter:
            try:
                tf_robot = self.tf_buffer.lookup_transform(
                    self.robot_frame,
                    msg.header.frame_id,
                    msg.header.stamp,
                    rospy.Duration(0.2)
                )
                points_robot = self.transform_points(points, tf_robot)
            except Exception as e:
                rospy.logwarn_throttle(1.0, "Robot-frame transform failed: %s", str(e))

        mask = self.filter_mask(points_map, points_robot)
        points_map = points_map[mask]

        self.mark_obstacles(points_map)
        self.inflate_obstacles()
        self.publish_map(msg.header.stamp)

    def read_points(self, cloud_msg):
        pts = []
        for p in pc2.read_points(cloud_msg, field_names=("x", "y", "z"), skip_nans=True):
            x, y, z = p
            pts.append([x, y, z])

        if len(pts) == 0:
            return np.empty((0, 3), dtype=np.float32)

        return np.array(pts, dtype=np.float32)

    def transform_points(self, points, tf_msg):
        tx = tf_msg.transform.translation.x
        ty = tf_msg.transform.translation.y
        tz = tf_msg.transform.translation.z

        qx = tf_msg.transform.rotation.x
        qy = tf_msg.transform.rotation.y
        qz = tf_msg.transform.rotation.z
        qw = tf_msg.transform.rotation.w

        T = quaternion_matrix([qx, qy, qz, qw])
        T[0, 3] = tx
        T[1, 3] = ty
        T[2, 3] = tz

        ones = np.ones((points.shape[0], 1), dtype=np.float32)
        pts_h = np.hstack((points, ones))
        pts_out = (T @ pts_h.T).T[:, :3]
        return pts_out

    def filter_mask(self, points_map, points_robot=None):
        if points_map.shape[0] == 0:
            return np.zeros((0,), dtype=bool)

        # Range is always measured around the robot/sensor, not map origin.
        if points_robot is not None:
            xy_dist = np.linalg.norm(points_robot[:, :2], axis=1)
        else:
            # Fallback: use cloud-local metric if robot-frame transform is unavailable.
            xy_dist = np.linalg.norm(points_map[:, :2], axis=1)
        range_mask = (xy_dist >= self.min_range) & (xy_dist <= self.max_range)

        if self.use_relative_height_filter and points_robot is not None:
            z_mask = (points_robot[:, 2] >= self.rel_min_z) & (points_robot[:, 2] <= self.rel_max_z)
        else:
            z_mask = (points_map[:, 2] >= self.min_z) & (points_map[:, 2] <= self.max_z)

        return range_mask & z_mask

    def world_to_grid(self, x, y):
        gx = int((x - self.origin_x) / self.resolution)
        gy = int((y - self.origin_y) / self.resolution)
        return gx, gy

    def mark_obstacles(self, points):
        for p in points:
            x, y, _ = p
            gx, gy = self.world_to_grid(x, y)
            if 0 <= gx < self.width and 0 <= gy < self.height:
                self.grid[gy, gx] = 100

    def inflate_obstacles(self):
        r = int(math.ceil(self.inflation_radius / self.resolution))
        if r <= 0:
            return

        occ_idx = np.argwhere(self.grid > 0)
        if occ_idx.shape[0] == 0:
            return

        inflated = self.grid.copy()

        for gy, gx in occ_idx:
            for dy in range(-r, r + 1):
                for dx in range(-r, r + 1):
                    ny = gy + dy
                    nx = gx + dx
                    if 0 <= nx < self.width and 0 <= ny < self.height:
                        if dx * dx + dy * dy <= r * r:
                            inflated[ny, nx] = 100

        self.grid = inflated

    def publish_map(self, stamp):
        msg = OccupancyGrid()
        msg.header.stamp = stamp
        msg.header.frame_id = self.map_frame

        msg.info.resolution = self.resolution
        msg.info.width = self.width
        msg.info.height = self.height
        msg.info.origin.position.x = self.origin_x
        msg.info.origin.position.y = self.origin_y
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0

        msg.data = self.grid.flatten().tolist()
        self.map_pub.publish(msg)


if __name__ == "__main__":
    try:
        MapBuilder()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
