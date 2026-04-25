#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import csv
import os
from datetime import datetime

import matplotlib

# Use non-interactive backend so plotting works even without X forwarding.
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import rospy
from nav_msgs.msg import Odometry


class OdomXYZLoggerPlot:
    def __init__(self):
        rospy.init_node("odom_xyz_logger_plot")

        self.odom_topic = rospy.get_param("~odom_topic", "/visual_slam/odom_aligned")
        self.output_dir = rospy.get_param("~output_dir", os.path.expanduser("~/catkin_ws/logs"))
        self.save_prefix = rospy.get_param("~save_prefix", "odom_xyz")
        self.min_samples_to_plot = int(rospy.get_param("~min_samples_to_plot", 2))
        self.drop_non_monotonic_time = bool(rospy.get_param("~drop_non_monotonic_time", True))

        os.makedirs(self.output_dir, exist_ok=True)

        self.t_list = []
        self.x_list = []
        self.y_list = []
        self.z_list = []
        self.t0 = None
        self.last_t = None
        self.drop_count = 0

        rospy.Subscriber(self.odom_topic, Odometry, self.odom_cb, queue_size=200)
        rospy.on_shutdown(self.on_shutdown)
        rospy.loginfo("odom_xyz_logger_plot started, topic=%s output_dir=%s", self.odom_topic, self.output_dir)

    def odom_cb(self, msg):
        t = msg.header.stamp.to_sec()
        if t <= 0.0:
            t = rospy.Time.now().to_sec()
        if self.t0 is None:
            self.t0 = t

        t_rel = t - self.t0
        if self.drop_non_monotonic_time and self.last_t is not None and t_rel <= self.last_t:
            self.drop_count += 1
            rospy.logwarn_throttle(
                2.0,
                "odom_xyz_logger_plot: dropped non-monotonic stamp sample (%d dropped)",
                self.drop_count,
            )
            return

        self.last_t = t_rel
        self.t_list.append(t_rel)
        self.x_list.append(msg.pose.pose.position.x)
        self.y_list.append(msg.pose.pose.position.y)
        self.z_list.append(msg.pose.pose.position.z)

    def on_shutdown(self):
        n = len(self.t_list)
        if n < 1:
            rospy.logwarn("odom_xyz_logger_plot: no samples recorded, skip save/plot")
            return

        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        csv_path = os.path.join(self.output_dir, f"{self.save_prefix}_{stamp}.csv")
        png_path = os.path.join(self.output_dir, f"{self.save_prefix}_{stamp}.png")

        self.save_csv(csv_path)
        if n >= self.min_samples_to_plot:
            self.save_plot(png_path)
            rospy.loginfo(
                "odom_xyz_logger_plot: saved %d samples (%d dropped non-monotonic)\ncsv: %s\npng: %s",
                n, self.drop_count, csv_path, png_path
            )
        else:
            rospy.loginfo("odom_xyz_logger_plot: saved %d samples (too few for plot)\ncsv: %s", n, csv_path)

    def save_csv(self, path):
        with open(path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["t_sec", "x", "y", "z"])
            for i in range(len(self.t_list)):
                writer.writerow([self.t_list[i], self.x_list[i], self.y_list[i], self.z_list[i]])

    def save_plot(self, path):
        fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)

        axes[0].plot(self.t_list, self.x_list, "r-", linewidth=1.2)
        axes[0].set_ylabel("x (m)")
        axes[0].grid(True, linestyle="--", alpha=0.4)

        axes[1].plot(self.t_list, self.y_list, "g-", linewidth=1.2)
        axes[1].set_ylabel("y (m)")
        axes[1].grid(True, linestyle="--", alpha=0.4)

        axes[2].plot(self.t_list, self.z_list, "b-", linewidth=1.2)
        axes[2].set_ylabel("z (m)")
        axes[2].set_xlabel("time (s)")
        axes[2].grid(True, linestyle="--", alpha=0.4)

        fig.suptitle("Odometry XYZ vs Time")
        fig.tight_layout()
        fig.savefig(path, dpi=150)
        plt.close(fig)


if __name__ == "__main__":
    try:
        OdomXYZLoggerPlot()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
