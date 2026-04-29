#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import glob
import os
import re
import signal
import subprocess
import sys
import time
from datetime import datetime

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from rosgraph_msgs.msg import Log


def now_str():
    return datetime.now().strftime("%Y-%m-%d %H:%M:%S")


def dist3(a, b):
    return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2) ** 0.5


def dist2(a, b):
    return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5


class GoalSwitchWatcher:
    """Detect transition: target goal -> next non-target goal."""

    def __init__(self, topic, target_xyz, tol, use_xy_only):
        self.topic = topic
        self.target = target_xyz
        self.tol = tol
        self.use_xy_only = use_xy_only
        self.seen_target = False
        self.switched = False
        self.last_goal = None
        self.last_stamp = 0.0
        self.sub = rospy.Subscriber(self.topic, PoseStamped, self._cb, queue_size=100)

    def _cb(self, msg):
        p = msg.pose.position
        g = (float(p.x), float(p.y), float(p.z))
        self.last_goal = g
        self.last_stamp = time.time()
        d = dist2(g, self.target) if self.use_xy_only else dist3(g, self.target)
        is_target = d <= self.tol
        if is_target:
            self.seen_target = True
            return
        if self.seen_target:
            self.switched = True


class OdomStallWatcher:
    """Detect stale motion from odom stream (stuck condition)."""

    def __init__(self, topic, move_eps_xy, move_eps_z):
        self.topic = topic
        self.move_eps_xy = move_eps_xy
        self.move_eps_z = move_eps_z
        self.last_pos = None
        self.last_odom_t = 0.0
        now = time.time()
        self.start_t = now
        self.last_move_t = now
        self.sub = rospy.Subscriber(self.topic, Odometry, self._cb, queue_size=50)

    def _cb(self, msg):
        p = msg.pose.pose.position
        cur = (float(p.x), float(p.y), float(p.z))
        self.last_odom_t = time.time()
        if self.last_pos is None:
            self.last_pos = cur
            self.last_move_t = self.last_odom_t
            return

        dx = cur[0] - self.last_pos[0]
        dy = cur[1] - self.last_pos[1]
        dz = abs(cur[2] - self.last_pos[2])
        dxy = (dx * dx + dy * dy) ** 0.5
        if dxy >= self.move_eps_xy or dz >= self.move_eps_z:
            self.last_move_t = self.last_odom_t
            self.last_pos = cur


class WaypointIndexSwitchWatcher:
    """Detect transition: seen wp[target] -> next wp[other]."""

    def __init__(self, rosout_topic, target_wp_idx):
        self.rosout_topic = rosout_topic
        self.target_wp_idx = int(target_wp_idx)
        self.seen_target = False
        self.switched = False
        self.last_wp_idx = None
        self.last_stamp = 0.0
        self._wp_re = re.compile(r"(?:switch to wp|-> wp)\[(\d+)/(\d+)\]")
        self.sub = rospy.Subscriber(self.rosout_topic, Log, self._cb, queue_size=200)

    def _cb(self, msg):
        text = msg.msg
        m = self._wp_re.search(text)
        if not m:
            return
        idx = int(m.group(1))
        self.last_wp_idx = idx
        self.last_stamp = time.time()
        if idx == self.target_wp_idx:
            self.seen_target = True
            return
        if self.seen_target and idx != self.target_wp_idx:
            self.switched = True


def start_proc(cmd, log_path):
    log_f = open(log_path, "w")
    proc = subprocess.Popen(
        cmd,
        stdout=log_f,
        stderr=subprocess.STDOUT,
        preexec_fn=os.setsid,
        text=True,
    )
    return proc, log_f


def stop_proc(proc, log_f, name, soft_sec=6.0):
    if proc is None:
        return
    try:
        if proc.poll() is None:
            os.killpg(proc.pid, signal.SIGINT)
            t0 = time.time()
            while proc.poll() is None and (time.time() - t0) < soft_sec:
                time.sleep(0.2)
        if proc.poll() is None:
            os.killpg(proc.pid, signal.SIGTERM)
            t0 = time.time()
            while proc.poll() is None and (time.time() - t0) < 4.0:
                time.sleep(0.2)
        if proc.poll() is None:
            os.killpg(proc.pid, signal.SIGKILL)
    except Exception:
        pass
    try:
        log_f.close()
    except Exception:
        pass


def latest_readable(out_dir, run_name):
    patt = os.path.join(out_dir, f"{run_name}_*_readable.txt")
    cands = sorted(glob.glob(patt), key=os.path.getmtime, reverse=True)
    return cands[0] if cands else ""


def _pgrep_alive(args):
    r = subprocess.run(args, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    return r.returncode == 0


def any_leftovers_alive(launch_pkg, launch_file, bag_dir, prefix):
    return (
        _pgrep_alive(["pgrep", "-f", f"roslaunch {launch_pkg} {launch_file}"])
        or _pgrep_alive(["pgrep", "-f", f"{bag_dir}/{prefix}_run.*\\.bag"])
        or _pgrep_alive(["pgrep", "-x", "gzserver"])
        or _pgrep_alive(["pgrep", "-x", "gzclient"])
        or _pgrep_alive(["pgrep", "-x", "gazebo"])
    )


def cleanup_leftovers(launch_pkg, launch_file, bag_dir, prefix, wait_sec=25):
    # Graceful stop first
    for cmd in [
        ["pkill", "-INT", "-f", f"roslaunch {launch_pkg} {launch_file}"],
        ["pkill", "-INT", "-f", f"{bag_dir}/{prefix}_run.*\\.bag"],
        ["pkill", "-INT", "-x", "gzserver"],
        ["pkill", "-INT", "-x", "gzclient"],
        ["pkill", "-INT", "-x", "gazebo"],
    ]:
        subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    time.sleep(1.0)

    # Escalate if still alive
    if any_leftovers_alive(launch_pkg, launch_file, bag_dir, prefix):
        for cmd in [
            ["pkill", "-TERM", "-f", f"roslaunch {launch_pkg} {launch_file}"],
            ["pkill", "-TERM", "-f", f"{bag_dir}/{prefix}_run.*\\.bag"],
            ["pkill", "-TERM", "-x", "gzserver"],
            ["pkill", "-TERM", "-x", "gzclient"],
            ["pkill", "-TERM", "-x", "gazebo"],
        ]:
            subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    deadline = time.time() + max(5.0, wait_sec)
    while any_leftovers_alive(launch_pkg, launch_file, bag_dir, prefix):
        if time.time() >= deadline:
            for cmd in [
                ["pkill", "-KILL", "-f", f"roslaunch {launch_pkg} {launch_file}"],
                ["pkill", "-KILL", "-f", f"{bag_dir}/{prefix}_run.*\\.bag"],
                ["pkill", "-KILL", "-x", "gzserver"],
                ["pkill", "-KILL", "-x", "gzclient"],
                ["pkill", "-KILL", "-x", "gazebo"],
            ]:
                subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            break
        time.sleep(1.0)


def main():
    parser = argparse.ArgumentParser(
        description="Repeat UAV experiment N times and stop each run when goal switches from target waypoint to next waypoint."
    )
    parser.add_argument("--runs", type=int, default=10)
    parser.add_argument("--start_idx", type=int, default=1, help="Starting run index (e.g., 5 -> run05)")
    parser.add_argument("--prefix", default="g2_gl_batch")
    parser.add_argument("--bag_dir", default="/home/lry/catkin_ws/exp_bags")
    parser.add_argument("--out_dir", default="/home/lry/catkin_ws/logs/bag_metrics")
    parser.add_argument("--launch_pkg", default="my_gazebo_sim")
    parser.add_argument("--launch_file", default="ego_orb_gazebo_bridge.launch")
    parser.add_argument("--launch_args", default="use_auto_patrol:=true")
    parser.add_argument("--goal_topic", default="/move_base_simple/goal")
    parser.add_argument("--rosout_topic", default="/rosout_agg")
    parser.add_argument("--stop_wp_idx", type=int, default=7,
                        help="Stop each run when route switches from wp[this] to another wp")
    parser.add_argument("--target_x", type=float, default=25.0)
    parser.add_argument("--target_y", type=float, default=30.5)
    parser.add_argument("--target_z", type=float, default=1.0)
    parser.add_argument("--target_tol", type=float, default=0.35)
    parser.add_argument(
        "--target_xy_only",
        action="store_true",
        default=True,
        help="Use XY distance only for target waypoint match (default: on).",
    )
    parser.add_argument(
        "--target_xyz",
        action="store_true",
        help="Use XYZ distance for target waypoint match (override --target_xy_only).",
    )
    parser.add_argument("--odom_topic", default="/visual_slam/odom_safe_filtered")
    parser.add_argument("--stall_sec", type=float, default=20.0,
                        help="Stop run if odom movement is stale for this many seconds")
    parser.add_argument("--stall_min_age", type=float, default=15.0,
                        help="Ignore stall detection during initial startup period")
    parser.add_argument("--move_eps_xy", type=float, default=0.12,
                        help="Minimum XY movement (m) to be considered progress")
    parser.add_argument("--move_eps_z", type=float, default=0.08,
                        help="Minimum Z movement (m) to be considered progress")
    parser.add_argument("--startup_sec", type=float, default=10.0)
    parser.add_argument("--tail_sec", type=float, default=2.0)
    parser.add_argument("--timeout_sec", type=float, default=300.0)
    parser.add_argument("--cleanup_wait_sec", type=float, default=25.0)
    parser.add_argument("--cooldown_sec", type=float, default=2.0)
    parser.add_argument("--method", default="global_local")
    parser.add_argument("--metrics_script", default="/home/lry/catkin_ws/src/my_gazebo_sim/scripts/bag_experiment_metrics.py")
    args = parser.parse_args()
    if args.target_xyz:
        args.target_xy_only = False

    os.makedirs(args.bag_dir, exist_ok=True)
    os.makedirs(args.out_dir, exist_ok=True)

    summary_path = os.path.join(args.out_dir, f"batch_goal_switch_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt")
    with open(summary_path, "w", encoding="utf-8") as sf:
        sf.write(f"Batch start: {now_str()}\n")
        sf.write(f"runs={args.runs}, launch={args.launch_pkg}/{args.launch_file} {args.launch_args}\n")
        mode = "XY" if args.target_xy_only else "XYZ"
        sf.write(
            f"stop condition: goal switches from [{args.target_x},{args.target_y},{args.target_z}] to next point "
            f"(match mode={mode}, tol={args.target_tol})\n\n"
        )
        sf.write(f"wp-index condition: switch away after wp[{args.stop_wp_idx}]\n\n")

    # Ignore outer CLI args (especially strings containing ':=') to avoid
    # rospy treating them as remap params and throwing ROSInitException.
    rospy.init_node(
        "goal_switch_batch_runner",
        anonymous=True,
        disable_signals=True,
        argv=[sys.argv[0]],
    )

    for i in range(args.start_idx, args.start_idx + args.runs):
        run_name = f"{args.prefix}_run{i:02d}"
        bag_path = os.path.join(args.bag_dir, f"{run_name}.bag")
        launch_log = os.path.join(args.out_dir, f"{run_name}_launch.log")
        bag_log = os.path.join(args.out_dir, f"{run_name}_rosbag.log")
        parse_log = os.path.join(args.out_dir, f"{run_name}_parse.log")

        k = i - args.start_idx + 1
        print(f"[{now_str()}] [{k}/{args.runs}] start {run_name}")
        cleanup_leftovers(args.launch_pkg, args.launch_file, args.bag_dir, args.prefix, args.cleanup_wait_sec)
        target = (args.target_x, args.target_y, args.target_z)
        watcher = GoalSwitchWatcher(args.goal_topic, target, args.target_tol, args.target_xy_only)
        wp_switch_watcher = WaypointIndexSwitchWatcher(args.rosout_topic, args.stop_wp_idx)
        stall_watcher = OdomStallWatcher(args.odom_topic, args.move_eps_xy, args.move_eps_z)

        launch_cmd = [
            "bash",
            "-lc",
            (
                "source /opt/ros/noetic/setup.bash && "
                "source /home/lry/catkin_ws/devel/setup.bash && "
                f"roslaunch {args.launch_pkg} {args.launch_file} {args.launch_args}"
            ),
        ]
        launch_proc, launch_log_f = start_proc(launch_cmd, launch_log)

        time.sleep(max(1.0, args.startup_sec))

        bag_cmd = [
            "bash",
            "-lc",
            (
                "source /opt/ros/noetic/setup.bash && "
                "source /home/lry/catkin_ws/devel/setup.bash && "
                f"rosbag record --lz4 -O '{bag_path}' "
                "/clock /move_base_simple/goal /planner/ego_goal /planning/pos_cmd "
                "/planning/bspline /visual_slam/odom_safe_filtered /gazebo/model_states /rosout_agg"
            ),
        ]
        bag_proc, bag_log_f = start_proc(bag_cmd, bag_log)

        t0 = time.time()
        stop_reason = "timeout"
        while True:
            if wp_switch_watcher.switched:
                stop_reason = f"wp{args.stop_wp_idx}_switched_to_next"
                break
            if watcher.switched:
                stop_reason = "goal_switched_after_target_coord"
                break
            now_t = time.time()
            if (
                args.stall_sec > 0.0
                and (now_t - stall_watcher.start_t) >= args.stall_min_age
                and (now_t - stall_watcher.last_move_t) >= args.stall_sec
            ):
                stop_reason = f"stall_no_motion_{int(args.stall_sec)}s"
                break
            if launch_proc.poll() is not None:
                stop_reason = "launch_exited"
                break
            if (time.time() - t0) >= args.timeout_sec:
                stop_reason = "timeout"
                break
            time.sleep(0.2)

        time.sleep(max(0.0, args.tail_sec))
        stop_proc(bag_proc, bag_log_f, "rosbag")
        stop_proc(launch_proc, launch_log_f, "roslaunch")
        cleanup_leftovers(args.launch_pkg, args.launch_file, args.bag_dir, args.prefix, args.cleanup_wait_sec)

        parse_cmd = [
            "bash",
            "-lc",
            (
                "source /opt/ros/noetic/setup.bash && "
                "source /home/lry/catkin_ws/devel/setup.bash && "
                f"/usr/bin/python3 '{args.metrics_script}' --bag '{bag_path}' "
                f"--method '{args.method}' --output_dir '{args.out_dir}'"
            ),
        ]
        with open(parse_log, "w") as pf:
            subprocess.run(parse_cmd, stdout=pf, stderr=subprocess.STDOUT, text=True)

        readable = latest_readable(args.out_dir, run_name)
        brief = "readable missing"
        if readable and os.path.isfile(readable):
            try:
                with open(readable, "r", encoding="utf-8") as rf:
                    brief = rf.readline().strip()
            except Exception:
                pass

        with open(summary_path, "a", encoding="utf-8") as sf:
            sf.write(f"{run_name}\n")
            sf.write(f"  stop_reason: {stop_reason}\n")
            sf.write(f"  bag: {bag_path}\n")
            sf.write(f"  readable: {readable}\n")
            sf.write(f"  brief: {brief}\n\n")

        print(f"[{now_str()}] [{k}/{args.runs}] done {run_name} ({stop_reason})")
        time.sleep(max(0.0, args.cooldown_sec))

    with open(summary_path, "a", encoding="utf-8") as sf:
        sf.write(f"Batch end: {now_str()}\n")

    print(f"\nDone. summary: {summary_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
