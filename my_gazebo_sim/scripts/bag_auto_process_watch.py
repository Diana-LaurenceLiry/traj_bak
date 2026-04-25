#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import json
import os
import subprocess
import time
from datetime import datetime


def now():
    return datetime.now().strftime("%Y-%m-%d %H:%M:%S")


def load_state(path):
    if not os.path.exists(path):
        return {"processed": {}}
    try:
        with open(path, "r") as f:
            data = json.load(f)
            if "processed" not in data:
                data["processed"] = {}
            return data
    except Exception:
        return {"processed": {}}


def save_state(path, state):
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        json.dump(state, f, indent=2, ensure_ascii=False, sort_keys=True)
    os.replace(tmp, path)


def list_bags(bag_dir):
    bags = []
    for name in os.listdir(bag_dir):
        if name.endswith(".bag"):
            bags.append(os.path.join(bag_dir, name))
    bags.sort()
    return bags


def is_stable_file(path, settle_sec):
    if not os.path.exists(path):
        return False
    try:
        s1 = os.path.getsize(path)
        t1 = os.path.getmtime(path)
        time.sleep(1.0)
        s2 = os.path.getsize(path)
        t2 = os.path.getmtime(path)
        if s1 != s2 or t1 != t2:
            return False
        # Ensure writer has been idle for a bit.
        return (time.time() - t2) >= settle_sec
    except OSError:
        return False


def run_processor(py, processor, bag_path, output_dir, method, append_csv, append_bag_info_csv, world_file):
    cmd = [
        py,
        processor,
        "--bag",
        bag_path,
        "--method",
        method,
        "--output_dir",
        output_dir,
    ]
    if append_csv:
        cmd += ["--append_csv", append_csv]
    if append_bag_info_csv:
        cmd += ["--append_bag_info_csv", append_bag_info_csv]
    if world_file:
        cmd += ["--world_file", world_file]

    print(f"[{now()}] Processing bag: {bag_path}")
    print(f"[{now()}] Command: {' '.join(cmd)}")
    proc = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    print(proc.stdout.strip())
    return proc.returncode


def main():
    parser = argparse.ArgumentParser(description="Watch a bag folder and auto-run metrics script for new bags.")
    parser.add_argument("--bag_dir", default="/home/lry/catkin_ws/exp_bags")
    parser.add_argument("--output_dir", default="/home/lry/catkin_ws/logs/bag_metrics")
    parser.add_argument(
        "--processor",
        default="/home/lry/catkin_ws/src/my_gazebo_sim/scripts/bag_experiment_metrics.py",
        help="Path to bag metrics processor script",
    )
    parser.add_argument("--python_bin", default="/usr/bin/python3")
    parser.add_argument("--method", default="auto")
    parser.add_argument("--append_csv", default="/home/lry/catkin_ws/logs/bag_metrics/all_runs_summary.csv")
    parser.add_argument("--append_bag_info_csv", default="/home/lry/catkin_ws/logs/bag_metrics/all_bags_info.csv")
    parser.add_argument("--world_file", default="", help="Optional .world path for static obstacle overlay in PNG")
    parser.add_argument("--poll_sec", type=float, default=2.0)
    parser.add_argument("--settle_sec", type=float, default=3.0)
    parser.add_argument(
        "--state_file",
        default="",
        help="Optional custom state file path; defaults to <output_dir>/.bag_auto_state.json",
    )
    args = parser.parse_args()

    os.makedirs(args.bag_dir, exist_ok=True)
    os.makedirs(args.output_dir, exist_ok=True)
    state_file = args.state_file or os.path.join(args.output_dir, ".bag_auto_state.json")
    state = load_state(state_file)

    print(f"[{now()}] bag auto watcher started")
    print(f"[{now()}] bag_dir={args.bag_dir}")
    print(f"[{now()}] output_dir={args.output_dir}")
    print(f"[{now()}] state_file={state_file}")

    while True:
        try:
            for bag_path in list_bags(args.bag_dir):
                bag_name = os.path.basename(bag_path)
                rec = state["processed"].get(bag_name)
                if rec and rec.get("status") == "ok":
                    continue

                # Wait until bag file is stable (recording stopped).
                if not is_stable_file(bag_path, args.settle_sec):
                    continue

                code = run_processor(
                    args.python_bin,
                    args.processor,
                    bag_path,
                    args.output_dir,
                    args.method,
                    args.append_csv,
                    args.append_bag_info_csv,
                    args.world_file,
                )
                state["processed"][bag_name] = {
                    "status": "ok" if code == 0 else "failed",
                    "code": code,
                    "time": now(),
                }
                save_state(state_file, state)

            time.sleep(max(0.2, args.poll_sec))
        except KeyboardInterrupt:
            print(f"\n[{now()}] watcher stopped by user")
            break
        except Exception as e:
            print(f"[{now()}] watcher error: {e}")
            time.sleep(2.0)


if __name__ == "__main__":
    main()
