#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import csv
import glob
import math
import os
import re
import xml.etree.ElementTree as ET
from datetime import datetime

import rosbag

HAS_MPL = True
try:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
except Exception:
    HAS_MPL = False


def _odd_window_size(n):
    n = max(1, int(n))
    return n if (n % 2 == 1) else (n + 1)


def moving_median(seq, win):
    if not seq:
        return []
    w = _odd_window_size(win)
    h = w // 2
    out = []
    for i in range(len(seq)):
        lo = max(0, i - h)
        hi = min(len(seq), i + h + 1)
        s = sorted(seq[lo:hi])
        out.append(s[len(s) // 2])
    return out


def moving_mean(seq, win):
    if not seq:
        return []
    w = _odd_window_size(win)
    h = w // 2
    out = []
    for i in range(len(seq)):
        lo = max(0, i - h)
        hi = min(len(seq), i + h + 1)
        seg = seq[lo:hi]
        out.append(sum(seg) / float(len(seg)))
    return out


def smooth_rel_series(rx, ry, rd, med_win=9, mean_win=7):
    # Remove spikes first (median), then smooth jitter (mean).
    rx1 = moving_median(rx, med_win)
    ry1 = moving_median(ry, med_win)
    rd1 = moving_median(rd, med_win)
    rx2 = moving_mean(rx1, mean_win)
    ry2 = moving_mean(ry1, mean_win)
    rd2 = moving_mean(rd1, mean_win)
    return rx2, ry2, rd2


def choose_topic(available, candidates):
    for c in candidates:
        if c in available:
            return c
    return None


def parse_pose_text(pose_text):
    vals = [float(x) for x in pose_text.strip().split()]
    if len(vals) < 6:
        return None
    return vals[0], vals[1], vals[2], vals[3], vals[4], vals[5]


def apply_yaw_2d(x, y, yaw):
    c = math.cos(yaw)
    s = math.sin(yaw)
    return c * x - s * y, s * x + c * y


def compose_pose_2d(parent_pose, local_pose):
    px, py, pyaw = parent_pose
    lx, ly, lyaw = local_pose
    rx, ry = apply_yaw_2d(lx, ly, pyaw)
    return px + rx, py + ry, pyaw + lyaw


def pose6_to_pose2(pose6):
    x, y, _, _, _, yaw = pose6
    return x, y, yaw


def parse_geometry_radius(geom_node):
    """Return footprint-equivalent radius in meters from a <geometry> node."""
    if geom_node is None:
        return None

    cyl = geom_node.find("cylinder")
    if cyl is not None:
        r_node = cyl.find("radius")
        if r_node is not None and r_node.text:
            try:
                return max(0.05, float(r_node.text.strip()))
            except Exception:
                pass

    sph = geom_node.find("sphere")
    if sph is not None:
        r_node = sph.find("radius")
        if r_node is not None and r_node.text:
            try:
                return max(0.05, float(r_node.text.strip()))
            except Exception:
                pass

    box = geom_node.find("box")
    if box is not None:
        sz = box.find("size")
        if sz is not None and sz.text:
            try:
                sx, sy, _ = [float(v) for v in sz.text.strip().split()]
                # Equivalent circle radius for 2D footprint of a rectangle.
                return max(0.05, 0.5 * math.hypot(sx, sy))
            except Exception:
                pass

    mesh = geom_node.find("mesh")
    if mesh is not None:
        scale_node = mesh.find("scale")
        if scale_node is not None and scale_node.text:
            try:
                sc = [float(v) for v in scale_node.text.strip().split()]
                if len(sc) >= 2:
                    return max(0.05, 0.5 * math.hypot(sc[0], sc[1]))
            except Exception:
                pass

    return None


def parse_link_obstacles(link_node, model_world_pose2, model_name="", default_radius=0.25):
    """Parse one link into obstacle circles in world frame."""
    out = []
    link_pose2 = (0.0, 0.0, 0.0)
    lpose = link_node.find("pose")
    if lpose is not None and lpose.text:
        p = parse_pose_text(lpose.text)
        if p is not None:
            link_pose2 = pose6_to_pose2(p)

    link_world_pose2 = compose_pose_2d(model_world_pose2, link_pose2)
    lxw, lyw, lyaw = link_world_pose2

    collisions = link_node.findall("collision")
    if not collisions:
        # Fallback for simple tree links without explicit collision geometry.
        lname = link_node.attrib.get("name", "")
        if lname.startswith("tree"):
            out.append((lxw, lyw, default_radius, model_name))
        return out

    for col in collisions:
        cpose2 = (0.0, 0.0, 0.0)
        cpose = col.find("pose")
        if cpose is not None and cpose.text:
            p = parse_pose_text(cpose.text)
            if p is not None:
                cpose2 = pose6_to_pose2(p)
        cxw, cyw, _ = compose_pose_2d((lxw, lyw, lyaw), cpose2)

        r = parse_geometry_radius(col.find("geometry"))
        if r is None:
            r = default_radius
        out.append((cxw, cyw, r, model_name))

    return out


def candidate_model_dirs():
    roots = []
    env_paths = os.environ.get("GAZEBO_MODEL_PATH", "")
    if env_paths:
        roots.extend([p for p in env_paths.split(":") if p])
    roots.extend(
        [
            "/home/lry/catkin_ws/src/UAV_world/AIrDrone_Security-main/airdrone_navigation/models/ADS_models",
            "/home/lry/catkin_ws/src/UAV_world/AIrDrone_Security-main/airdrone_navigation/models/gazebo_models",
            "/usr/share/gazebo-11/models",
        ]
    )
    uniq = []
    seen = set()
    for p in roots:
        if p not in seen and os.path.isdir(p):
            seen.add(p)
            uniq.append(p)
    return uniq


def resolve_model_sdf(uri_text):
    if not uri_text:
        return ""
    uri = uri_text.strip()

    if uri.startswith("model://"):
        model_name = uri[len("model://") :].split("/")[0]
        for root in candidate_model_dirs():
            mdir = os.path.join(root, model_name)
            if not os.path.isdir(mdir):
                continue
            model_sdf = os.path.join(mdir, "model.sdf")
            if os.path.isfile(model_sdf):
                return model_sdf
            sdf_files = sorted(glob.glob(os.path.join(mdir, "*.sdf")))
            if sdf_files:
                return sdf_files[0]
        return ""

    if uri.startswith("file://"):
        path = uri[len("file://") :]
        if os.path.isfile(path):
            return path
        return ""

    if os.path.isfile(uri):
        return uri
    return ""


def load_obstacles_from_model_sdf(sdf_file, model_world_pose2):
    out = []
    if not sdf_file or not os.path.isfile(sdf_file):
        return out
    try:
        root = ET.parse(sdf_file).getroot()
    except Exception:
        return out

    models = root.findall(".//model")
    if not models and root.tag == "model":
        models = [root]

    for model in models:
        mp2 = model_world_pose2
        mpose = model.find("pose")
        if mpose is not None and mpose.text:
            p = parse_pose_text(mpose.text)
            if p is not None:
                mp2 = compose_pose_2d(model_world_pose2, pose6_to_pose2(p))
        links = model.findall("link")
        for link in links:
            out.extend(parse_link_obstacles(link, mp2, model_name=model.attrib.get("name", "")))
    return out


def load_static_obs_from_world(world_file):
    """Parse static obstacle circles (x, y, radius) from Gazebo world/SDF."""
    if not world_file:
        return []
    if not os.path.isfile(world_file):
        return []

    try:
        tree = ET.parse(world_file)
        root = tree.getroot()
    except Exception:
        return []

    out = []

    # 1) Direct models defined in world.
    for model in root.findall(".//model"):
        pose6 = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        mpose = model.find("pose")
        if mpose is not None and mpose.text:
            p = parse_pose_text(mpose.text)
            if p is not None:
                pose6 = p
        mp2 = pose6_to_pose2(pose6)

        links = model.findall("link")
        if links:
            for link in links:
                out.extend(parse_link_obstacles(link, mp2))
        else:
            # Fallback if model has no explicit links in world file.
            mname = model.attrib.get("name", "")
            if mname.startswith("tree"):
                out.append((mp2[0], mp2[1], 0.25, mname))

    # 2) Included models (model://...).
    for inc in root.findall(".//include"):
        uri_node = inc.find("uri")
        if uri_node is None or not uri_node.text:
            continue
        pose6 = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        pnode = inc.find("pose")
        if pnode is not None and pnode.text:
            p = parse_pose_text(pnode.text)
            if p is not None:
                pose6 = p
        mp2 = pose6_to_pose2(pose6)

        sdf_file = resolve_model_sdf(uri_node.text.strip())
        parsed = load_obstacles_from_model_sdf(sdf_file, mp2)
        if parsed:
            out.extend(parsed)
        else:
            # Conservative fallback: one small circle at include pose.
            out.append((mp2[0], mp2[1], 0.30, inc.findtext("name", "").strip()))

    return out


def filter_world_obstacles(obs_list, max_radius=4.5):
    """Filter out map-floor like obstacles and overly large footprints for visualization."""
    if not obs_list:
        return []
    ignored_tokens = ("ground", "plane", "floor", "asphalt")
    out = []
    for ox, oy, rr, name in obs_list:
        lname = (name or "").lower()
        if any(tok in lname for tok in ignored_tokens):
            continue
        if rr is None or rr <= 0.0:
            continue
        if rr > max_radius:
            continue
        out.append((ox, oy, rr, name))
    return out


def infer_world_file(world_file_arg):
    """Resolve world file path.

    Priority:
    1) explicit --world_file
    2) parse default world_name from ego_orb_gazebo_bridge.launch
    """
    if world_file_arg and os.path.isfile(world_file_arg):
        return world_file_arg

    launch_file = "/home/lry/catkin_ws/src/my_gazebo_sim/launch/ego_orb_gazebo_bridge.launch"
    if not os.path.isfile(launch_file):
        return ""

    try:
        root = ET.parse(launch_file).getroot()
    except Exception:
        return ""

    for arg in root.findall("arg"):
        if arg.attrib.get("name", "") == "world_name":
            w = arg.attrib.get("default", "").strip()
            if w and os.path.isfile(w):
                return w
            break
    return ""


def euclid3(a, b):
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2)


def integrate_path(odom_samples, t0, t1):
    pts = [p for p in odom_samples if t0 <= p[0] <= t1]
    if len(pts) < 2:
        return 0.0, 0.0
    length = 0.0
    for i in range(1, len(pts)):
        length += euclid3(pts[i - 1][1:], pts[i][1:])
    duration = max(1e-6, pts[-1][0] - pts[0][0])
    return length, length / duration


def compute_waypoint_reached_count(odom_samples, mission_start, mission_end, wp_points, xy_thresh, z_thresh):
    """Count waypoints that are actually reached by odom trajectory."""
    if not wp_points:
        return 0
    pts = [p for p in odom_samples if mission_start <= p[0] <= mission_end]
    if not pts:
        return 0

    reached = 0
    for _, wx, wy, wz in sorted(wp_points, key=lambda x: x[0]):
        ok = False
        for _, ox, oy, oz in pts:
            dxy = math.hypot(ox - wx, oy - wy)
            dz = abs(oz - wz)
            if dxy <= xy_thresh and dz <= z_thresh:
                ok = True
                break
        if ok:
            reached += 1
    return reached


def find_final_waypoint_reach_time(odom_samples, mission_start, mission_end, wp_points, xy_thresh, z_thresh):
    """Return first timestamp when the final waypoint is reached, else None."""
    if not wp_points:
        return None
    final_idx = max(p[0] for p in wp_points)
    target = None
    for p in wp_points:
        if p[0] == final_idx:
            target = p
            break
    if target is None:
        return None

    _, wx, wy, wz = target
    for ts, ox, oy, oz in odom_samples:
        if ts < mission_start or ts > mission_end:
            continue
        dxy = math.hypot(ox - wx, oy - wy)
        dz = abs(oz - wz)
        if dxy <= xy_thresh and dz <= z_thresh:
            return ts
    return None


def find_waypoint_reach_time_by_index(odom_samples, mission_start, mission_end, wp_points, target_idx, xy_thresh, z_thresh):
    """Return first timestamp when waypoint[target_idx] is reached, else None."""
    if not wp_points or target_idx <= 0:
        return None
    target = None
    for p in wp_points:
        if int(p[0]) == int(target_idx):
            target = p
            break
    if target is None:
        return None

    _, wx, wy, wz = target
    for ts, ox, oy, oz in odom_samples:
        if ts < mission_start or ts > mission_end:
            continue
        dxy = math.hypot(ox - wx, oy - wy)
        dz = abs(oz - wz)
        if dxy <= xy_thresh and dz <= z_thresh:
            return ts
    return None


def set_compact_xy_limits(ax, xs, ys, goal_xy, pad=2.0):
    """Auto-focus XY plot around trajectory/goals for cleaner visualization."""
    pts = []
    if xs and ys:
        pts.extend(zip(xs, ys))
    if goal_xy:
        pts.extend(goal_xy)
    if not pts:
        return

    min_x = min(p[0] for p in pts)
    max_x = max(p[0] for p in pts)
    min_y = min(p[1] for p in pts)
    max_y = max(p[1] for p in pts)

    dx = max(1.0, max_x - min_x)
    dy = max(1.0, max_y - min_y)
    cx = 0.5 * (min_x + max_x)
    cy = 0.5 * (min_y + max_y)
    half = 0.5 * max(dx, dy) + max(0.5, pad)

    ax.set_xlim(cx - half, cx + half)
    ax.set_ylim(cy - half, cy + half)


def collision_episodes(dist_series, threshold):
    in_col = False
    episodes = 0
    for _, d in dist_series:
        if d is None:
            continue
        now_col = d < threshold
        if now_col and not in_col:
            episodes += 1
        in_col = now_col
    return episodes


def infer_method(bag_name, method_arg):
    if method_arg and method_arg != "auto":
        return method_arg
    low = bag_name.lower()
    if "global" in low or "g1" in low:
        return "global_local"
    if "ego" in low or "local" in low or "l1" in low:
        return "ego_only"
    return "unknown"


def list_bags(bag_path, bag_dir):
    if bag_path:
        return [bag_path]
    if not os.path.isdir(bag_dir):
        raise RuntimeError(f"bag_dir not found: {bag_dir}")
    bags = [os.path.join(bag_dir, x) for x in os.listdir(bag_dir) if x.endswith(".bag")]
    bags.sort()
    if not bags:
        raise RuntimeError(f"No .bag files found under: {bag_dir}")
    return bags


def bag_meta_row(bag_path, bag, topics):
    start_t = bag.get_start_time()
    end_t = bag.get_end_time()
    duration = max(0.0, end_t - start_t)
    size_mb = os.path.getsize(bag_path) / (1024.0 * 1024.0)
    msg_count = sum(topics[t].message_count for t in topics)
    return {
        "bag_name": os.path.basename(bag_path),
        "bag_path": bag_path,
        "start_time": f"{start_t:.3f}",
        "end_time": f"{end_t:.3f}",
        "duration_sec": f"{duration:.3f}",
        "file_size_mb": f"{size_mb:.3f}",
        "topic_count": len(topics),
        "message_count": int(msg_count),
    }


def write_csv(path, rows, fieldnames):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fieldnames)
        w.writeheader()
        for r in rows:
            w.writerow(r)


def append_csv(path, row, fieldnames):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    exists = os.path.exists(path)
    with open(path, "a", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fieldnames)
        if not exists:
            w.writeheader()
        w.writerow(row)


def process_one_bag(bag_path, args, stamp):
    bag_name = os.path.splitext(os.path.basename(bag_path))[0]
    method = infer_method(bag_name, args.method)

    bag = rosbag.Bag(bag_path, "r")
    topic_info = bag.get_type_and_topic_info().topics
    available = set(topic_info.keys())

    meta = bag_meta_row(bag_path, bag, topic_info)

    odom_topic = choose_topic(
        available,
        ["/visual_slam/odom_safe_filtered", "/visual_slam/odom_aligned", "/odom"],
    )
    rosout_topic = choose_topic(available, ["/rosout_agg", "/rosout"])
    goal_topic = args.goal_topic if args.goal_topic in available else choose_topic(
        available, ["/planner/ego_goal", "/move_base_simple/goal"]
    )
    model_states_topic = args.model_states_topic if args.model_states_topic in available else None

    if odom_topic is None:
        bag.close()
        raise RuntimeError(f"{bag_name}: No odometry topic found in bag")

    odom_samples = []
    goal_samples = []
    min_dist_any = []
    min_dist_dyn = []
    dyn_tracks = {}
    dyn_rel_samples = []
    static_xy = {}
    world_static_obs = filter_world_obstacles(
        load_static_obs_from_world(args.world_file),
        max_radius=args.world_obs_max_radius,
    )

    final_plan_fail = 0
    final_plan_succ = 0
    finished_route = False
    finished_route_t = None
    goal_reached_msgs = 0
    no_progress_skip = 0
    max_wp_idx = 0
    total_wp = 0

    wp_regex = re.compile(r"(?:switch to wp|-> wp)\[(\d+)/(\d+)\]")
    wp_point_regex = re.compile(
        r"auto_goal_patrol -> wp\[(\d+)/(\d+)\]: \(([-+]?\d*\.?\d+),\s*([-+]?\d*\.?\d+),\s*([-+]?\d*\.?\d+)\)"
    )
    wp_points = {}

    read_topics = [odom_topic]
    if goal_topic is not None:
        read_topics.append(goal_topic)
    if model_states_topic is not None:
        read_topics.append(model_states_topic)
    if rosout_topic is not None:
        read_topics.append(rosout_topic)

    for topic, msg, t in bag.read_messages(topics=read_topics):
        ts = t.to_sec()

        if topic == odom_topic:
            p = msg.pose.pose.position
            odom_samples.append((ts, p.x, p.y, p.z))

        elif goal_topic is not None and topic == goal_topic:
            p = msg.pose.position
            goal_samples.append((ts, p.x, p.y, p.z))

        elif model_states_topic is not None and topic == model_states_topic:
            names = list(msg.name)
            if args.uav_model not in names:
                continue
            u_idx = names.index(args.uav_model)
            if u_idx >= len(msg.pose):
                continue
            up = msg.pose[u_idx].position
            u_xyz = (up.x, up.y, up.z)

            d_any = None
            d_dyn = None
            for i, name in enumerate(names):
                if i == u_idx or i >= len(msg.pose):
                    continue
                if name in ("ground_plane", "sun"):
                    continue
                op = msg.pose[i].position
                d = euclid3(u_xyz, (op.x, op.y, op.z))
                if d_any is None or d < d_any:
                    d_any = d
                if name.startswith(args.dyn_prefix):
                    if d_dyn is None or d < d_dyn:
                        d_dyn = d
                    dyn_tracks.setdefault(name, []).append((op.x, op.y))
                    dyn_rel_samples.append((ts, name, up.x - op.x, up.y - op.y, up.z - op.z, d))
                else:
                    # Static models: keep one representative XY sample for plotting.
                    static_xy.setdefault(name, (op.x, op.y))

            min_dist_any.append((ts, d_any))
            min_dist_dyn.append((ts, d_dyn))

        elif rosout_topic is not None and topic == rosout_topic:
            text = msg.msg
            if "final_plan_success=0" in text:
                final_plan_fail += 1
            elif "final_plan_success=1" in text:
                final_plan_succ += 1
            if "auto_goal_patrol finished route" in text:
                finished_route = True
                finished_route_t = ts
            if "Goal reached." in text:
                goal_reached_msgs += 1
            if "no-progress timeout, skip wp[" in text:
                no_progress_skip += 1
            m = wp_regex.search(text)
            if m:
                max_wp_idx = max(max_wp_idx, int(m.group(1)))
                total_wp = max(total_wp, int(m.group(2)))
            mp = wp_point_regex.search(text)
            if mp:
                idx = int(mp.group(1))
                total_wp = max(total_wp, int(mp.group(2)))
                wp_points[idx] = (idx, float(mp.group(3)), float(mp.group(4)), float(mp.group(5)))

    bag.close()

    if not odom_samples:
        raise RuntimeError(f"{bag_name}: No odom samples in bag")

    mission_start = goal_samples[0][0] if goal_samples else odom_samples[0][0]
    mission_end_raw = finished_route_t if finished_route_t is not None else odom_samples[-1][0]
    final_wp_reach_t = find_final_waypoint_reach_time(
        odom_samples,
        mission_start,
        mission_end_raw,
        list(wp_points.values()),
        args.wp_reach_xy_thresh,
        args.wp_reach_z_thresh,
    )
    mission_end = final_wp_reach_t if (finished_route_t is None and final_wp_reach_t is not None) else mission_end_raw
    if mission_end < mission_start:
        mission_end = odom_samples[-1][0]

    path_len, avg_speed = integrate_path(odom_samples, mission_start, mission_end)
    mission_time = max(0.0, mission_end - mission_start)

    any_in_window = [d for ts, d in min_dist_any if mission_start <= ts <= mission_end and d is not None]
    dyn_in_window = [d for ts, d in min_dist_dyn if mission_start <= ts <= mission_end and d is not None]
    min_any = min(any_in_window) if any_in_window else None
    min_dyn = min(dyn_in_window) if dyn_in_window else None
    col_eps = collision_episodes(
        [(ts, d) for ts, d in min_dist_any if mission_start <= ts <= mission_end],
        args.collision_threshold,
    )
    collision_flag = 1 if col_eps > 0 else 0

    success = 1 if (finished_route or (total_wp > 0 and max_wp_idx >= total_wp)) else 0
    reached_wp_count = compute_waypoint_reached_count(
        odom_samples,
        mission_start,
        mission_end,
        list(wp_points.values()),
        args.wp_reach_xy_thresh,
        args.wp_reach_z_thresh,
    )
    strict_success = 1 if (total_wp > 0 and reached_wp_count >= total_wp) else 0
    reached_wp_ratio = (
        f"{(100.0 * reached_wp_count / total_wp):.1f}%"
        if total_wp > 0
        else "NA"
    )

    summary = {
        "bag_name": bag_name,
        "method": method,
        "odom_topic": odom_topic,
        "goal_topic": goal_topic if goal_topic else "",
        "mission_start": f"{mission_start:.3f}",
        "mission_end": f"{mission_end:.3f}",
        "mission_time_sec": f"{mission_time:.3f}",
        "path_length_m": f"{path_len:.3f}",
        "avg_speed_mps": f"{avg_speed:.3f}",
        "success": success,
        "collision": collision_flag,
        "collision_episodes": col_eps,
        "min_obs_dist_m": "" if min_any is None else f"{min_any:.3f}",
        "min_dyn_obs_dist_m": "" if min_dyn is None else f"{min_dyn:.3f}",
        "goal_msgs": len(goal_samples),
        "goal_reached_msgs": goal_reached_msgs,
        "no_progress_skip_count": no_progress_skip,
        "final_plan_fail_count": final_plan_fail,
        "final_plan_success_count": final_plan_succ,
        "max_wp_idx_seen": max_wp_idx,
        "total_wp_seen": total_wp,
        "waypoint_reached_count": reached_wp_count,
        "waypoint_reached_ratio": reached_wp_ratio,
        "strict_success": strict_success,
    }

    # Human-readable interpretation (Chinese).
    fail_reasons = []
    if summary["success"] == 0:
        if total_wp > 0 and max_wp_idx < total_wp:
            fail_reasons.append(f"仅到达航点 {max_wp_idx}/{total_wp}")
        if no_progress_skip > 0:
            fail_reasons.append(f"无进展跳点 {no_progress_skip} 次")
        if final_plan_fail > 0:
            fail_reasons.append(f"局部规划失败 {final_plan_fail} 次")
        if not fail_reasons:
            fail_reasons.append("未检测到完成标志")

    brief = (
        f"方法={method} | 流程成功={summary['success']} | 严格成功={summary['strict_success']} | 碰撞={summary['collision']} | "
        f"耗时={summary['mission_time_sec']}s | 路程={summary['path_length_m']}m | 平均速度={summary['avg_speed_mps']}m/s"
    )
    detail_lines = [
        f"bag: {bag_name}",
        f"method: {method}",
        f"success: {summary['success']}",
        f"strict_success: {summary['strict_success']}",
        f"collision: {summary['collision']} (episodes={summary['collision_episodes']})",
        f"mission_time_sec: {summary['mission_time_sec']}",
        f"path_length_m: {summary['path_length_m']}",
        f"avg_speed_mps: {summary['avg_speed_mps']}",
        f"min_obs_dist_m: {summary['min_obs_dist_m'] or 'NA'}",
        f"min_dyn_obs_dist_m: {summary['min_dyn_obs_dist_m'] or 'NA'}",
        f"goal_msgs: {summary['goal_msgs']}",
        f"goal_reached_msgs: {summary['goal_reached_msgs']}",
        f"no_progress_skip_count: {summary['no_progress_skip_count']}",
        f"final_plan_fail_count: {summary['final_plan_fail_count']}",
        f"final_plan_success_count: {summary['final_plan_success_count']}",
        f"waypoint_progress: {summary['max_wp_idx_seen']}/{summary['total_wp_seen']}",
        f"waypoint_reached_count: {summary['waypoint_reached_count']}/{summary['total_wp_seen']}",
        f"waypoint_reached_ratio: {summary['waypoint_reached_ratio']}",
    ]
    if fail_reasons:
        detail_lines.append("failure_reason: " + "；".join(fail_reasons))
    else:
        detail_lines.append("failure_reason: 无（任务成功完成）")

    readable_row = {
        "bag_name": bag_name,
        "method": method,
        "result": "成功" if summary["success"] == 1 else "失败",
        "strict_result": "成功" if summary["strict_success"] == 1 else "失败",
        "collision": "是" if summary["collision"] == 1 else "否",
        "mission_time_sec": summary["mission_time_sec"],
        "path_length_m": summary["path_length_m"],
        "avg_speed_mps": summary["avg_speed_mps"],
        "min_obs_dist_m": summary["min_obs_dist_m"] or "NA",
        "waypoint_progress": f"{summary['max_wp_idx_seen']}/{summary['total_wp_seen']}",
        "waypoint_reached_count": f"{summary['waypoint_reached_count']}/{summary['total_wp_seen']}",
        "waypoint_reached_ratio": summary["waypoint_reached_ratio"],
        "no_progress_skip_count": summary["no_progress_skip_count"],
        "final_plan_fail_count": summary["final_plan_fail_count"],
        "conclusion": "；".join(fail_reasons) if fail_reasons else "任务成功完成",
        "brief": brief,
    }

    summary_csv = ""
    readable_csv = ""
    if args.save_csv:
        summary_csv = os.path.join(args.output_dir, f"{bag_name}_{stamp}_summary.csv")
        write_csv(summary_csv, [summary], list(summary.keys()))
        readable_csv = os.path.join(args.output_dir, f"{bag_name}_{stamp}_readable.csv")
        write_csv(readable_csv, [readable_row], list(readable_row.keys()))

    readable_txt = os.path.join(args.output_dir, f"{bag_name}_{stamp}_readable.txt")
    with open(readable_txt, "w", encoding="utf-8") as f:
        f.write(brief + "\n\n")
        f.write("\n".join(detail_lines) + "\n")

    bag_info_txt = os.path.join(args.output_dir, f"{bag_name}_{stamp}_bag_info.txt")
    with open(bag_info_txt, "w", encoding="utf-8") as f:
        f.write("Bag Basic Info\n")
        f.write(f"bag_name: {meta['bag_name']}\n")
        f.write(f"bag_path: {meta['bag_path']}\n")
        f.write(f"start_time: {meta['start_time']}\n")
        f.write(f"end_time: {meta['end_time']}\n")
        f.write(f"duration_sec: {meta['duration_sec']}\n")
        f.write(f"file_size_mb: {meta['file_size_mb']}\n")
        f.write(f"topic_count: {meta['topic_count']}\n")
        f.write(f"message_count: {meta['message_count']}\n")
        f.write("\nTopics\n")
        for tname in sorted(topic_info.keys()):
            info = topic_info[tname]
            freq = "" if info.frequency is None else f"{info.frequency:.3f}"
            f.write(f"- {tname} | {info.msg_type} | count={int(info.message_count)} | hz={freq}\n")

    bag_info_csv = ""
    topics_csv = ""
    if args.save_csv:
        bag_info_csv = os.path.join(args.output_dir, f"{bag_name}_{stamp}_bag_info.csv")
        write_csv(bag_info_csv, [meta], list(meta.keys()))

        topics_csv = os.path.join(args.output_dir, f"{bag_name}_{stamp}_topics.csv")
        topic_rows = []
        for tname, info in topic_info.items():
            topic_rows.append(
                {
                    "topic": tname,
                    "type": info.msg_type,
                    "message_count": int(info.message_count),
                    "frequency": "" if info.frequency is None else f"{info.frequency:.3f}",
                }
            )
        topic_rows.sort(key=lambda x: x["topic"])
        write_csv(topics_csv, topic_rows, ["topic", "type", "message_count", "frequency"])

    plot_end = mission_end
    if args.plot_cut_wp_idx > 0:
        cut_t = find_waypoint_reach_time_by_index(
            odom_samples,
            mission_start,
            mission_end,
            list(wp_points.values()),
            args.plot_cut_wp_idx,
            args.wp_reach_xy_thresh,
            args.wp_reach_z_thresh,
        )
        if cut_t is not None and cut_t >= mission_start:
            plot_end = min(plot_end, cut_t)

    traj_png = ""
    dist_png = ""
    dyn_rel_png = ""
    dyn_rel_pngs = []
    if HAS_MPL:
        # Plot: XY trajectory + goals + dynamic obstacle tracks.
        traj_png = os.path.join(args.output_dir, f"{bag_name}_{stamp}_traj_xy.png")
        xs = [p[1] for p in odom_samples if mission_start <= p[0] <= plot_end]
        ys = [p[2] for p in odom_samples if mission_start <= p[0] <= plot_end]
        fig = plt.figure(figsize=(8, 7))
        ax = fig.add_subplot(111)
        if xs and ys:
            ax.plot(xs, ys, "b-", linewidth=1.6, label="UAV trajectory")
            ax.scatter([xs[0]], [ys[0]], c="g", s=40, label="start")
            ax.scatter([xs[-1]], [ys[-1]], c="r", s=40, label="end")
        if world_static_obs:
            # Draw size-aware static obstacles parsed from world/SDF.
            first = True
            for ox, oy, rr, _name in world_static_obs:
                draw_r = max(rr * args.world_obs_radius_scale, args.world_obs_min_draw_radius)
                circ = plt.Circle(
                    (ox, oy),
                    draw_r,
                    facecolor="#A8A8A8",
                    edgecolor="#5F5F5F",
                    linewidth=0.4,
                    alpha=0.32,
                    label=("world obs" if first else None),
                )
                ax.add_patch(circ)
                first = False
        elif static_xy:
            # Fallback: static models from /gazebo/model_states.
            spts = list(static_xy.values())
            stride = max(1, len(spts) // 2500)
            spts = spts[::stride]
            ax.scatter(
                [p[0] for p in spts],
                [p[1] for p in spts],
                c="lightgray",
                s=8,
                alpha=0.55,
                marker="s",
                label="static obs",
            )
        goal_xy_for_focus = []
        if wp_points:
            # Prefer true waypoint index from auto_goal_patrol logs: wp[1], wp[2], ...
            ordered_wp = [wp_points[k] for k in sorted(wp_points.keys())]
            if args.max_plot_wp > 0:
                ordered_wp = ordered_wp[: args.max_plot_wp]
            gx = [g[1] for g in ordered_wp]
            gy = [g[2] for g in ordered_wp]
            goal_xy_for_focus = list(zip(gx, gy))
            ax.scatter(gx, gy, c="k", s=20, alpha=0.6, label="goals")
            if len(ordered_wp) >= 2:
                ax.plot(gx, gy, "k--", linewidth=0.9, alpha=0.35, label="goal order")
            for idx, gxx, gyy, _ in ordered_wp:
                ax.text(gxx + 0.06, gyy + 0.06, str(idx), fontsize=7, color="k", alpha=0.8)
        elif goal_samples:
            # Fallback when waypoint index logs are absent.
            uniq_goals = []
            for _, gx, gy, gz in goal_samples:
                if not uniq_goals:
                    uniq_goals.append((gx, gy, gz))
                    continue
                lx, ly, lz = uniq_goals[-1]
                if euclid3((gx, gy, gz), (lx, ly, lz)) > 0.25:
                    uniq_goals.append((gx, gy, gz))
            if args.max_plot_wp > 0:
                uniq_goals = uniq_goals[: args.max_plot_wp]
            gx = [g[0] for g in uniq_goals]
            gy = [g[1] for g in uniq_goals]
            goal_xy_for_focus = list(zip(gx, gy))
            ax.scatter(gx, gy, c="k", s=20, alpha=0.6, label="goals")
            if len(uniq_goals) >= 2:
                ax.plot(gx, gy, "k--", linewidth=0.9, alpha=0.35, label="goal order")
            for i, (gxx, gyy, _) in enumerate(uniq_goals, start=1):
                ax.text(gxx + 0.06, gyy + 0.06, str(i), fontsize=7, color="k", alpha=0.8)
        for name, pts in dyn_tracks.items():
            if len(pts) >= 2:
                ax.plot([p[0] for p in pts], [p[1] for p in pts], "--", linewidth=1.0, alpha=0.7, label=name)
        set_compact_xy_limits(ax, xs, ys, goal_xy_for_focus, pad=args.xy_focus_pad)
        if args.traj_xlim_min is not None and args.traj_xlim_max is not None:
            ax.set_xlim(args.traj_xlim_min, args.traj_xlim_max)
        if args.traj_ylim_min is not None and args.traj_ylim_max is not None:
            ax.set_ylim(args.traj_ylim_min, args.traj_ylim_max)
        ax.set_xlabel("x (m)")
        ax.set_ylabel("y (m)")
        ax.set_title(f"Trajectory XY - {bag_name} ({method})")
        ax.grid(True, linestyle="--", alpha=0.4)
        ax.set_aspect("equal", adjustable="box")
        ax.legend(loc="best", fontsize=8)
        fig.tight_layout()
        fig.savefig(traj_png, dpi=160)
        plt.close(fig)

        # Plot: min obstacle distance over time.
        dist_png = os.path.join(args.output_dir, f"{bag_name}_{stamp}_dist_time.png")
        fig = plt.figure(figsize=(9, 4))
        ax = fig.add_subplot(111)
        ta = [ts - mission_start for ts, d in min_dist_any if mission_start <= ts <= mission_end and d is not None]
        da = [d for ts, d in min_dist_any if mission_start <= ts <= mission_end and d is not None]
        td = [ts - mission_start for ts, d in min_dist_dyn if mission_start <= ts <= mission_end and d is not None]
        dd = [d for ts, d in min_dist_dyn if mission_start <= ts <= mission_end and d is not None]
        if ta:
            ax.plot(ta, da, "m-", linewidth=1.3, label="min dist to any obstacle")
        if td:
            ax.plot(td, dd, "c-", linewidth=1.2, label=f"min dist to {args.dyn_prefix}*")
        ax.axhline(args.collision_threshold, color="r", linestyle="--", linewidth=1.0, label="collision threshold")
        ax.set_xlabel("time from mission start (s)")
        ax.set_ylabel("distance (m)")
        ax.set_title(f"Obstacle Distance - {bag_name} ({method})")
        ax.grid(True, linestyle="--", alpha=0.4)
        ax.legend(loc="best", fontsize=8)
        fig.tight_layout()
        fig.savefig(dist_png, dpi=160)
        plt.close(fig)

        # Plot: one relative-position figure per dynamic obstacle.
        if dyn_rel_samples:
            by_name = {}
            for s in dyn_rel_samples:
                if mission_start <= s[0] <= mission_end:
                    by_name.setdefault(s[1], []).append(s)

            rel_half_window = 8.0
            for rel_name in sorted(by_name.keys()):
                samples = by_name[rel_name]
                if not samples:
                    continue
                nearest = min(samples, key=lambda s: s[5])
                center_t = nearest[0]
                rel_seg = [s for s in samples if (center_t - rel_half_window) <= s[0] <= (center_t + rel_half_window)]
                if not rel_seg:
                    continue

                out_png = os.path.join(args.output_dir, f"{bag_name}_{stamp}_dyn_rel_{rel_name}.png")
                rt = [s[0] - mission_start for s in rel_seg]
                rx = [s[2] for s in rel_seg]
                ry = [s[3] for s in rel_seg]
                rd = [s[5] for s in rel_seg]
                rx_s, ry_s, rd_s = smooth_rel_series(
                    rx,
                    ry,
                    rd,
                    med_win=args.rel_smooth_median_win,
                    mean_win=args.rel_smooth_mean_win,
                )

                fig = plt.figure(figsize=(9, 8))
                ax1 = fig.add_subplot(211)
                ax1.plot(rx_s, ry_s, "b-", linewidth=1.8, label=f"UAV in {rel_name} frame")
                ax1.scatter([rx_s[0]], [ry_s[0]], c="g", s=36, label="start")
                ax1.scatter([rx_s[-1]], [ry_s[-1]], c="r", s=36, label="end")
                ax1.scatter([0.0], [0.0], c="k", s=28, marker="x", label=rel_name)
                circle = plt.Circle((0.0, 0.0), args.collision_threshold, color="r", fill=False, linestyle="--", linewidth=1.0)
                ax1.add_patch(circle)
                ax1.set_aspect("equal", adjustable="box")
                ax1.grid(True, linestyle="--", alpha=0.4)
                ax1.set_xlabel("relative x (m): UAV - obstacle")
                ax1.set_ylabel("relative y (m): UAV - obstacle")
                ax1.set_title(f"Relative XY around closest dynamic encounter ({rel_name})")
                ax1.legend(loc="best", fontsize=8)

                ax2 = fig.add_subplot(212)
                ax2.plot(rt, rd_s, "c-", linewidth=1.6, label=f"dist(UAV,{rel_name})")
                ax2.plot(rt, rx_s, color="#4B4BFF", linewidth=1.2, alpha=0.95, label="rel x")
                ax2.plot(rt, ry_s, color="#FF8C00", linewidth=1.2, alpha=0.95, label="rel y")
                ax2.axhline(args.collision_threshold, color="r", linestyle="--", linewidth=1.0, label="collision threshold")
                ax2.axvline(center_t - mission_start, color="#666666", linestyle=":", linewidth=1.0, label="closest time")
                ax2.grid(True, linestyle="--", alpha=0.4)
                ax2.set_xlabel("time from mission start (s)")
                ax2.set_ylabel("meter / distance (m)")
                ax2.set_title(f"Relative Components & Distance (window +/-{rel_half_window:.0f}s)")
                ax2.legend(loc="best", fontsize=8)

                fig.tight_layout()
                fig.savefig(out_png, dpi=160)
                plt.close(fig)
                dyn_rel_pngs.append(out_png)

            if dyn_rel_pngs:
                dyn_rel_png = dyn_rel_pngs[0]

    return {
        "summary": summary,
        "readable": readable_row,
        "meta": meta,
        "summary_csv": summary_csv,
        "readable_csv": readable_csv,
        "readable_txt": readable_txt,
        "bag_info_txt": bag_info_txt,
        "bag_info_csv": bag_info_csv,
        "topics_csv": topics_csv,
        "traj_png": traj_png,
        "dist_png": dist_png,
        "dyn_rel_png": dyn_rel_png,
        "dyn_rel_pngs": dyn_rel_pngs,
    }


def main():
    parser = argparse.ArgumentParser(description="Compute experiment metrics from ROS1 bag(s).")
    parser.add_argument("--bag", default="", help="Single input .bag file")
    parser.add_argument("--bag_dir", default="/home/lry/catkin_ws/exp_bags", help="Directory containing .bag files")
    parser.add_argument("--method", default="auto", help="Method label: global_local / ego_only / auto")
    parser.add_argument("--output_dir", default=os.path.expanduser("~/catkin_ws/logs/bag_metrics"))
    parser.add_argument("--save_csv", action="store_true", help="Generate csv outputs (default: off)")
    parser.add_argument("--append_csv", default="", help="Optional summary csv to append")
    parser.add_argument("--append_bag_info_csv", default="", help="Optional bag-info csv to append")
    parser.add_argument("--uav_model", default="simple_uav")
    parser.add_argument("--dyn_prefix", default="dyn_obs")
    parser.add_argument("--goal_topic", default="/move_base_simple/goal")
    parser.add_argument("--model_states_topic", default="/gazebo/model_states")
    parser.add_argument("--world_file", default="", help="Optional .world path for static obstacle overlay")
    parser.add_argument(
        "--world_obs_max_radius",
        type=float,
        default=4.5,
        help="Max static obstacle footprint radius (m) to draw from world/SDF",
    )
    parser.add_argument(
        "--world_obs_radius_scale",
        type=float,
        default=0.55,
        help="Scale factor for displayed world obstacle footprint radius",
    )
    parser.add_argument(
        "--world_obs_min_draw_radius",
        type=float,
        default=0.35,
        help="Min visualization radius (m) for static obstacles",
    )
    parser.add_argument("--xy_focus_pad", type=float, default=1.0, help="XY plot extra margin around traj/goals (m)")
    parser.add_argument("--traj_xlim_min", type=float, default=None, help="Optional fixed xmin for trajectory plot")
    parser.add_argument("--traj_xlim_max", type=float, default=None, help="Optional fixed xmax for trajectory plot")
    parser.add_argument("--traj_ylim_min", type=float, default=None, help="Optional fixed ymin for trajectory plot")
    parser.add_argument("--traj_ylim_max", type=float, default=None, help="Optional fixed ymax for trajectory plot")
    parser.add_argument(
        "--max_plot_wp",
        type=int,
        default=0,
        help="Only plot first N waypoints in trajectory figure (<=0 means all).",
    )
    parser.add_argument(
        "--plot_cut_wp_idx",
        type=int,
        default=0,
        help="Cut trajectory drawing at first reach of waypoint index N (<=0 disables).",
    )
    parser.add_argument("--collision_threshold", type=float, default=0.45)
    parser.add_argument("--rel_smooth_median_win", type=int, default=9, help="Median window for dynamic-relative smoothing")
    parser.add_argument("--rel_smooth_mean_win", type=int, default=7, help="Mean window after median smoothing")
    parser.add_argument("--wp_reach_xy_thresh", type=float, default=1.0)
    parser.add_argument("--wp_reach_z_thresh", type=float, default=0.5)
    args = parser.parse_args()
    args.world_file = infer_world_file(args.world_file)

    os.makedirs(args.output_dir, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")

    bags = list_bags(args.bag, args.bag_dir)
    print(f"Found {len(bags)} bag(s).")

    all_summary = []
    all_readable = []
    all_meta = []

    for b in bags:
        print(f"\n=== Processing: {b}")
        out = process_one_bag(b, args, stamp)
        all_summary.append(out["summary"])
        all_readable.append(out["readable"])
        all_meta.append(out["meta"])

        print("Done.")
        print(f"  readable txt: {out['readable_txt']}")
        print(f"  bag info txt: {out['bag_info_txt']}")
        if out["summary_csv"]:
            print(f"  summary csv : {out['summary_csv']}")
        if out["readable_csv"]:
            print(f"  readable csv: {out['readable_csv']}")
        if out["bag_info_csv"]:
            print(f"  bag info csv: {out['bag_info_csv']}")
        if out["topics_csv"]:
            print(f"  topics csv  : {out['topics_csv']}")
        if out["traj_png"]:
            print(f"  traj png    : {out['traj_png']}")
        if out["dist_png"]:
            print(f"  dist png    : {out['dist_png']}")
        if out.get("dyn_rel_pngs"):
            for p in out["dyn_rel_pngs"]:
                print(f"  dyn-rel png : {p}")
        elif out["dyn_rel_png"]:
            print(f"  dyn-rel png : {out['dyn_rel_png']}")
        if not HAS_MPL:
            print("  png plots   : skipped (matplotlib not installed)")

        if args.save_csv and args.append_csv:
            append_csv(args.append_csv, out["summary"], list(out["summary"].keys()))
        if args.save_csv and args.append_bag_info_csv:
            append_csv(args.append_bag_info_csv, out["meta"], list(out["meta"].keys()))

    # Batch outputs
    if args.save_csv and all_summary:
        batch_summary_csv = os.path.join(args.output_dir, f"{stamp}_all_summary.csv")
        write_csv(batch_summary_csv, all_summary, list(all_summary[0].keys()))
        print(f"\nBatch summary csv: {batch_summary_csv}")
    if args.save_csv and all_readable:
        batch_readable_csv = os.path.join(args.output_dir, f"{stamp}_all_readable.csv")
        write_csv(batch_readable_csv, all_readable, list(all_readable[0].keys()))
        print(f"Batch readable csv: {batch_readable_csv}")
    if args.save_csv and all_meta:
        batch_meta_csv = os.path.join(args.output_dir, f"{stamp}_all_bag_info.csv")
        write_csv(batch_meta_csv, all_meta, list(all_meta[0].keys()))
        print(f"Batch bag-info csv: {batch_meta_csv}")


if __name__ == "__main__":
    main()
