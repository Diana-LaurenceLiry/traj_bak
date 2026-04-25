#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from gazebo_msgs.srv import DeleteModel, GetWorldProperties


def parse_csv_param(name, default_csv):
    raw = rospy.get_param(name, default_csv)
    if isinstance(raw, list):
        vals = [str(x).strip() for x in raw]
    else:
        vals = [x.strip() for x in str(raw).split(",")]
    return [v for v in vals if v]


def should_delete(model_name, prefixes, contains):
    for p in prefixes:
        if model_name.startswith(p):
            return True
    for s in contains:
        if s in model_name:
            return True
    return False


def main():
    rospy.init_node("cleanup_gazebo_models")

    prefixes = parse_csv_param("~prefixes", "dyn_obs")
    contains = parse_csv_param("~contains", "")
    ignore = set(parse_csv_param("~ignore", "ground_plane,sun"))
    service_timeout = float(rospy.get_param("~service_timeout", 5.0))
    dry_run = bool(rospy.get_param("~dry_run", False))

    rospy.loginfo(
        "cleanup_gazebo_models start, prefixes=%s contains=%s dry_run=%s",
        prefixes,
        contains,
        str(dry_run),
    )

    try:
        rospy.wait_for_service("/gazebo/get_world_properties", timeout=service_timeout)
        rospy.wait_for_service("/gazebo/delete_model", timeout=service_timeout)
    except rospy.ROSException:
        rospy.logwarn("cleanup_gazebo_models: gazebo services unavailable, skip cleanup")
        return

    get_world = rospy.ServiceProxy("/gazebo/get_world_properties", GetWorldProperties)
    delete_model = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)

    try:
        resp = get_world()
    except rospy.ServiceException as e:
        rospy.logwarn("cleanup_gazebo_models: get_world_properties failed: %s", str(e))
        return

    deleted = 0
    for name in resp.model_names:
        if name in ignore:
            continue
        if not should_delete(name, prefixes, contains):
            continue
        if dry_run:
            rospy.loginfo("cleanup_gazebo_models dry-run delete: %s", name)
            deleted += 1
            continue
        try:
            delete_model(name)
            rospy.loginfo("cleanup_gazebo_models deleted: %s", name)
            deleted += 1
        except rospy.ServiceException as e:
            rospy.logwarn("cleanup_gazebo_models failed for %s: %s", name, str(e))

    rospy.loginfo("cleanup_gazebo_models done, deleted=%d", deleted)


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
