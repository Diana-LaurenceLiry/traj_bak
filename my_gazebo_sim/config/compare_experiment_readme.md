# Compare Experiment Runner (A/B/C/D)

This project provides four launch files with unified waypoints and dynamic-obstacle settings:

- Group A (global only): `launch/experiments/group_a_global_only.launch`
- Group B (global + rule avoidance): `launch/experiments/group_b_global_plus_rule.launch`
- Group C (EGO only): `launch/experiments/group_c_ego_only.launch`
- Group D (global + EGO, target method): `launch/experiments/group_d_global_plus_ego.launch`

Unified route file:

- `config/compare_route.yaml`

## Start commands

```bash
conda deactivate
source /opt/ros/noetic/setup.bash
source /home/lry/catkin_ws/devel/setup.bash
```

```bash
# Group A
roslaunch my_gazebo_sim experiments/group_a_global_only.launch

# Group B
roslaunch my_gazebo_sim experiments/group_b_global_plus_rule.launch

# Group C
roslaunch my_gazebo_sim experiments/group_c_ego_only.launch

# Group D
roslaunch my_gazebo_sim experiments/group_d_global_plus_ego.launch
```

## Recommended recording

Run in another terminal after launch:

```bash
source /opt/ros/noetic/setup.bash
source /home/lry/catkin_ws/devel/setup.bash
rosbag record -O groupX_runYY.bag \
/move_base_simple/goal \
/planner/global_path \
/planner/ego_goal \
/visual_slam/odom_safe_filtered \
/odom \
/planning/pos_cmd \
/gazebo/model_states \
/rosout
```

## Suggested metrics

- Mission success rate (all waypoints reached within run timeout)
- Collision count / collision rate
- Total completion time
- Total traveled path length
- Minimum distance to dynamic obstacles

