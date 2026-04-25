# my_gazebo_sim

这个包用于在 Gazebo 中运行差速小车仿真，并提供两条感知到地图的链路：
- 深度相机直出点云（`/depth_camera/depth/points`）
- 双目图像经 `stereo_image_proc` 生成点云（`/points2`）

当前默认启动链路是深度相机版本。

## 快速开始（直接复制运行）

### 0) 每个终端先执行（避免 conda 干扰 ROS Python）
```bash
conda deactivate
source /opt/ros/noetic/setup.bash
source /home/lry/catkin_ws/devel/setup.bash
```

### 1) A* 导航（当前推荐）
终端1：
```bash
conda deactivate
source /opt/ros/noetic/setup.bash
source /home/lry/catkin_ws/devel/setup.bash
roslaunch my_gazebo_sim astar_navigation.launch
```
终端2（可选，查看状态）：
```bash
conda deactivate
source /opt/ros/noetic/setup.bash
source /home/lry/catkin_ws/devel/setup.bash
rostopic echo -n 1 /global_path
```
使用方法：
- 在 RViz 使用 `2D Nav Goal` 点目标点。
- 小车会沿 `/global_path` 运动。

### 2) 老版 waypoint + 避障
终端1：
```bash
conda deactivate
source /opt/ros/noetic/setup.bash
source /home/lry/catkin_ws/devel/setup.bash
roslaunch my_gazebo_sim gazebo_waypoint_nav_obstacle.launch frame_id:=odom
```
终端2：
```bash
conda deactivate
source /opt/ros/noetic/setup.bash
source /home/lry/catkin_ws/devel/setup.bash
roslaunch my_gazebo_sim map_builder.launch cloud_topic:=/depth_camera/depth/points
```

### 3) 老版 waypoint（不启用避障）
```bash
conda deactivate
source /opt/ros/noetic/setup.bash
source /home/lry/catkin_ws/devel/setup.bash
roslaunch my_gazebo_sim gazebo_waypoint_nav_obstacle.launch use_obstacle_avoidance:=false frame_id:=odom
```

## launch 文件说明

### `launch/simple_diffbot_gazebo.launch`
- 功能：启动 Gazebo 世界、加载机器人模型并在仿真中 spawn 出来。
- 关键参数：
  - `world_name`：Gazebo 世界文件。
  - `robot_urdf`：要加载的 URDF 文件路径（默认是 `simple_diffbot_depth.urdf`）。
- 典型用途：所有上层导航/建图 launch 的底座启动文件。

### `launch/map_builder.launch`
- 功能：启动 `scripts/map_builder.py`，把点云转换成 `OccupancyGrid`（`/occupancy_map`）。
- 关键参数：
  - `cloud_topic`：输入点云话题（默认 `/camera/depth/points`，Gazebo 深度相机时改为 `/depth_camera/depth/points`）。
  - `map_topic`、`map_frame`、分辨率和膨胀参数。
- 典型用途：给避障和导航提供可直接消费的栅格障碍图。

### `launch/stereo_proc.launch`
- 功能：启动 `stereo_image_proc`，将左右相机图像+相机内参处理为双目结果（含点云）。
- 关键参数：
  - `left_image_topic`、`left_camera_info_topic`
  - `right_image_topic`、`right_camera_info_topic`
  - `points2_topic`（默认可设成 `/camera/depth/points`）
- 典型用途：统一双目输入接口，便于仿真和实机共用。

### `launch/sim_to_d435i_bridge.launch`
- 功能：把 Gazebo 双目话题转成 D435i 风格话题命名。
- 默认映射：
  - `/left_camera/image_raw` -> `/camera/infra1/image_rect_raw`
  - `/left_camera/camera_info` -> `/camera/infra1/camera_info`
  - `/right_camera/image_raw` -> `/camera/infra2/image_rect_raw`
  - `/right_camera/camera_info` -> `/camera/infra2/camera_info`
- 关键参数：
  - `use_imu`（默认 `false`）：是否同时桥接 IMU 到 `/camera/imu`。
- 典型用途：先在仿真中对齐实机接口，后续换 bag/实机尽量不改代码。

### `launch/perception_pipeline.launch`
- 功能：统一感知入口（可选 sim 话题桥 + stereo 处理 + `map_builder`）。
- 关键参数：
  - `use_sim_bridge`（默认 `true`）：是否启用“仿真话题 -> D435i 风格话题”的桥接层。
    - `true`：输入来自 Gazebo（如 `/left_camera/...`、`/right_camera/...`）时使用。
    - `false`：输入本身已经是 D435i 风格（如 `/camera/infra1/...`、`/camera/infra2/...`）时使用。
  - `use_imu`（默认 `false`）：预留 IMU 接口，不强依赖。
  - `points_topic`（默认 `/camera/depth/points`）：下游统一点云话题。
- 典型用途：构建“当前无 IMU、后续可加 IMU”的稳定接口层。

### `use_sim_bridge` 快速理解
- 它只负责“改话题名/转接话题”，不做算法处理。
- 开了以后：会启动 `sim_to_d435i_bridge.launch`，把 Gazebo 的左右目话题 relay 到 D435i 风格命名。
- 关了以后：直接使用你当前已有的话题，不再额外 relay。

### `launch/gazebo_waypoint_nav.launch`
- 功能：启动 Gazebo + waypoint 控制器（`ORB_SLAM3/waypoint_controller`）+ 可选 RViz。
- 关键参数：
  - `gazebo_launch`：底层 Gazebo 启动文件（默认 `simple_diffbot_gazebo.launch`）。
  - `robot_urdf`：透传给 Gazebo 启动文件，方便切换深度/双目模型。
  - `use_obstacle_avoidance`（默认 `false`）：是否启用避障层（`cmd_vel_raw` -> `cmd_vel`）。
  - `map_topic`：避障层读取的占据栅格话题（默认 `/occupancy_map`）。
  - `waypoints_yaml`：航点配置文件。
- 典型用途：空场景 waypoint 巡航测试。

### `launch/gazebo_waypoint_nav_obstacle.launch`
- 功能：与上一个类似，但默认加载 3D 障碍物世界 `worlds/nav_obstacle_3d.world`。
- 关键参数：
  - `world_name`：障碍物场景。
  - `robot_urdf`：可切换传感器模型。
  - `use_obstacle_avoidance`（默认 `true`）：默认开启避障层。
  - `use_3d_obstacle_avoidance`（默认 `true`）：启用基于点云的 3D 避障（用于无人机场景）。
  - `oa3d_cloud_topic`（默认 `/depth_camera/depth/points`）：3D 避障点云输入。
  - `oa3d_*`：3D 避障参数（前视距离、横向/垂向窗口、转向与升降速度等）。
  - 避障调参（`oa_*`）：
    - `oa_forward_distance`：前方探测距离（越大越早触发）。
    - `oa_critical_distance`：进入强制减速/停走阈值。
    - `oa_slowdown_distance`：开始减速阈值。
    - `oa_corridor_half_width`：前方走廊半宽（越大越“保守”）。
    - `oa_min_turn_rate`、`oa_max_turn_rate`：避障转向强度。
- 典型用途：障碍环境下 waypoint 测试与后续避障算法联调。

### `launch/astar_navigation.launch`
- 功能：一键启动 A* 导航最小链路（Gazebo + map_builder + A* 全局路径 + 路径跟踪）。
- 输入：
  - 地图：`/occupancy_map`
  - 当前位姿：`/odom`
  - 目标点：`/move_base_simple/goal`（RViz 的 `2D Nav Goal`）
- 输出：
  - 全局路径：`/global_path`
  - 速度：`/cmd_vel`（中间为 `/cmd_vel_raw`）
- 典型用途：替代简单 waypoint 控制，验证“栅格地图 + A* + 跟踪控制”。

### `launch/real_navigation.launch`
- 功能：不启动 Gazebo，直接用实机/rosbag 的双目数据做建图+A*导航。
- 输入：
  - `/camera/infra1/image_rect_raw`
  - `/camera/infra1/camera_info`
  - `/camera/infra2/image_rect_raw`
  - `/camera/infra2/camera_info`
  - `/odom`（必须外部提供）
- 输出：
  - `/occupancy_map`
  - `/global_path`
  - `/cmd_vel`
- 注意：如果没有 `/odom`，规划和跟踪无法正常工作。

## urdf 文件说明

### `urdf/simple_diffbot.urdf`
- 功能：基础差速小车模型（车体、轮子、差速驱动插件）。
- 特点：最简模型，可作为结构参考。

### `urdf/simple_diffbot_depth.urdf`
- 功能：在小车上挂载深度相机插件（`libgazebo_ros_openni_kinect.so`）。
- 输出：
  - 深度图：`/depth_camera/depth/image_raw`
  - 点云：`/depth_camera/depth/points`
- 典型用途：默认链路，快速得到点云并建图。

### `urdf/simple_diffbot_stereo.urdf`
- 功能：挂载左右单目相机（`left_camera` / `right_camera`），用于双目处理。
- 特点：双目基线参数存在，但历史版本可能存在左右配置不够严谨的问题。
- 典型用途：双目流程原始版本参考。

### `urdf/simple_diffbot_stereo_fixed.urdf`
- 功能：双目模型修正版（用于替代 `simple_diffbot_stereo.urdf`）。
- 特点：左右相机 `hackBaseline` 参数更合理，建议优先使用。
- 典型用途：双目链路正式测试版本。

### `urdf/simple_diffbot_depth.urdf.bak_20260326`
- 功能：`simple_diffbot_depth.urdf` 的备份文件。
- 典型用途：历史回滚/对比，不建议直接作为运行入口。

## 其他启动组合

### 1) 深度相机链路（默认，最稳）
```bash
conda deactivate
source /opt/ros/noetic/setup.bash
source /home/lry/catkin_ws/devel/setup.bash
roslaunch my_gazebo_sim gazebo_waypoint_nav_obstacle.launch
roslaunch my_gazebo_sim map_builder.launch cloud_topic:=/depth_camera/depth/points
```

### 2) 双目链路（双目图像 -> `/points2` -> 栅格图）
```bash
conda deactivate
source /opt/ros/noetic/setup.bash
source /home/lry/catkin_ws/devel/setup.bash
roslaunch my_gazebo_sim gazebo_waypoint_nav_obstacle.launch \
  robot_urdf:=/home/lry/catkin_ws/src/my_gazebo_sim/urdf/simple_diffbot_stereo_fixed.urdf
roslaunch my_gazebo_sim perception_pipeline.launch use_sim_bridge:=true use_imu:=false
```

### 3) 回放 D435i bag（纯双目，不加 IMU）
```bash
conda deactivate
source /opt/ros/noetic/setup.bash
source /home/lry/catkin_ws/devel/setup.bash
roslaunch my_gazebo_sim perception_pipeline.launch use_sim_bridge:=false use_imu:=false
```

### 4) 后续切到“带 IMU”时
```bash
conda deactivate
source /opt/ros/noetic/setup.bash
source /home/lry/catkin_ws/devel/setup.bash
roslaunch my_gazebo_sim perception_pipeline.launch use_sim_bridge:=false use_imu:=true
```

### 5) 关闭避障（只用 waypoint 原始输出）
```bash
conda deactivate
source /opt/ros/noetic/setup.bash
source /home/lry/catkin_ws/devel/setup.bash
roslaunch my_gazebo_sim gazebo_waypoint_nav_obstacle.launch use_obstacle_avoidance:=false
```

### 6) 避障更明显（推荐调参示例）
```bash
conda deactivate
source /opt/ros/noetic/setup.bash
source /home/lry/catkin_ws/devel/setup.bash
roslaunch my_gazebo_sim gazebo_waypoint_nav_obstacle.launch \
  oa_forward_distance:=1.6 \
  oa_critical_distance:=0.55 \
  oa_slowdown_distance:=1.30 \
  oa_corridor_half_width:=0.45 \
  oa_min_turn_rate:=0.65 \
  oa_max_turn_rate:=1.50
```

### 7) A* 全局规划 + 路径跟踪（推荐下一阶段）
```bash
conda deactivate
source /opt/ros/noetic/setup.bash
source /home/lry/catkin_ws/devel/setup.bash
roslaunch my_gazebo_sim astar_navigation.launch
```
说明：
- 打开 RViz 后，使用 `2D Nav Goal` 在地图上点目标点。
- 节点会在 `/global_path` 发布路径，`path_follower` 跟踪路径并输出速度。

### 8) 接入实机/rosbag（不跑 Gazebo）
终端1：回放 bag
```bash
conda deactivate
source /opt/ros/noetic/setup.bash
source /home/lry/catkin_ws/devel/setup.bash
rosbag play /home/lry/bags/260307/main_3min_infra_gyro_accel_260307.bag --clock
```
终端2：导航主链路
```bash
conda deactivate
source /opt/ros/noetic/setup.bash
source /home/lry/catkin_ws/devel/setup.bash
roslaunch my_gazebo_sim real_navigation.launch
```
终端3：里程计来源（必须）
```bash
# 这里启动你的 VO/SLAM 节点（例如 ORB_SLAM3 Stereo）
# 目标是提供 /odom (以及对应 TF)
```
检查项：
```bash
rostopic echo -n 1 /camera/depth/points
rostopic echo -n 1 /occupancy_map
rostopic echo -n 1 /odom
rostopic echo -n 1 /global_path
```

### 9) 无人机本体最小版（第一步迁移）
```bash
conda deactivate
source /opt/ros/noetic/setup.bash
source /home/lry/catkin_ws/devel/setup.bash
roslaunch my_gazebo_sim uav_astar_navigation.launch
```
说明：
- 该入口会启动：
  - `simple_uav_depth.urdf`（无人机外形 + 深度相机）
  - `uav_cmdvel_to_gazebo.py`（把 `cmd_vel` 转成 Gazebo 模型三维速度）
  - `map_builder` + `astar_global_planner` + `path_follower` + 可选 `obstacle_avoidance_3d`
- 这是“无人机软件链路迁移”的最小可运行版，控制层是运动学近似，不是完整四旋翼动力学/飞控仿真。

## 相关脚本

### `scripts/map_builder.py`
- 功能：点云转占据栅格图（含高度过滤、距离过滤、膨胀）。
- 输出：`/occupancy_map`（`nav_msgs/OccupancyGrid`）。

### `scripts/obstacle_avoidance.py`
- 功能：局部避障层。订阅 `cmd_vel_raw` + `/occupancy_map` + `/odom`，输出最终 `cmd_vel`。
- 策略：前方走廊碰撞检测 + 线速度衰减 + 左右避障转向偏置。
- 说明：这是最小可用版，便于你后续替换成更复杂算法（DWA/TEB/自研策略）。

### `scripts/obstacle_avoidance_3d.py`
- 功能：3D 避障层。订阅 `cmd_vel_raw` + 点云（`PointCloud2`），输出最终 `cmd_vel`。
- 策略：前向体素窗口检测 + 速度衰减 + 左右绕障 + 上下高度避让（`linear.z`）。
- 适用：无人机/3D 场景优先。

### `scripts/astar_global_planner.py`
- 功能：基于 `/occupancy_map` 执行 A*，从当前里程计位置规划到 RViz 目标点。
- 输出：`/global_path`（`nav_msgs/Path`）。

### `scripts/path_follower.py`
- 功能：跟踪 `/global_path` 并输出 `cmd_vel_raw`。
- 策略：简化 pure-pursuit 风格控制（前视点 + 角度误差控制）。

### `scripts/uav_cmdvel_to_gazebo.py`
- 功能：把 `cmd_vel` 映射到 Gazebo 模型速度（x/y/z/yaw），用于最小无人机控制桥。
- 说明：用于第一阶段快速验证导航链路，后续可替换为 PX4/ArduPilot 或更真实动力学控制器。

## 后续建议（避障前）
- 确认你的避障节点订阅的是 `/occupancy_map` 还是局部代价地图。
- 如果用于局部避障，建议后续把地图从“每帧重建”改成“短时融合/滑窗融合”，稳定性会更好。
