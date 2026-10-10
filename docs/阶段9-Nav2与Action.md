# 阶段 9：Nav2 手动导航与 Action

## 1. 文档范围与配置基线

本文档记录 cakebot 在 **2026-10-09～2026-10-10** 完成阶段 9 时使用的 Nav2 配置与验收基线，适用于 ROS 2 Humble。实际运行参数的唯一真值来源是：

```text
src/cakebot_navigation/config/nav2_params.yaml
```

创建本基线时的运行版本与启动方式：

| 项目 | 当前基线 |
|---|---|
| ROS 2 发行版 | Humble |
| Navigation2 上游版本 | `1.1.20` |
| Debian 包版本 | `1.1.20-1jammy.20260908.*` |
| Nav2 上游启动文件 | `nav2_bringup/launch/navigation_launch.py` |
| Composition | `false`，各服务器作为独立进程，便于阶段 9 观察 |
| 默认时间源 | Gazebo `/clock`，即 `use_sim_time: true` |

本文档的参数表是学习和验收用快照。后续修改 YAML 时，应同步更新本文档的“配置变更记录”，并写明修改原因和实验结果。

阶段 9 的目标是：

- 用 MID-360 投影得到的统一 `/scan` 接口启动已知地图定位和 Nav2。
- 在 RViz 中完成导航、取消和更换目标。
- 理解 Action 的 goal、feedback、result 和 cancel。

本阶段不编写 C++ Action 客户端，不定制规划器/控制器，也不将原始 `PointCloud2` 直接接入代价地图。

## 2. 系统数据流与启动边界

```text
Gazebo MID-360
       │
       ▼
/lidar/points_raw (PointCloud2)
       │  pointcloud_to_laserscan
       ▼
     /scan (LaserScan)
       ├──► AMCL ──► map → odom
       └──► Nav2 global/local costmap

RViz Nav2 Goal
       │ NavigateToPose Action
       ▼
BT Navigator ─► Planner ─► Smoother ─► Controller
                                                  │
                                                  ▼
                                           cmd_vel_nav
                                                  │
                                           Velocity Smoother
                                                  │
                                                  ▼
                                              /cmd_vel
```

各 launch 文件的责任边界：

| Launch | 责任 | 明确不负责 |
|---|---|---|
| `gazebo.launch.py` | 仿真机器人、`/odom`、TF 和 MID-360 点云 | 定位、Nav2 |
| `lidar_adapter.launch.py` | `/lidar/points_raw → /scan` | AMCL、Nav2 |
| `localization.launch.py` | `map_server`、AMCL、定位 lifecycle manager | Gazebo、Nav2 导航节点 |
| `navigation.launch.py` | Nav2 导航节点、导航 lifecycle manager、导航 RViz | `map_server`、AMCL、SLAM |

这种分层使 AMCL 和 Nav2 只依赖标准 `/scan`，不需要知道它来自二维雷达还是 MID-360 适配器。

## 3. 当前选用的 Nav2 插件

| 所属服务器 | 配置名 | 插件 | 当前用途与选择理由 |
|---|---|---|---|
| Controller Server | `progress_checker` | `nav2_controller::SimpleProgressChecker` | 用指定时间内的位移量判断机器人是否卡住，逻辑简单，适合初次闭环 |
| Controller Server | `general_goal_checker` | `nav2_controller::SimpleGoalChecker` | 使用 XY 与 yaw 容差判断到达 |
| Controller Server | `FollowPath` | `dwb_core::DWBLocalPlanner` | Humble 成熟的差速底盘局部控制器，暂不引入自定义插件 |
| Global Costmap | `static_layer` | `nav2_costmap_2d::StaticLayer` | 读取已保存的 `/map` |
| Global/Local Costmap | `obstacle_layer` | `nav2_costmap_2d::ObstacleLayer` | 使用投影后的 `/scan` 标记与清除二维障碍 |
| Global/Local Costmap | `inflation_layer` | `nav2_costmap_2d::InflationLayer` | 在障碍周围建立连续代价梯度 |
| Planner Server | `GridBased` | `nav2_navfn_planner/NavfnPlanner` | 在已知二维地图上产生全局路径 |
| Smoother Server | `simple_smoother` | `nav2_smoother::SimpleSmoother` | 在控制之前平滑全局路径 |
| Behavior Server | `spin` | `nav2_behaviors/Spin` | 原地旋转恢复 |
| Behavior Server | `backup` | `nav2_behaviors/BackUp` | 后退恢复 |
| Behavior Server | `drive_on_heading` | `nav2_behaviors/DriveOnHeading` | 按给定朝向直行 |
| Behavior Server | `assisted_teleop` | `nav2_behaviors/AssistedTeleop` | 保留 Humble 默认的辅助遥控行为，阶段 9 不主动使用 |
| Behavior Server | `wait` | `nav2_behaviors/Wait` | 等待恢复 |
| Waypoint Follower | `wait_at_waypoint` | `nav2_waypoint_follower::WaitAtWaypoint` | 满足 Humble navigation bringup 的 waypoint follower 配置，当前等待时间为 0 |

`bt_navigator.plugin_lib_names` 保留 ROS 2 Humble 默认的完整 BT 节点库，包含路径计算、路径跟随、旋转/后退/等待恢复、代价地图清理、目标更新、计时/距离条件和 cancel 节点。具体动态库名以 `nav2_params.yaml` 的 `plugin_lib_names` 列表为准。阶段 9 不自定义 BT XML，使用 Nav2 默认的带重规划与恢复的 `NavigateToPose` 行为树。

## 4. 当前关键参数快照

### 4.1 坐标系、时间和更新频率

| 参数 | 当前值 | 含义 |
|---|---:|---|
| `bt_navigator.global_frame` | `map` | 导航目标和全局任务坐标系 |
| `robot_base_frame` | `base_link` | cakebot 当前无 `base_footprint` |
| `odom_topic` | `/odom` | Gazebo 差速插件提供的里程计 |
| `use_sim_time` | `true` | 当前使用 Gazebo 时钟；launch 可覆盖 |
| Controller 频率 | `20 Hz` | DWB 控制循环 |
| Local costmap 更新/发布 | `5/2 Hz` | 近场障碍更新与 RViz 发布频率 |
| Global costmap 更新/发布 | `1/1 Hz` | 全局代价地图频率 |
| Behavior Server 频率 | `10 Hz` | 恢复行为执行频率 |
| Velocity Smoother 频率 | `20 Hz` | 速度平滑输出频率 |

### 4.2 机器人几何与代价地图

| 参数 | 当前值 | 选择依据 |
|---|---:|---|
| `footprint` | `(±0.16, ±0.125) m` 矩形 | 底盘长 `0.32 m`，车轮外缘总宽约 `0.245 m` |
| `footprint_padding` | `0.01 m` | 初始额外几何余量 |
| Costmap resolution | `0.05 m` | 与当前 SLAM 地图分辨率一致 |
| Local window | `3 × 3 m` | 低速阶段的近场规划窗口 |
| `inflation_radius` | `0.30 m` | 初始保守障碍代价范围，不是最终真机安全距离 |
| `cost_scaling_factor` | `5.0` | 控制障碍周围代价随距离衰减的速度 |
| Global plugins | static + obstacle + inflation | 同时使用已知地图和实时 `/scan` |
| Local plugins | obstacle + inflation | 局部滚动窗口不复制静态地图层 |
| `track_unknown_space` | `true` | 在全局代价地图保留未知区分类 |

`footprint` 是机器人的碰撞投影；`inflation_radius` 是规划代价范围。两者不应通过重复放大来表达同一份安全余量。

### 4.3 `/scan` 障碍源

| 参数 | 当前值 |
|---|---:|
| Topic / type | `/scan` / `LaserScan` |
| `marking` / `clearing` | `true` / `true` |
| `obstacle_min_range` | `0.12 m` |
| `obstacle_max_range` | `7.5 m` |
| `raytrace_min_range` | `0.12 m` |
| `raytrace_max_range` | `8.0 m` |
| `max_obstacle_height` | `2.0 m` |
| `inf_is_valid` | `false` |

当前 `inf_is_valid: false` 表示代价地图不把 `+inf` 量程当作有效的最远清除射线。而 MID-360 适配器当前可以对无回波方向输出 `+inf`。阶段 9 先保留此配置；若实验出现障碍消失后代价地图长时间残留“鬼影”，应优先对比测试 `inf_is_valid: true`，不应在没有现象和对照实验时盲目修改。

### 4.4 路径规划和平滑

| 参数 | 当前值 | 含义 |
|---|---:|---|
| Planner | NavFn | 栅格全局规划 |
| `tolerance` | `0.10 m` | 目标附近可接受的规划容差 |
| `use_astar` | `false` | 使用 NavFn 默认的 Dijkstra 搜索 |
| `allow_unknown` | `false` | 阶段 9 是已知地图导航，禁止路径穿过未知区 |
| Smoother | SimpleSmoother | 平滑全局路径 |
| `max_its` | `1000` | 最大迭代次数 |
| `do_refinement` | `true` | 对平滑结果继续精化 |

`allow_unknown: false` 是本阶段的明确选择。进入在线 SLAM 与前沿探索阶段时，必须根据探索逻辑重新评估，否则机器人无法将路径规划到当前地图的未知边界。

### 4.5 DWB 控制、进度与到达条件

| 参数 | 当前值 |
|---|---:|
| `max_vel_x` / `max_speed_xy` | `0.15 m/s` |
| `max_vel_theta` | `0.60 rad/s` |
| `acc_lim_x` / `decel_lim_x` | `0.10 / -0.10 m/s²` |
| `acc_lim_theta` / `decel_lim_theta` | `0.80 / -0.80 rad/s²` |
| `min_vel_y` / `max_vel_y` | `0 / 0` |
| `vx_samples` / `vtheta_samples` | `20 / 20` |
| `vy_samples` | `1` |
| `sim_time` | `1.7 s` |
| `linear_granularity` | `0.05 m` |
| `angular_granularity` | `0.025 rad` |
| `transform_tolerance` | `0.2 s` |
| `trans_stopped_velocity` | `0.02 m/s` |
| Goal XY tolerance | `0.12 m` |
| Goal yaw tolerance | `0.15 rad` |
| Progress movement radius | `0.05 m` |
| Progress time allowance | `10 s` |

线加速度 `0.10 m/s²` 与当前 Gazebo 差速插件一致：轮子最大角加速度为 `2 rad/s²`，轮半径为 `0.05 m`，直行时对应 `2 × 0.05 = 0.10 m/s²`。当前速度值用于低速闭环验证，不是 cakebot 最终性能上限。

DWB 当前启用的 critics 为：

```text
RotateToGoal, Oscillation, BaseObstacle, GoalAlign,
PathAlign, PathDist, GoalDist
```

它们分别对目标朝向、往复振荡、障碍碰撞、目标/路径对齐和目标/路径距离进行评价。阶段 9 只要求理解它们会对候选速度轨迹打分，不要在没有重复测试场景时调整权重。

### 4.6 Behavior Server 与 Velocity Smoother

| 参数 | 当前值 |
|---|---:|
| Behavior `max_rotational_vel` | `0.60 rad/s` |
| Behavior `min_rotational_vel` | `0.10 rad/s` |
| Behavior `rotational_acc_lim` | `0.80 rad/s²` |
| Velocity smoother `feedback` | `OPEN_LOOP` |
| Velocity smoother max velocity | `[0.15, 0.0, 0.60]` |
| Velocity smoother min velocity | `[-0.15, 0.0, -0.60]` |
| Velocity smoother max accel | `[0.10, 0.0, 0.80]` |
| Velocity smoother max decel | `[-0.10, 0.0, -0.80]` |
| Velocity smoother timeout | `1.0 s` |

Navigation bringup 将 Controller Server 的 `cmd_vel` 重映射为 `cmd_vel_nav`，再由 Velocity Smoother 输出最终 `/cmd_vel`。Nav2 Humble 上游启动文件没有把 Behavior Server 的恢复速度重映射到 `cmd_vel_nav`，因此旋转、后退等恢复动作可能直接发布 `/cmd_vel`。这也是 DWB、Behavior Server 和 Velocity Smoother 的速度/加速度上限需要保持一致的原因；阶段 15 引入独立安全链时，还必须确保这些速度来源最终都经过安全层。

## 5. MID-360 模式启动顺序

每个终端都应先执行：

```bash
source /opt/ros/humble/setup.bash
source /home/wen/cakebot/install/setup.bash
```

### 终端 1：Gazebo 和 MID-360 仿真

```bash
ros2 launch cakebot_description gazebo.launch.py \
  world:=/home/wen/cakebot/src/cakebot_description/worlds/test_env.world \
  sensor_mode:=mid360_sim \
  visualize_lidar:=false \
  use_rviz:=false
```

### 终端 2：点云转换为统一 `/scan`

```bash
ros2 launch cakebot_perception lidar_adapter.launch.py \
  use_sim_time:=true
```

### 终端 3：已知地图定位

```bash
ros2 launch cakebot_navigation localization.launch.py \
  map:=/home/wen/cakebot/maps/mid360_runs/test_env_mid360.yaml \
  use_sim_time:=true \
  use_rviz:=false \
  set_initial_pose:=true \
  initial_x:=0.0 \
  initial_y:=0.0 \
  initial_z:=0.0 \
  initial_yaw:=0.0
```

这些位姿是当前 `test_env` 中机器人从世界原点、yaw 为 0 出生时的重复实验基线。`initial_x`、`initial_y` 是保存地图的 `map` 坐标，`initial_yaw` 的单位是 rad；更换地图或 Gazebo 出生位姿后不能盲目沿用。

在启动 Nav2 前，新开一个诊断终端确认 AMCL 已经持续提供完整 TF：

```bash
ros2 run tf2_ros tf2_echo map base_link
```

必须持续输出有效的 translation/rotation，不能出现 `Invalid frame ID "map"`。这一门禁避免 Controller 在 `map → odom` 尚未建立时激活。

### 终端 4：Nav2 导航与唯一 RViz

```bash
ros2 launch cakebot_navigation navigation.launch.py \
  use_sim_time:=true
```

启动后等待 Controller、Planner 和 BT Navigator 都进入 `active`。RViz 现在主要用于观察地图、路径、costmap 和 footprint，不再必须用鼠标设置初始位姿或目标。

## 6. Action 学习要点

Nav2 的 `/navigate_to_pose` 是长时间任务接口：

- **goal**：目标 `PoseStamped`，包含坐标系、位置和朝向；可选指定行为树。
- **feedback**：执行中的当前位姿、导航时间、预计剩余时间、恢复次数和剩余距离。
- **result**：任务结束时返回。Humble 的 `NavigateToPose` result 负载是空消息，应主要通过 Action 终止状态区分 `SUCCEEDED`、`CANCELED` 和 `ABORTED`。
- **cancel**：客户端请求取消某个 goal；它不会关闭 Nav2 节点。
- **更换目标**：发送一个新 goal，由 Nav2 中止/抢占旧 goal 并重新规划；不是修改原 goal 的消息内容。

查看接口：

```bash
ros2 action list -t
ros2 action info /navigate_to_pose
ros2 interface show nav2_msgs/action/NavigateToPose
```

RViz 可以继续作为 Action 客户端，但为了固定目标坐标并直接看到 feedback，本阶段优先使用 `ros2 action send_goal ... --feedback`。这是现成命令行 Action 客户端，不需要提前编写阶段 10 的 C++ 客户端。

当前 `test_env` 的第一个命令行目标可使用 `(x=1.0, y=0.0, yaw=0)`：

```bash
ros2 action send_goal \
  /navigate_to_pose \
  nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}" \
  --feedback
```

平面 yaw 转换为四元数时：

```text
orientation.z = sin(yaw / 2)
orientation.w = cos(yaw / 2)
```

| yaw | `orientation.z` | `orientation.w` |
|---:|---:|---:|
| `0` | `0.0` | `1.0` |
| `π/2` | `0.7071` | `0.7071` |
| `π` | `1.0` | `0.0` |
| `-π/2` | `-0.7071` | `0.7071` |

Humble 的 `ros2 action` 没有独立的便捷 cancel 子命令。阶段 9 如果希望完全不点击 RViz，可在另一终端通过 Action 的 cancel service 取消当前所有 `/navigate_to_pose` goal：

```bash
ros2 service call \
  /navigate_to_pose/_action/cancel_goal \
  action_msgs/srv/CancelGoal \
  "{goal_info: {goal_id: {uuid: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]}, stamp: {sec: 0, nanosec: 0}}}"
```

全零 goal ID 和全零时间戳的语义是“取消所有活动 goal”。这适合本阶段的命令行实验；阶段 10 应由 C++ Action 客户端保存 goal handle 并精确取消指定 goal。

## 7. 阶段 9 验收记录

### 7.1 启动与可观测性

```bash
ros2 action list -t
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
ros2 lifecycle get /bt_navigator
ros2 topic info /scan --verbose
ros2 run tf2_ros tf2_echo map base_link
```

通过条件：

- `/navigate_to_pose` 存在且类型为 `nav2_msgs/action/NavigateToPose`。
- Controller、Planner 和 BT Navigator 均为 `active`。
- `/scan` 只有预期的一个发布者。
- `map → odom → base_link → laser_link` 连通。
- RViz 中能看到与机器人对齐的 footprint、全局/局部代价地图和路径。

### 7.2 手动 Action 用例

| 用例 | 操作 | 通过条件 | 实验结果 |
|---|---|---|---|
| 正常导航 | 用 `ros2 action send_goal --feedback` 向需要绕过障碍的可达位置发送 goal | 无碰撞，终态 `SUCCEEDED`，误差满足 goal checker | 已通过 |
| 取消 | 机器人运动后调用 cancel service | 终态 `CANCELED`，`/cmd_vel` 回零，不继续向原目标运动 | 已通过 |
| 更换目标 | 终端 A 的 goal 执行时，在终端 B 发送新 goal | 放弃 A，重新规划并到达 B | 已通过 |
| 不可达目标 | 向墙内或不可达区域发送 goal | 不无限运动，经有限恢复后以 `ABORTED` 结束 | 已通过 |

### 7.3 学习验收

完成阶段 9 时，应能脱离文档解释：

1. Topic、Service 和 Action 各适合什么类型的任务。
2. `NavigateToPose` 的 goal、feedback、result 和 cancel 各自的作用。
3. 为什么更换目标是新 goal 抢占旧 goal。
4. Planner、Smoother、Controller、Costmap 和 BT Navigator 在导航闭环中的基本边界。
5. 为什么 footprint 不能用 inflation 代替。
6. 为什么 AMCL 和 Nav2 可以在不修改的情况下共用 MID-360 投影得到的 `/scan`。

## 8. 配置变更记录

| 日期 | 变更 | 原因 | 对照场景与结果 |
|---|---|---|---|
| 2026-10-09 | 记录阶段 9 初始基线 | 在手动导航实验前固定插件、几何和运动限制 | MID-360 模式的正常、取消、换目标和不可达目标测试已通过 |
| 2026-10-09 | `localization.launch.py` 增加参数化 AMCL 初始位姿 | 避免 RViz 手动点选，固定重复实验的起始条件，并确保 Nav2 在 `map → odom` 建立后才启动 | 已完成阶段 9 导航回归 |
| 2026-10-10 | 完成阶段 9 | 确认 Nav2 导航闭环和 Action 的成功、取消、目标替换与失败路径 | 阶段 9 验收完成，下一阶段为 C++ Action 客户端 |
