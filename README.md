# cakebot ROS 2 仿真、建图与定位说明

本项目用于学习 ROS 2 Humble、Gazebo Classic、TF、激光雷达、SLAM 和 Nav2。当前已经完成机器人仿真、差速驱动、里程计、激光雷达、人工遥控、在线建图、地图保存和 AMCL 定位；阶段 8.5 已建立 MID-360 三维点云到现有二维建图/定位链路的感知适配层，下一步进入阶段 9 的 Nav2 导航闭环。

## 1. 环境与编译

以下命令默认工作空间位于 `/home/wen/cakebot`，ROS 2 发行版为 Humble。

首次使用时确认建图和定位依赖已安装：

```bash
sudo apt install \
  ros-humble-slam-toolbox \
  ros-humble-nav2-bringup \
  ros-humble-nav2-amcl \
  ros-humble-nav2-map-server \
  ros-humble-nav2-lifecycle-manager \
  ros-humble-nav2-rviz-plugins \
  ros-humble-pointcloud-to-laserscan
```

编译整个工作空间：

```bash
cd /home/wen/cakebot
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

后续每打开一个新终端，都需要加载 ROS 2 和当前工作空间：

```bash
source /opt/ros/humble/setup.bash
source /home/wen/cakebot/install/setup.bash
```

只编译本项目的四个包：

```bash
cd /home/wen/cakebot
source /opt/ros/humble/setup.bash
colcon build \
  --packages-select cakebot_description cakebot_demo_cpp cakebot_navigation cakebot_perception \
  --symlink-install
```

## 2. 项目组成与核心接口

| 包 | 作用 |
|---|---|
| `cakebot_description` | Xacro/URDF、Gazebo 世界、launch、RViz、SLAM 参数和地图自动保存脚本 |
| `cakebot_demo_cpp` | 基础运动测试和键盘遥控节点 |
| `cakebot_navigation` | AMCL 参数、静态地图定位 launch、定位 RViz、AMCL 估计轨迹节点和定位冒烟测试脚本 |
| `cakebot_perception` | MID-360 点云接口、`PointCloud2 → LaserScan` 适配参数和启动文件 |

两个 Gazebo 世界的用途不同：

| 世界文件 | 用途 |
|---|---|
| `cakebot.world` | 只有地面和太阳，适合基础模型与运动测试 |
| `test_env.world` | 包含墙体和障碍物，用于雷达验证及 SLAM 建图 |

`test_env.world` 是仿真环境，不是 Nav2 使用的地图。SLAM 生成的地图是 `.yaml` 与 `.pgm` 文件。

当前地图按感知来源分目录保存，避免二维雷达地图和 MID-360 投影地图互相覆盖：

| 目录 | 内容 |
|---|---|
| `maps/map-2d/` | 原二维雷达流程生成的地图，例如 `test_env.yaml` 和 `test_env.pgm` |
| `maps/mid360_runs/` | MID-360 点云投影流程生成的地图，例如 `test_env_mid360.yaml` 和 `test_env_mid360.pgm` |

核心话题与 TF：

| 接口 | 说明 |
|---|---|
| `/cmd_vel` | 机器人速度指令 |
| `/odom` | Gazebo 差速驱动里程计 |
| `/lidar/points_raw` | `sensor_msgs/msg/PointCloud2` 三维雷达原始点云；阶段 8.5 的感知适配层输入 |
| `/lidar/imu` | `sensor_msgs/msg/Imu` 雷达内置 IMU 数据；阶段 8.5 只保留接口，暂不参与建图、定位或里程计融合 |
| `/scan` | `sensor_msgs/msg/LaserScan` 二维扫描；二维仿真模式由雷达直接发布，MID-360 模式由 `/lidar/points_raw` 投影生成，供 SLAM Toolbox 和 AMCL 使用 |
| `/map` | 建图时由 `slam_toolbox` 发布；定位时由 `map_server` 加载并发布保存的地图 |
| `/amcl_pose` | AMCL 输出的当前定位结果，消息类型为 `PoseWithCovarianceStamped` |
| `/particle_cloud` | AMCL 粒子云，用于观察定位是否收敛 |
| `/amcl_path` | 项目节点根据 `/amcl_pose` 累积的估计轨迹，消息类型为 `nav_msgs/Path` |
| `odom → base_link → laser_link` | 建图和定位都必须存在的基础 TF 链 |
| `map → odom` | 建图时由 `slam_toolbox` 发布；定位时由 AMCL 发布；两者不能同时运行 |

### 2.1 阶段 8.5 感知接口契约

阶段 8.5 使用以下数据流：

```text
仿真三维雷达 / 真机 MID-360
              │
              ▼
 /lidar/points_raw  (PointCloud2)
              │
              ▼
  pointcloud_to_laserscan
              │
              ▼
       /scan  (LaserScan)
              │
              ├── SLAM Toolbox
              └── AMCL
```

- SLAM Toolbox 和 AMCL 继续只使用 `/scan`，不直接依赖 Livox 驱动或私有消息类型。
- 三维仿真或真机模式下，`/scan` 只能由点云转换节点发布，不得同时存在第二个 `/scan` 发布者。
- `/lidar/points_raw` 必须使用标准 `PointCloud2`，消息至少包含 `x`、`y`、`z` 字段，其 `frame_id` 应与机器人 TF 中的 `laser_link` 一致。
- `/lidar/points_raw` 和 `/lidar/imu` 保留给后续阶段；阶段 8.5 不将三维点云接入避障。过滤点云、局部代价地图和 Collision Monitor 在阶段 15 实现。
- 二维仿真兼容模式可继续由雷达直接发布 `/scan`，此时不要启动点云转换节点。

## 3. Launch 脚本

### 3.1 脚本职责

| 脚本 | 启动内容 | 适用场景 |
|---|---|---|
| `display.launch.py` | Xacro、`robot_state_publisher`、`joint_state_publisher` 和可选 RViz | 检查模型、关节和静态 TF，不启动 Gazebo |
| `gazebo.launch.py` | Gazebo、机器人实体、`robot_state_publisher` 和可选 RViz | 基础仿真、运动和传感器测试 |
| `lidar_adapter.launch.py` | 将 `/lidar/points_raw` 投影成 `/scan` | MID-360 仿真和真机的二维建图/定位适配 |
| `slam.launch.py` | `slam_toolbox`、可选建图 RViz、可选地图自动保存节点 | 在已经运行的机器人和雷达基础上建图 |
| `localization.launch.py` | `map_server`、AMCL、生命周期管理器、AMCL 轨迹节点和可选 RViz | 在已保存地图中定位 |

`slam.launch.py` 与 `gazebo.launch.py` 分开，是为了能够单独重启 SLAM 而不重置仿真，也便于以后复用到真机。建图时只应启动一个 RViz：Gazebo 使用 `use_rviz:=false`，SLAM 使用默认的 `use_rviz:=true`。

### 3.2 `display.launch.py`

只显示机器人模型：

```bash
ros2 launch cakebot_description display.launch.py
```

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `use_rviz` | `true` | 是否启动 RViz |
| `caster_x` | `-0.12` | 万向轮相对 `base_link` 的 x 坐标 |
| `caster_y` | `0.0` | 万向轮相对 `base_link` 的 y 坐标 |
| `caster_yaw` | `0.0` | 万向轮偏航角，单位 rad |
| `wheel_x` | `0.0` | 左右驱动轮共同的 x 坐标 |
| `wheel_separation` | `0.22` | 左右驱动轮中心距离，单位 m |

### 3.3 `gazebo.launch.py`

启动基础仿真：

```bash
ros2 launch cakebot_description gazebo.launch.py
```

默认 `sensor_mode:=laser_2d`，由 Gazebo 直接发布 `/scan`。阶段 8.5 的 MID-360 仿真模式只发布三维点云，再由感知适配器转换：

```bash
ros2 launch cakebot_description gazebo.launch.py \
  sensor_mode:=mid360_sim \
  use_rviz:=false
```

该模式发布 `/lidar/points_raw`（`sensor_msgs/msg/PointCloud2`），不会同时发布 `/scan`。`mid360_update_rate`、`mid360_horizontal_samples` 和 `mid360_vertical_samples` 可用于调整仿真负载；默认值分别为 `10.0`、`360` 和 `16`。垂直采样数是 Gazebo 仿真性能参数，不代表真机 MID-360 的实际点云密度。

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `world` | `cakebot.world` | Gazebo 世界文件 |
| `server` | `true` | 是否启动 Gazebo 仿真服务器；由 `gazebo_ros` 的被包含脚本提供 |
| `gui` | `true` | 是否启动 Gazebo 图形界面 |
| `paused` | `false` | 是否暂停仿真 |
| `use_sim_time` | `true` | ROS 节点是否使用 Gazebo 时间 |
| `use_rviz` | `true` | 是否同时启动 RViz |
| `sensor_mode` | `laser_2d` | `laser_2d` 发布 `/scan`；`mid360_sim` 发布 `/lidar/points_raw` |
| `visualize_lidar` | `true` | 是否在 Gazebo GUI 中显示雷达射线；设为 `false` 不影响消息发布 |
| `mid360_update_rate` | `10.0` | MID-360 仿真点云更新频率，单位 Hz |
| `mid360_horizontal_samples` | `360` | MID-360 仿真水平采样数 |
| `mid360_vertical_samples` | `16` | MID-360 仿真垂直采样数 |
| `x`、`y`、`z` | `0.0`、`0.0`、`0.01` | 机器人出生位置，单位 m |
| `yaw` | `0.0` | 机器人出生偏航角，单位 rad |

### 3.4 `lidar_adapter.launch.py`

MID-360 仿真模式启动后，在另一个终端启动点云适配器：

```bash
ros2 launch cakebot_perception lidar_adapter.launch.py \
  use_sim_time:=true
```

适配器订阅 `/lidar/points_raw`，选取雷达坐标系中稳定的水平高度带并发布 `/scan`。默认投影范围为 `-0.05～0.20 m`、距离范围为 `0.12～8.0 m`，参数文件位于 `config/pointcloud_to_laserscan.yaml`。二维 `laser_2d` 模式不要启动该适配器。

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `use_sim_time` | `false` | 仿真时传 `true`，真机时保持 `false` |
| `params_file` | 包内 `config/pointcloud_to_laserscan.yaml` | 点云投影参数文件 |
| `input_cloud_topic` | `/lidar/points_raw` | 输入 `PointCloud2` topic |
| `output_scan_topic` | `/scan` | 输出 `LaserScan` topic |

投影参数文件当前使用：

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `target_frame` | `laser_link` | 转换后的扫描坐标系；保持现有 TF 和 AMCL 配置不变 |
| `transform_tolerance` | `0.05` | 点云转换等待 TF 的容差，单位 s |
| `min_height`、`max_height` | `-0.05`、`0.20` | 相对 `laser_link` 的定位用高度带，主动排除地面 |
| `angle_min`、`angle_max` | `-π`、`π` | 输出完整 360° 扫描 |
| `angle_increment` | `0.01745329252` | 约 1°，输出约 360 条射线 |
| `scan_time` | `0.10` | 扫描周期，单位 s |
| `range_min`、`range_max` | `0.12`、`8.0` | 供现有 SLAM 和 AMCL 使用的有效距离范围，单位 m |
| `use_inf` | `true` | 没有命中时输出正无穷 |

转换节点采用按需订阅：只有 `/scan` 存在订阅者时才处理输入点云。使用 SLAM、AMCL、RViz 或 `ros2 topic echo /scan` 均会触发转换。

### 3.5 `perception_smoke_test.py`

阶段 8.5 的感知冒烟测试同时检查 `/lidar/points_raw`、`/scan`、传感器 TF、消息频率、有效扫描距离以及 topic 发布者数量：

```bash
ros2 run cakebot_perception perception_smoke_test.py \
  --ros-args -p use_sim_time:=true -p timeout_sec:=20.0
```

默认要求点云和扫描频率至少为 `5 Hz`、扫描至少包含 300 条射线和 20 个有效距离值，并要求 `/scan` 只有一个发布者。测试只使用墙体等稳定高度带生成的 `/scan`，不验证三维避障。

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `input_cloud_topic` | `/lidar/points_raw` | 待检查的 `PointCloud2` topic |
| `scan_topic` | `/scan` | 待检查的投影扫描 topic |
| `sensor_frame` | `laser_link` | 点云和扫描期望使用的 frame |
| `base_frame` | `base_link` | TF 检查的父坐标系 |
| `timeout_sec` | `20.0` | 等待点云、扫描和 TF 的最长时间，单位 s |
| `min_samples` | `5` | 计算频率前至少接收的消息数 |
| `min_rate_hz` | `5.0` | 点云和扫描允许的最低频率 |
| `min_scan_size` | `300` | `LaserScan.ranges` 的最小长度 |
| `min_valid_ranges` | `20` | 单帧扫描中最少的有限有效距离数 |
| `require_tf` | `true` | 是否要求 `base_link → laser_link` 可用 |

### 3.6 `slam.launch.py`

该脚本假定 `/scan`、`/odom` 和机器人 TF 已经由仿真或真机发布。它不会启动 Gazebo，也不会生成机器人实体。

```bash
ros2 launch cakebot_description slam.launch.py
```

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `use_sim_time` | `true` | SLAM 和 RViz 是否使用仿真时间 |
| `slam_params_file` | 包内 `config/slam_toolbox.yaml` | SLAM 参数文件 |
| `use_rviz` | `true` | 是否启动建图 RViz |
| `rviz_config_file` | 包内 `rviz/cakebot.rviz` | RViz 配置文件 |
| `autosave_on_shutdown` | `false` | 是否在该 launch 收到 `Ctrl+C` 时保存最新地图 |
| `autosave_map_file` | `maps/slam_map` | 自动保存文件前缀，不含扩展名 |

RViz 建图配置使用：

- Fixed Frame：`map`
- Map Topic：`/map`
- LaserScan Topic：`/scan`
- LaserScan Reliability：`Best Effort`
- MID-360 原始点云：`/lidar/points_raw`，在二维模式下没有数据属于正常现象

如果没有运行 SLAM，只想检查雷达，请临时把 RViz Fixed Frame 改为 `odom`。

### 3.7 `localization.launch.py`

该脚本假定 Gazebo 或真机已经提供 `/scan`、`/odom` 和 `odom → base_link → laser_link`。它加载已有地图并启动 `map_server`、`amcl`、`lifecycle_manager_localization` 与 `amcl_path_publisher`；不会启动 Gazebo，也不会启动 `slam_toolbox`。

```bash
ros2 launch cakebot_navigation localization.launch.py \
  map:=/home/wen/cakebot/maps/map-2d/test_env.yaml
```

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `map` | 无，必填 | Nav2 地图 YAML 的绝对路径 |
| `params_file` | 包内 `config/amcl.yaml` | AMCL 参数文件路径 |
| `use_sim_time` | `true` | 地图服务器、AMCL 和 RViz 是否使用 Gazebo 时间 |
| `use_rviz` | `true` | 是否启动定位 RViz；无图形界面时设为 `false` |
| `rviz_config_file` | 包内 `rviz/localization.rviz` | 定位 RViz 配置文件路径 |
| `save_trajectory` | `false` | 定位进程正常退出时是否自动保存完整 AMCL 轨迹 CSV |
| `trajectory_file` | `amcl_trajectory.csv` | CSV 输出路径；相对路径以启动命令的工作目录为基准 |
| `trajectory_overwrite` | `false` | 是否允许覆盖已有的同名 CSV |

定位 RViz 的 Fixed Frame 为 `map`，预置显示 `/map`、`/scan`、机器人模型、TF、`/particle_cloud`、`/amcl_pose` 和 `/amcl_path`。它还预置了默认关闭的 `/lidar/points_raw` 显示，需要对照原始点云时可手动开启。使用顶部工具栏的 **2D Pose Estimate** 可向 `/initialpose` 发布初始位姿。

定位模式中，`map → odom` 的唯一发布者应当是 AMCL。因此启动 `localization.launch.py` 前，必须停止 `slam.launch.py`。

## 4. 地图自动保存脚本

`slam_map_autosaver.py` 订阅并缓存最新的 `/map`。收到正常退出信号后，它将地图保存为 Nav2 兼容的 YAML 和 PGM 文件。父目录不存在时会自动创建。

通常不需要直接运行该脚本，应通过 `slam.launch.py` 启用：

```bash
ros2 launch cakebot_description slam.launch.py \
  autosave_on_shutdown:=true \
  autosave_map_file:=/home/wen/cakebot/maps/map-2d/test_env
```

| launch 参数 | 对应行为 |
|---|---|
| `autosave_on_shutdown:=false` | 不启动自动保存节点；这是默认行为 |
| `autosave_on_shutdown:=true` | 缓存 `/map`，在运行 SLAM 的终端按 `Ctrl+C` 后保存 |
| `autosave_map_file:=...` | 指定输出前缀，例如 `.../test_env` 生成 `test_env.yaml` 和 `test_env.pgm` |

也可以在 SLAM 已运行时单独启动保存节点：

```bash
ros2 run cakebot_description slam_map_autosaver.py --ros-args \
  -p output_file:=/home/wen/cakebot/maps/map-2d/test_env
```

直接运行时，确认它已经收到 `/map`，再在它所在的终端按 `Ctrl+C`。节点参数只有一个：

| 节点参数 | 默认值 | 说明 |
|---|---:|---|
| `output_file` | `maps/slam_map` | 地图文件前缀；允许传入无后缀、`.yaml` 或 `.pgm` 路径 |

如果终端提示 `Autosave skipped because no /map message was received`，说明退出前没有收到有效地图，不会生成文件。相对路径以启动命令时的当前目录为基准，因此推荐使用绝对路径。

## 5. 阶段 7：完整建图流程

以下三个终端都应先加载 ROS 2 和工作空间环境。

### 5.1 终端 1：启动测试环境

```bash
ros2 launch cakebot_description gazebo.launch.py \
  world:=/home/wen/cakebot/src/cakebot_description/worlds/test_env.world \
  use_rviz:=false
```

这里关闭 Gazebo 自带的 RViz，避免和建图 RViz 重复。

### 5.2 终端 2：启动 SLAM 和自动保存

推荐在验收时启用自动保存：

```bash
ros2 launch cakebot_description slam.launch.py \
  autosave_on_shutdown:=true \
  autosave_map_file:=/home/wen/cakebot/maps/map-2d/test_env
```

等待 RViz 出现地图，并确认没有持续的 TF 或 Message Filter 错误。

### 5.3 终端 3：键盘遥控

```bash
ros2 run cakebot_demo_cpp keyboard_control_test
```

| 按键 | 动作 |
|---|---|
| `w` | 持续前进 |
| `s` | 持续后退 |
| `a` | 原地左转 |
| `d` | 原地右转 |
| `x` 或空格 | 停止 |
| `q` | 停止并退出键盘节点 |

每次运动按键会保持生效，直到按下其他运动键或停止键。可调参数：

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `linear_speed` | `0.20` | 前进/后退速度，单位 m/s |
| `angular_speed` | `0.80` | 原地旋转速度，单位 rad/s |
| `command_timeout` | `0.0` | 无按键后自动停止时间，单位 s；`0.0` 表示禁用 |

低速示例：

```bash
ros2 run cakebot_demo_cpp keyboard_control_test --ros-args \
  -p linear_speed:=0.10 \
  -p angular_speed:=0.50 \
  -p command_timeout:=1.0
```

探索时先覆盖外围墙体，再覆盖内部障碍物，最后回到已经走过的区域以帮助闭合回环。不要同时运行 `basic_motion_test`，否则两个节点会同时发布 `/cmd_vel`。

### 5.4 结束建图并自动保存

1. 在键盘终端按 `x`，再按 `q`，确保机器人停止。
2. 切换到运行 `slam.launch.py` 的终端，按 `Ctrl+C`。
3. 等待终端输出 `Saved map automatically` 后，再停止 Gazebo。

只有运行 `slam.launch.py` 的终端中的 `Ctrl+C` 会触发本流程的自动保存。键盘终端或 Gazebo 终端中的 `Ctrl+C` 不会触发保存。

默认输出为：

```text
/home/wen/cakebot/maps/map-2d/test_env.yaml
/home/wen/cakebot/maps/map-2d/test_env.pgm
```

### 5.5 不启用自动保存时

保持 SLAM 运行，在另一个终端执行：

```bash
mkdir -p /home/wen/cakebot/maps/map-2d
ros2 run nav2_map_server map_saver_cli \
  -f /home/wen/cakebot/maps/map-2d/test_env
```

确认保存成功后，再停止键盘、SLAM 和 Gazebo。

## 6. 阶段 7 验收指令

### 6.1 运行期间检查

检查 SLAM 节点、地图话题和数据：

```bash
ros2 node list | grep slam_toolbox
ros2 topic list | grep '^/map'
ros2 topic info /map -v
ros2 topic echo /map --once
ros2 topic hz /map
```

检查雷达、里程计和完整 TF 链：

```bash
ros2 topic echo /scan --once
ros2 topic hz /scan
ros2 topic echo /odom --once
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link laser_link
```

`ros2 topic hz` 和 `tf2_echo` 会持续运行，观察到稳定输出后按 `Ctrl+C` 结束检查命令。

运行期间应满足：

- `/scan`、`/odom` 和 `/map` 持续更新；
- `/map` 消息的 `frame_id` 为 `map`；
- `map → odom → base_link → laser_link` TF 链完整；
- RViz 中激光点与墙体轮廓基本重合；
- 移动时地图逐渐扩展，回到旧区域后地图没有明显重影或跳变。

### 6.2 保存结果检查

```bash
ls -lh /home/wen/cakebot/maps/map-2d/test_env.yaml \
  /home/wen/cakebot/maps/map-2d/test_env.pgm
sed -n '1,80p' /home/wen/cakebot/maps/map-2d/test_env.yaml
file /home/wen/cakebot/maps/map-2d/test_env.pgm
```

YAML 至少应包含 `image`、`resolution`、`origin`、`occupied_thresh` 和 `free_thresh`。

### 6.3 重新加载保存的地图

停止 `slam.launch.py` 后，终端 1 启动地图服务器：

```bash
ros2 run nav2_map_server map_server --ros-args \
  -p yaml_filename:=/home/wen/cakebot/maps/map-2d/test_env.yaml
```

终端 2 激活生命周期节点：

```bash
ros2 lifecycle set /map_server configure
ros2 lifecycle set /map_server activate
ros2 lifecycle get /map_server
ros2 topic echo /map --once
```

终端 3 打开 RViz：

```bash
rviz2 -d \
  /home/wen/cakebot/install/cakebot_description/share/cakebot_description/rviz/cakebot.rviz
```

验收结束后，在地图服务器和 RViz 终端分别按 `Ctrl+C`。

本小节只验证地图文件可以被重新加载，不会启动 AMCL；完整定位流程见第 7 节。

### 6.4 完成标准

- RViz 能实时显示由雷达生成的占据栅格地图；
- 地图主要墙体闭合，障碍物位置与 `test_env.world` 基本一致；
- 能观察到 `map → odom`；
- 能生成并重新加载 `test_env.yaml` 和 `test_env.pgm`；
- 自动保存模式下，SLAM 终端按 `Ctrl+C` 后出现成功保存日志。

## 7. 阶段 8：AMCL 定位、遥控与估计轨迹

阶段 7 的 `test_env.yaml` 和 `test_env.pgm` 是本阶段的输入。AMCL 不会重新建图：它将当前 `/scan` 与已保存 `/map` 匹配，并结合 `/odom` 估计机器人在 `map` 坐标系中的位置，发布 `/amcl_pose` 和 `map → odom`。

### 7.1 启动测试环境

终端 1 启动 Gazebo；关闭该 launch 自带的 RViz，避免和定位 RViz 重复：

```bash
ros2 launch cakebot_description gazebo.launch.py \
  world:=/home/wen/cakebot/src/cakebot_description/worlds/test_env.world \
  use_rviz:=false
```

### 7.2 启动静态地图与 AMCL

确认没有运行 `slam.launch.py` 后，在终端 2 启动定位：

```bash
ros2 launch cakebot_navigation localization.launch.py \
  map:=/home/wen/cakebot/maps/map-2d/test_env.yaml
```

等待终端出现 `Managed nodes are active`。此时 RViz 会显示地图和机器人；在尚未设置初始位姿前，粒子云与机器人位置可能不可靠，这是正常现象。

### 7.3 设置初始位姿

在定位 RViz 中选择 **2D Pose Estimate**：在地图中单击机器人实际出生的大致位置，并沿机器人朝向拖动箭头。该操作向 `/initialpose` 发布 `PoseWithCovarianceStamped`，AMCL 用它初始化粒子云。

Gazebo 的出生位姿是 `odom`/仿真世界中的位姿，保存地图中的位姿则属于 `map` 坐标系；SLAM 回环校正后，两者不一定同为 `(0, 0, 0)`。因此应以地图墙体、障碍物和机器人实际位置为参照设置初始位姿，不要机械地假定默认出生位姿对应地图原点。若通过 `gazebo.launch.py` 的 `x`、`y` 或 `yaw` 改变了出生位姿，必须在 RViz 中给出对应的地图位姿。

定位收敛后应观察到：

- `/particle_cloud` 从大范围分散逐渐收敛；
- 红色 `/scan` 点与地图墙体基本重合；
- 机器人模型通过 `map → odom → base_link` 出现在正确位置；
- 橙色 `/amcl_path` 随 AMCL 位姿更新而延长。

### 7.4 键盘遥控并观察估计轨迹

终端 3 复用阶段 7 的键盘控制节点：

```bash
ros2 run cakebot_demo_cpp keyboard_control_test
```

按键、速度参数和停止方式见第 5.3 节。缓慢前进、转向并经过不同墙体结构；RViz 中的 `/amcl_path` 是 AMCL 估计轨迹，不是 Gazebo 真值轨迹。定位校正时轨迹有小幅调整是正常现象。

不要同时运行 `basic_motion_test` 或其他 `/cmd_vel` 发布者。

### 7.5 使用定位冒烟测试脚本

`localization_smoke_test.py` 自动检查定位数据路径：它等待 `/map`、`/scan`、`/odom`，发布 `/initialpose`，再检查 `/amcl_pose`、`/amcl_path` 和 `map → odom`。默认初始位姿为 `(0, 0, 0)`、默认超时为 20 秒；默认位姿只是示例，必须按保存地图中的实际位置调整。

在仿真中运行时必须传入 `use_sim_time:=true`：

```bash
ros2 run cakebot_navigation localization_smoke_test.py --ros-args \
  -p use_sim_time:=true \
  -p initial_x:=0.0 \
  -p initial_y:=0.0 \
  -p initial_yaw:=0.0
```

若还要验证键盘移动确实让 AMCL 位姿和轨迹更新，在脚本运行后于键盘终端移动机器人：

```bash
ros2 run cakebot_navigation localization_smoke_test.py --ros-args \
  -p use_sim_time:=true \
  -p require_motion:=true \
  -p motion_distance:=0.05 \
  -p motion_timeout_sec:=15.0
```

`require_motion:=true` 时，脚本会在基础定位检查成功后等待 AMCL 估计位置移动至少 `motion_distance` 米，并输出 `Localization smoke test PASSED` 或明确的超时原因。脚本运行时会自动设置初始位姿，因此不要同时在 RViz 中反复使用 **2D Pose Estimate** 覆盖它的初始条件。

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `initial_x`、`initial_y`、`initial_yaw` | `0.0` | 自动发布到 `/initialpose` 的地图位姿；yaw 单位为 rad |
| `timeout_sec` | `20.0` | 等待传感器和 AMCL 输出的超时，单位 s |
| `require_motion` | `false` | 是否额外要求 AMCL 估计位置发生移动 |
| `motion_distance` | `0.05` | 通过移动检查所需的最小估计位移，单位 m |
| `motion_timeout_sec` | `15.0` | 等待键盘移动的最长时间，单位 s |

### 7.6 阶段 8 验收指令与完成标准

RViz 无法稳定运行时，可以关闭 RViz 并启用轨迹自动保存。建议每次验收使用不同的绝对路径：

```bash
ros2 launch cakebot_navigation localization.launch.py \
  map:=/home/wen/cakebot/maps/map-2d/test_env.yaml \
  use_rviz:=false \
  save_trajectory:=true \
  trajectory_file:=/home/wen/cakebot/trajectories/amcl_run_01.csv
```

设置初始位姿并用键盘控制机器人移动后，在这个定位 launch 终端按一次 `Ctrl+C`。`amcl_path_publisher` 会在退出过程中保存 CSV，并输出保存的轨迹点数量和绝对路径。父目录不存在时会自动创建；默认不会覆盖已有文件，如确实需要覆盖可追加 `trajectory_overwrite:=true`。

CSV 的字段为：

```text
index,stamp_sec,stamp_nanosec,frame_id,x,y,yaw
```

其中 `x`、`y` 和 `yaw` 是 AMCL 在 `map` 坐标系中的估计位姿。CSV 使用与 `/amcl_path` 相同的距离和角度筛选规则，但会保留本次运行的全部轨迹点，不受 RViz 路径显示所用 `max_poses` 上限影响。没有收到任何 `/amcl_pose` 时不会生成正式 CSV。正常的 `Ctrl+C`、`SIGINT` 或 `SIGTERM` 会触发保存；`SIGKILL`、系统断电或虚拟机崩溃无法执行退出保存，此时目标路径旁可能留下带 `.tmp` 后缀的未完成数据。

运行期间可在额外终端检查：

```bash
ros2 lifecycle get /map_server
ros2 lifecycle get /amcl
ros2 topic echo /amcl_pose --once
ros2 topic echo /particle_cloud --once
ros2 topic echo /amcl_path --once
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo map base_link
```

`map_server` 和 `amcl` 应处于 `active`。`tf2_echo` 会持续运行，观察到稳定输出后按 `Ctrl+C` 结束。

阶段 8 完成标准：

- 不运行 `slam_toolbox`，仍能加载 `test_env.yaml`；
- RViz 设置初始位姿后，AMCL 发布 `/amcl_pose`、`/particle_cloud` 和稳定的 `map → odom`；
- `map → odom → base_link → laser_link` TF 链完整；
- 遥控移动时激光轮廓与地图基本对齐，`/amcl_path` 持续更新；
- 开启 `save_trajectory` 后正常结束定位，CSV 中包含多行不同的 AMCL 位姿；
- 冒烟测试在基础检查和可选移动检查中输出 `Localization smoke test PASSED`。

## 8. 阶段 8.5：MID-360 感知适配与回归验证

本阶段只把标准三维点云投影成现有建图和定位链路使用的二维 `/scan`。原始 `/lidar/points_raw` 会被保留，但尚未接入局部代价地图、Collision Monitor 或其他三维避障模块。

以下流程使用 `test_env.world`，所有终端都应先加载 ROS 2 和当前工作空间。

### 8.1 终端 1：启动 MID-360 仿真

```bash
ros2 launch cakebot_description gazebo.launch.py \
  world:=/home/wen/cakebot/src/cakebot_description/worlds/test_env.world \
  sensor_mode:=mid360_sim \
  visualize_lidar:=false \
  use_rviz:=false
```

`visualize_lidar:=false` 只关闭 Gazebo GUI 中的射线绘制，可降低图形负载，不会停止 `/lidar/points_raw`。若点云实际频率明显不足，可先降低 `mid360_vertical_samples` 或 `mid360_horizontal_samples`，再重新验收。

### 8.2 终端 2：启动点云适配器

```bash
ros2 launch cakebot_perception lidar_adapter.launch.py \
  use_sim_time:=true
```

此时数据链应为：

```text
/lidar/points_raw → pointcloud_to_laserscan → /scan
```

转换节点按需处理数据；如果还没有 `/scan` 订阅者，`/scan` 暂时没有消息属于正常现象。下一步的冒烟测试、SLAM、AMCL 或 RViz 都会建立订阅。

### 8.3 终端 3：运行感知冒烟测试

```bash
ros2 run cakebot_perception perception_smoke_test.py \
  --ros-args \
  -p use_sim_time:=true \
  -p timeout_sec:=20.0
```

通过时输出：

```text
Perception smoke test PASSED
```

也可以手动检查接口：

```bash
ros2 topic type /lidar/points_raw
ros2 topic hz /lidar/points_raw
ros2 topic type /scan
ros2 topic hz /scan
ros2 topic info /scan --verbose
ros2 run tf2_ros tf2_echo base_link laser_link
```

其中 `/scan` 必须恰好有一个发布者。在 `mid360_sim` 模式下若出现两个发布者，通常表示二维雷达实例或另一个点云转换节点仍在运行。

### 8.4 使用 MID-360 投影扫描建图

保持 Gazebo 和适配器运行，在新终端启动 SLAM：

```bash
ros2 launch cakebot_description slam.launch.py \
  use_sim_time:=true \
  autosave_on_shutdown:=true \
  autosave_map_file:=/home/wen/cakebot/maps/mid360_runs/test_env_mid360
```

再启动键盘遥控：

```bash
ros2 run cakebot_demo_cpp keyboard_control_test
```

完成环境覆盖后，先停止机器人，再在 SLAM 终端按 `Ctrl+C`。应生成：

```text
/home/wen/cakebot/maps/mid360_runs/test_env_mid360.yaml
/home/wen/cakebot/maps/mid360_runs/test_env_mid360.pgm
```

这个地图来自 MID-360 点云的高度带投影，应与 `maps/map-2d/` 中的二维雷达地图分开保存。

### 8.5 使用 MID-360 投影扫描进行 AMCL 定位

停止 `slam.launch.py`，但保持 Gazebo 和点云适配器运行，然后启动定位：

```bash
ros2 launch cakebot_navigation localization.launch.py \
  map:=/home/wen/cakebot/maps/mid360_runs/test_env_mid360.yaml \
  use_sim_time:=true
```

在 RViz 使用 **2D Pose Estimate** 设置初始位姿，或者运行自动定位检查：

```bash
ros2 run cakebot_navigation localization_smoke_test.py --ros-args \
  -p use_sim_time:=true \
  -p initial_x:=0.0 \
  -p initial_y:=0.0 \
  -p initial_yaw:=0.0
```

默认初始位姿只是示例；若保存地图中的机器人出生位置不在地图原点，应填写与地图匹配的位姿。

### 8.6 阶段 8.5 完成标准

- `mid360_sim` 模式持续发布非空 `/lidar/points_raw`，消息包含 `x`、`y`、`z` 字段，frame 为 `laser_link`；
- 适配器稳定生成约 360° 的 `/scan`，点云和扫描频率均不低于冒烟测试配置的阈值；
- `/scan` 恰好只有一个发布者，`base_link → laser_link` TF 可用；
- `perception_smoke_test.py` 输出 `Perception smoke test PASSED`；
- SLAM Toolbox 能使用投影后的 `/scan` 建图并保存 `test_env_mid360.yaml/.pgm`；
- AMCL 能加载 MID-360 流程生成的地图，发布 `/amcl_pose` 和稳定的 `map → odom`；
- `localization_smoke_test.py` 输出 `Localization smoke test PASSED`；
- `sensor_mode:=laser_2d` 的旧二维流程仍能运行；
- 原始三维点云没有在本阶段直接参与避障，SLAM 和 AMCL 不依赖 Livox 私有消息。

## 9. 基础运动测试

`basic_motion_test` 用于验证 `/cmd_vel`、`/odom` 和 `/scan`，不参与 SLAM 建图。先启动 Gazebo，再运行：

```bash
ros2 run cakebot_demo_cpp basic_motion_test
```

默认流程：等待传感器 → 等待 2 秒 → 前进 3 秒 → 停止 1 秒 → 原地旋转 2 秒 → 停止。

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `forward_speed` | `0.10` | 前进速度，单位 m/s |
| `angular_speed` | `0.30` | 旋转速度，单位 rad/s |
| `forward_duration` | `3.0` | 前进时间，单位 s |
| `rotate_duration` | `2.0` | 旋转时间，单位 s |
| `stop_duration` | `1.0` | 前进后的停止时间，单位 s |
| `start_delay` | `2.0` | 传感器就绪后的等待时间，单位 s |
| `safety_distance` | `0.45` | 前方障碍物安全距离，单位 m |
| `sensor_timeout` | `0.5` | `/scan` 超时时间，单位 s |

示例：

```bash
ros2 run cakebot_demo_cpp basic_motion_test --ros-args \
  -p forward_speed:=0.05 \
  -p safety_distance:=0.50
```

成功时终端输出 `Basic motion test PASSED`。该节点运行时不要同时启动键盘控制节点。

## 10. 模型与雷达参数

参数定义在 `src/cakebot_description/urdf/cakebot.urdf.xacro`：

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `laser_x` | `0.10` | 雷达相对 `base_link` 的 x 坐标 |
| `laser_y` | `0.0` | 雷达相对 `base_link` 的 y 坐标 |
| `laser_z` | `0.17` | 雷达相对 `base_link` 的 z 坐标 |
| `laser_update_rate` | `10.0` | 更新频率，单位 Hz |
| `laser_samples` | `360` | 水平扫描采样数 |
| `laser_min_angle` | `-pi` | 最小扫描角 |
| `laser_max_angle` | `pi` | 最大扫描角 |
| `laser_min_range` | `0.12` | 最小测距，单位 m |
| `laser_max_range` | `8.0` | 最大测距，单位 m |
| `laser_range_resolution` | `0.01` | 距离分辨率，单位 m |
| `sensor_mode` | `laser_2d` | `laser_2d` 使用二维雷达；`mid360_sim` 使用三维点云雷达 |
| `visualize_lidar` | `true` | 是否在 Gazebo GUI 中绘制雷达射线 |
| `mid360_update_rate` | `10.0` | 三维仿真点云更新频率，单位 Hz |
| `mid360_horizontal_samples` | `360` | 三维仿真水平采样数 |
| `mid360_vertical_samples` | `16` | 三维仿真垂直采样数 |
| `mid360_min_vertical_angle` | `-0.1221730476` | 最小垂直角，约 -7° |
| `mid360_max_vertical_angle` | `0.9075712110` | 最大垂直角，约 52° |
| `mid360_min_range` | `0.10` | 三维仿真雷达最小测距，单位 m |
| `mid360_max_range` | `40.0` | 三维仿真雷达最大测距，单位 m |
| `mid360_range_resolution` | `0.01` | 三维仿真距离分辨率，单位 m |
| `mid360_points_topic` | `lidar/points_raw` | 三维仿真点云输出 topic；最终解析为 `/lidar/points_raw` |

`mid360_sim` 是使用 Gazebo 规则射线实现的近似模型，用于验证 ROS 接口和 `PointCloud2 → LaserScan` 数据链，不模拟真机的非重复扫描模式、噪声、反射率、时间畸变或实际点云密度。二维与三维模式在空的 `cakebot.world` 中都可能没有有效回波；建图和有效距离测试应使用 `test_env.world`。

## 11. 常见问题

### RViz 打开了两个窗口

建图时 Gazebo 必须使用 `use_rviz:=false`，由 `slam.launch.py` 启动唯一的建图 RViz；定位时同样让 Gazebo 使用 `use_rviz:=false`，由 `localization.launch.py` 启动唯一的定位 RViz。

### RViz 显示 `Fixed Frame [map] does not exist`

建图模式下确认 `slam.launch.py` 已启动；定位模式下确认 `localization.launch.py` 已启动且 AMCL 已初始化。两种模式都可检查：

```bash
ros2 run tf2_ros tf2_echo map odom
```

如果只进行雷达测试、没有启动 SLAM 或 AMCL，把 RViz Fixed Frame 临时改为 `odom`。

### MID-360 模式存在点云，但 `/scan` 没有消息

`pointcloud_to_laserscan` 采用按需订阅。先确认适配器已经启动，再运行以下任意一个订阅者：

```bash
ros2 topic echo /scan --once
```

SLAM、AMCL、RViz 或 `perception_smoke_test.py` 也会触发转换。如果仍无数据，检查输入 topic 和 frame：

```bash
ros2 topic echo /lidar/points_raw --once --field header
ros2 run tf2_ros tf2_echo base_link laser_link
```

### 感知冒烟测试报告 `/scan` 发布者数量不为 1

MID-360 模式中，Gazebo 只应发布 `/lidar/points_raw`，`/scan` 只应由一个点云适配器发布。检查：

```bash
ros2 topic info /scan --verbose
ros2 node list
```

确认使用了 `sensor_mode:=mid360_sim`，并停止重复启动的 `lidar_adapter.launch.py` 或残留的二维雷达仿真实例。

### 感知冒烟测试频率不足或有效距离太少

使用包含墙体和障碍物的 `test_env.world`，并关闭 Gazebo 射线显示：

```bash
visualize_lidar:=false
```

若电脑无法维持默认负载，可适当降低 `mid360_vertical_samples` 或 `mid360_horizontal_samples`。点云正常但有效 `/scan` 太少时，检查 `pointcloud_to_laserscan.yaml` 的 `min_height`、`max_height` 和距离范围；不要为了增加点数把地面大量投影进定位扫描。

### 定位时没有 `map → odom`，或定位不断跳变

确认没有同时运行 `slam.launch.py`；`slam_toolbox` 与 AMCL 都会发布 `map → odom`。随后在 RViz 使用 **2D Pose Estimate** 给出与地图匹配的初始位姿，并检查 `/scan`、`/odom`、`odom → base_link → laser_link` 是否持续更新：

```bash
ros2 lifecycle get /amcl
ros2 topic echo /amcl_pose --once
ros2 run tf2_ros tf2_echo map odom
```

### 冒烟测试超时或 `/amcl_path` 没有更新

仿真运行脚本时必须传入 `-p use_sim_time:=true`。先确认 Gazebo、`localization.launch.py` 和键盘节点分别在独立终端运行；基础测试要求 `/map`、`/scan`、`/odom` 都已出现。若设置了 `require_motion:=true`，请在 `motion_timeout_sec` 内键盘移动机器人至少 `motion_distance` 米。初始位姿错误时，也应先在 RViz 中定位到正确位置，再把相同的地图坐标传给脚本的 `initial_x`、`initial_y` 和 `initial_yaw`。

### 键盘按键没有响应

确认键盘终端保持焦点，并检查是否有多个 `/cmd_vel` 发布者：

```bash
ros2 topic info /cmd_vel -v
```

### 自动保存没有生成文件

确认启动命令包含 `autosave_on_shutdown:=true`，退出前 `/map` 已经出现，并在运行 `slam.launch.py` 的终端按 `Ctrl+C`。优先使用绝对输出路径。

### 地图明显重影或漂移

降低遥控速度，减少急转，确认 `/odom` 和 TF 连续，并重复经过已经建好的区域形成回环。

## 12. 生成目录

以下目录由 colcon 生成，不应提交：

- `build/`
- `install/`
- `log/`

`maps/` 中的 `.yaml` 和 `.pgm` 是有效实验成果，可按项目需要保留或提交。当前约定：

- `maps/map-2d/` 保存二维雷达流程生成的地图；
- `maps/mid360_runs/` 保存 MID-360 点云投影流程生成的地图；
- 同一份 YAML 引用的 PGM 应与其放在同一目录，不要只移动其中一个文件。
