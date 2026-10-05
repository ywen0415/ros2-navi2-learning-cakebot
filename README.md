# cakebot ROS 2 仿真与建图说明

本项目用于学习 ROS 2 Humble、Gazebo Classic、TF、二维激光雷达、SLAM 和 Nav2。当前已经完成机器人仿真、差速驱动、里程计、激光雷达、人工遥控以及 `slam_toolbox` 在线建图接入，当前重点是阶段 7 的地图生成与验收。

## 1. 环境与编译

以下命令默认工作空间位于 `/home/wen/cakebot`，ROS 2 发行版为 Humble。

首次使用时确认建图依赖已安装：

```bash
sudo apt install ros-humble-slam-toolbox ros-humble-nav2-map-server
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

只编译本项目的两个包：

```bash
cd /home/wen/cakebot
source /opt/ros/humble/setup.bash
colcon build \
  --packages-select cakebot_description cakebot_demo_cpp \
  --symlink-install
```

## 2. 项目组成与核心接口

| 包 | 作用 |
|---|---|
| `cakebot_description` | Xacro/URDF、Gazebo 世界、launch、RViz、SLAM 参数和地图自动保存脚本 |
| `cakebot_demo_cpp` | 基础运动测试和键盘遥控节点 |

两个 Gazebo 世界的用途不同：

| 世界文件 | 用途 |
|---|---|
| `cakebot.world` | 只有地面和太阳，适合基础模型与运动测试 |
| `test_env.world` | 包含墙体和障碍物，用于雷达验证及 SLAM 建图 |

`test_env.world` 是仿真环境，不是 Nav2 使用的地图。SLAM 生成的地图是 `.yaml` 与 `.pgm` 文件。

核心话题与 TF：

| 接口 | 说明 |
|---|---|
| `/cmd_vel` | 机器人速度指令 |
| `/odom` | Gazebo 差速驱动里程计 |
| `/scan` | `sensor_msgs/msg/LaserScan` 二维雷达数据 |
| `/map` | SLAM 生成的占据栅格地图 |
| `odom → base_link → laser_link` | 建图前必须存在的 TF 链 |
| `map → odom` | `slam_toolbox` 建图时发布的 TF |

## 3. Launch 脚本

### 3.1 脚本职责

| 脚本 | 启动内容 | 适用场景 |
|---|---|---|
| `display.launch.py` | Xacro、`robot_state_publisher`、`joint_state_publisher` 和可选 RViz | 检查模型、关节和静态 TF，不启动 Gazebo |
| `gazebo.launch.py` | Gazebo、机器人实体、`robot_state_publisher` 和可选 RViz | 基础仿真、运动和传感器测试 |
| `slam.launch.py` | `slam_toolbox`、可选建图 RViz、可选地图自动保存节点 | 在已经运行的机器人和雷达基础上建图 |

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

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `world` | `cakebot.world` | Gazebo 世界文件 |
| `server` | `true` | 是否启动 Gazebo 仿真服务器；由 `gazebo_ros` 的被包含脚本提供 |
| `gui` | `true` | 是否启动 Gazebo 图形界面 |
| `paused` | `false` | 是否暂停仿真 |
| `use_sim_time` | `true` | ROS 节点是否使用 Gazebo 时间 |
| `use_rviz` | `true` | 是否同时启动 RViz |
| `x`、`y`、`z` | `0.0`、`0.0`、`0.01` | 机器人出生位置，单位 m |
| `yaw` | `0.0` | 机器人出生偏航角，单位 rad |

### 3.4 `slam.launch.py`

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

如果没有运行 SLAM，只想检查雷达，请临时把 RViz Fixed Frame 改为 `odom`。

## 4. 地图自动保存脚本

`slam_map_autosaver.py` 订阅并缓存最新的 `/map`。收到正常退出信号后，它将地图保存为 Nav2 兼容的 YAML 和 PGM 文件。父目录不存在时会自动创建。

通常不需要直接运行该脚本，应通过 `slam.launch.py` 启用：

```bash
ros2 launch cakebot_description slam.launch.py \
  autosave_on_shutdown:=true \
  autosave_map_file:=/home/wen/cakebot/maps/test_env
```

| launch 参数 | 对应行为 |
|---|---|
| `autosave_on_shutdown:=false` | 不启动自动保存节点；这是默认行为 |
| `autosave_on_shutdown:=true` | 缓存 `/map`，在运行 SLAM 的终端按 `Ctrl+C` 后保存 |
| `autosave_map_file:=...` | 指定输出前缀，例如 `.../test_env` 生成 `test_env.yaml` 和 `test_env.pgm` |

也可以在 SLAM 已运行时单独启动保存节点：

```bash
ros2 run cakebot_description slam_map_autosaver.py --ros-args \
  -p output_file:=/home/wen/cakebot/maps/test_env
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
  autosave_map_file:=/home/wen/cakebot/maps/test_env
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
/home/wen/cakebot/maps/test_env.yaml
/home/wen/cakebot/maps/test_env.pgm
```

### 5.5 不启用自动保存时

保持 SLAM 运行，在另一个终端执行：

```bash
mkdir -p /home/wen/cakebot/maps
ros2 run nav2_map_server map_saver_cli \
  -f /home/wen/cakebot/maps/test_env
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
ls -lh /home/wen/cakebot/maps/test_env.yaml \
  /home/wen/cakebot/maps/test_env.pgm
sed -n '1,80p' /home/wen/cakebot/maps/test_env.yaml
file /home/wen/cakebot/maps/test_env.pgm
```

YAML 至少应包含 `image`、`resolution`、`origin`、`occupied_thresh` 和 `free_thresh`。

### 6.3 重新加载保存的地图

停止 `slam.launch.py` 后，终端 1 启动地图服务器：

```bash
ros2 run nav2_map_server map_server --ros-args \
  -p yaml_filename:=/home/wen/cakebot/maps/test_env.yaml
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

### 6.4 完成标准

- RViz 能实时显示由雷达生成的占据栅格地图；
- 地图主要墙体闭合，障碍物位置与 `test_env.world` 基本一致；
- 能观察到 `map → odom`；
- 能生成并重新加载 `test_env.yaml` 和 `test_env.pgm`；
- 自动保存模式下，SLAM 终端按 `Ctrl+C` 后出现成功保存日志。

## 7. 基础运动测试

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

## 8. 模型与雷达参数

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

当前雷达为无噪声二维仿真雷达。在空的 `cakebot.world` 中，扫描没有击中物体时 `ranges` 可能全部为 `inf`；建图和有效回波测试应使用 `test_env.world`。

## 9. 常见问题

### RViz 打开了两个窗口

建图时 Gazebo 必须使用 `use_rviz:=false`，由 `slam.launch.py` 启动唯一的建图 RViz。

### RViz 显示 `Fixed Frame [map] does not exist`

确认 `slam.launch.py` 已启动，并检查：

```bash
ros2 run tf2_ros tf2_echo map odom
```

如果只进行雷达测试、没有启动 SLAM，把 RViz Fixed Frame 临时改为 `odom`。

### 键盘按键没有响应

确认键盘终端保持焦点，并检查是否有多个 `/cmd_vel` 发布者：

```bash
ros2 topic info /cmd_vel -v
```

### 自动保存没有生成文件

确认启动命令包含 `autosave_on_shutdown:=true`，退出前 `/map` 已经出现，并在运行 `slam.launch.py` 的终端按 `Ctrl+C`。优先使用绝对输出路径。

### 地图明显重影或漂移

降低遥控速度，减少急转，确认 `/odom` 和 TF 连续，并重复经过已经建好的区域形成回环。

## 10. 生成目录

以下目录由 colcon 生成，不应提交：

- `build/`
- `install/`
- `log/`

`maps/` 中的 `.yaml` 和 `.pgm` 是阶段 7 的有效成果，可按项目需要保留或提交。
