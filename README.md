# cakebot ROS 2 仿真使用说明

本项目用于学习 ROS 2、Gazebo Classic、TF、激光雷达和 Nav2。当前主要包：

- `cakebot_description`：机器人模型、Gazebo 启动文件、世界文件和 RViz 配置；
- `cakebot_demo_cpp`：C++ 示例节点。

## 环境与编译

当前命令以 ROS 2 Humble 为例；如果使用其他发行版，请替换 `/opt/ros/humble`。

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

只编译机器人描述包：

```bash
colcon build --packages-select cakebot_description --symlink-install
source install/setup.bash
```

## 推荐启动方式

一条命令启动 Gazebo、机器人、`robot_state_publisher` 和 RViz：

```bash
ros2 launch cakebot_description gazebo.launch.py
```

启动参数：

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `gui` | `true` | 是否启动 Gazebo 图形界面 |
| `paused` | `false` | 是否暂停启动仿真 |
| `use_sim_time` | `true` | 是否使用 Gazebo 仿真时间 |
| `use_rviz` | `true` | 是否同时启动 RViz |
| `world` | `cakebot.world` | Gazebo 世界文件 |
| `x` | `0.0` | 机器人初始 x 坐标 |
| `y` | `0.0` | 机器人初始 y 坐标 |
| `z` | `0.01` | 机器人初始 z 坐标 |
| `yaw` | `0.0` | 机器人初始偏航角，单位为弧度 |

示例：只启动 Gazebo，不启动 RViz，并暂停仿真：

```bash
ros2 launch cakebot_description gazebo.launch.py \
  use_rviz:=false paused:=true gui:=true \
  x:=0.0 y:=0.0 z:=0.01 yaw:=0.0
```

## 仅启动模型显示

不启动 Gazebo，只在 RViz 中显示机器人模型：

```bash
ros2 launch cakebot_description display.launch.py
```

`display.launch.py` 参数：

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `use_rviz` | `true` | 是否启动 RViz |
| `caster_x` | `-0.12` | 万向轮 x 坐标 |
| `caster_y` | `0.0` | 万向轮 y 坐标 |
| `caster_yaw` | `0.0` | 万向轮偏航角 |
| `wheel_x` | `0.0` | 驱动轮 x 坐标 |
| `wheel_separation` | `0.22` | 左右轮中心距离 |

## 当前模型与雷达参数

参数定义在 `src/cakebot_description/urdf/cakebot.urdf.xacro`：

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `laser_x` | `0.10` | 雷达相对 `base_link` 的 x 坐标 |
| `laser_y` | `0.0` | 雷达相对 `base_link` 的 y 坐标 |
| `laser_z` | `0.17` | 雷达相对 `base_link` 的 z 坐标 |
| `laser_update_rate` | `10.0` | 雷达更新频率，单位 Hz |
| `laser_samples` | `360` | 水平扫描采样数 |
| `laser_min_angle` | `-pi` | 最小扫描角 |
| `laser_max_angle` | `pi` | 最大扫描角 |
| `laser_min_range` | `0.12` | 最小测距，单位 m |
| `laser_max_range` | `8.0` | 最大测距，单位 m |
| `laser_range_resolution` | `0.01` | 距离分辨率，单位 m |

雷达话题和坐标系固定为：

- 话题：`/scan`；
- 类型：`sensor_msgs/msg/LaserScan`；
- 坐标系：`laser_link`；
- 固定 TF：`base_link -> laser_link`。

## 验收与排查命令

验证 Xacro 和 URDF：

```bash
xacro src/cakebot_description/urdf/cakebot.urdf.xacro > /tmp/cakebot.urdf
check_urdf /tmp/cakebot.urdf
```

检查雷达话题：

```bash
ros2 topic list | grep '^/scan$'
ros2 topic info /scan
ros2 topic echo /scan --once
ros2 topic hz /scan
```

检查雷达 TF：

```bash
ros2 run tf2_ros tf2_echo base_link laser_link
```

手动启动 RViz：

```bash
rviz2 -d src/cakebot_description/rviz/cakebot.rviz
```

RViz 中应设置：

- `Fixed Frame`：`base_link`；
- `LaserScan` Topic：`/scan`；
- QoS Reliability：`Best Effort`。

当前 `cakebot.world` 只有地面和太阳，没有墙或箱子。雷达水平扫描没有遇到障碍物时，`ranges` 可能全部为 `inf`，RViz 不显示扫描点属于正常现象。需要验证有效回波时，在 Gazebo 中放置 Box 等障碍物。

## 生成文件

以下内容由 ROS 2/colcon 生成，不提交到 Git：

- `build/`；
- `install/`；
- `log/`；
- `symlink_install_manifest.txt`。


### 注：目前雷达没有噪声