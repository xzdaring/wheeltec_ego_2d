# V550_mec + N10P + EGO 实车操作

代码在电脑上编写，硬件启动和控制在小车 ROS 2 Humble 主机执行。
下面命令使用 bash；zsh 用户将 setup.bash 换成 setup.zsh。
本入口沿用 EGO 的二维 B 样条和 FSM，使用全向反馈控制，保留横移。

## 1. 复制源码与编译

在小车上准备一个独立工作区，例如 `~/wheeltec_ego_2d`。
将电脑工作区的 `src/` 复制到小车该目录，保留其中 `depend/`、消息包、驱动、planner、uav_simulator、robot_sim 和模型。
不要复制电脑的 build/install/log；不同 CPU 架构必须重新编译。
如果使用 scp，将 `<小车用户>` 和 `<小车IP>` 替换为实际值，在电脑运行：

```bash
ssh <小车用户>@<小车IP> 'mkdir -p ~/wheeltec_ego_2d/src'
scp -r /ros_workspace/wheeltec_ego_2d/src/. <小车用户>@<小车IP>:~/wheeltec_ego_2d/src/
```

小车终端：

```bash
cd ~/wheeltec_ego_2d
source /opt/ros/humble/setup.bash
# 有且仅在缺少依赖时安装；本电脑未安装 robot_localization，小车也需检查。
sudo apt update
sudo apt install ros-humble-robot-localization ros-humble-joint-state-publisher
CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS=-j2 colcon build \
  --symlink-install --packages-up-to robot_sim --parallel-workers 2 \
  --cmake-args -DBUILD_TESTING=OFF -DAMENT_CMAKE_SYMLINK_INSTALL=ON
source install/setup.bash
```

每个新终端都要 source ROS 和这个工作区。若其他依赖报错，保留完整输出再处理。

## 2. 硬件入口

先关闭厂家自动导航、其他底盘/雷达驱动和旧 SLAM，避免重复打开串口及重复 TF。
在地面空旷区域操作，准备好小车实际的急停/断电方式。
检查串口路径和当前用户权限：

```bash
ls -l /dev/wheeltec_controller /dev/wheeltec_lidar
ros2 launch robot_sim v550_real_bringup.launch.py
```

可以覆盖串口：

```bash
ros2 launch robot_sim v550_real_bringup.launch.py \
  base_port:=/dev/ttyUSB0 lidar_port:=/dev/ttyUSB1
```

不要凭 ttyUSB 编号猜设备。默认底盘波特率 115200；N10P 使用
`lslidar_driver/params/lidar_uart_ros2/lsn10p.yaml` 和 `lslidar_driver_node`。
可以用 `lidar_config:=/绝对路径/lsn10p.yaml` 加载小车已有的标定配置。
厂家 N10P 参数默认屏蔽 120°–230°，本入口保留这一设置；先确认其对应车壳遮挡再调整，
不要把屏蔽扇区当作已观测自由空间。

本入口发布原始 `/odom`、`/imu/data_raw`，EKF 发布 `/odom_combined`。
EKF 直接使用驱动发布的 IMU，沿用现有 ekf.yaml，不额外启动 IMU 方向滤波器。
车型相关里程计比例沿用 1.0，尚未按你的实车标定。
模型雷达 TF 使用原 URDF 的 `base_link -> laser`，与仿真抬高的 `laser_scan` 不同。
车头正方向、雷达安装朝向和 IMU 正方向必须通过下一步现场检查。

## 3. 检查反馈，再启动 EGO

分别观察数秒，用 Ctrl+C 结束持续检查：

```bash
ros2 topic hz /odom_combined
ros2 topic echo /odom_combined --once
ros2 topic hz /scan
ros2 topic echo /scan --qos-reliability best_effort --once
ros2 run tf2_ros tf2_echo odom_combined base_footprint
ros2 run tf2_ros tf2_echo base_footprint laser
```

应有持续里程计和扫描；里程计 header.frame_id 为 odom_combined、child_frame_id 为 base_footprint。
雷达 frame_id 默认为 laser。实车朝前运动时，RViz 车体 X 正向应对应物理车头；
若方向相反，先检查 IMU、EKF、TF，不能靠旋转可视化模型掩盖。

终端 2：先用预览模式（此时导航不发布 /cmd_vel）：

```bash
ros2 launch robot_sim ego_real.launch.py rviz:=true
```

无桌面的主机用默认 `rviz:=false`；远程电脑运行 RViz 时需要相同 ROS_DOMAIN_ID、
可互通的 DDS 配置和系统时钟。在小车上完成首轮测试更容易排除网络问题。
可以在已加载本工作区的桌面终端单独执行：

```bash
rviz2 -d "$(ros2 pkg prefix robot_sim)/share/robot_sim/config/navigation.rviz"
```

等待 map 出现，RViz 用 2D Goal Pose 在车前约 0.5 m 的空地点击并拖动。
检查 `/ego/cloud`、`/drone_0_planning/bspline` 和 `/ego/cmd_vel_preview`：

```bash
ros2 topic hz /ego/cloud
ros2 topic echo /drone_0_planning/bspline --once
ros2 topic echo /ego/cmd_vel_preview --once
ros2 param get /drone_0_ego_planner_node use_sim_time
ros2 run tf2_ros tf2_echo map base_footprint
```

use_sim_time 应为 false；点云及 EGO 标记应为 odom_combined。
预览模式不会让车移动，所以仅能检查首段轨迹与输出，无法验证闭环行驶。

## 4. 开启实车执行

先 Ctrl+C 关闭终端 2 的预览导航，保留终端 1 的硬件。
量取车体平面外接圆半径，留出余量后填写 safety_radius；默认 0.25 m 是起始参数，未实车标定。

```bash
ros2 launch robot_sim ego_real.launch.py rviz:=true enable_motion:=true safety_radius:=0.25
ros2 topic info /cmd_vel --verbose
```

/cmd_vel 应只有 ground_traj_server 一个发布者。点击近处空地目标后才会运动。
首次执行限合速度 0.15 m/s、角速度 0.25 rad/s，x/y 共同限速，保留麦轮横移。
行驶时默认车头跟随 B 样条切线，预瞄时间为 0.3 s；朝向偏差大时降低平移速度，超过 90° 时先转向。
到点后仍使用 RViz 点击箭头作为最终朝向要求，横移用于位置纠偏。
可添加 `yaw_lookahead:=0.5` 调整预瞄时间；添加 `follow_trajectory_yaw:=false` 可关闭行驶转向，最终对齐仍有效。
在预览模式下，如果车头与轨迹方向相差很大，出现以角速度为主的输出是预期行为；车不实际转动时，不能用此模式验证朝向收敛。
如沿用小车已有 SLAM：加 `start_slam:=false`，仍需其发布 /map 和 map->odom_combined。
也可覆盖 odom_topic、planning_frame、base_frame、scan_topic；frame 必须与消息一致，不能只改名字。

停止时先关闭导航，再在加载环境的终端持续发零速度，确认停车后结束：

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist '{}'
```

断网或程序崩溃时，软件停车消息可能无法送达，不能替代底盘硬件急停或固件超时保护。
当前跟踪器保留里程计/雷达/地图超时停车和近障检查；二维雷达看不到低矮障碍与悬空障碍，
本版本尚未实车闭环验证，不保证复杂迷宫全局可达或动态障碍预测。

## 数据链

```
N10P /scan + EKF /odom_combined + TF
  -> slam_toolbox /map + map->odom_combined
  -> bridge /ego/cloud（真实变换到 odom_combined，z=0）
RViz /goal_pose（map）-> /move_base_simple/goal（odom_combined）
  -> EGO FSM -> /drone_0_planning/bspline
  -> ground_traj_server -> /ego/cmd_vel_preview 或 /cmd_vel
```

实车入口不会启动 Gazebo 或原无人机轨迹服务器；硬件入口与导航入口分开管理，
已经有可靠底盘、雷达、TF 时可直接使用导航入口，避免重复启动硬件。
