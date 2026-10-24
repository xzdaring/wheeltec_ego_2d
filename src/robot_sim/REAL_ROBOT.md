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

不要凭 ttyUSB 编号猜设备。默认底盘波特率 115200。
2026-10-16 在本车串口实测到 500000 波特率、158/160 字节的 M10_P 格式数据包，
因此本入口默认使用 `lslidar_driver/params/lidar_uart_ros2/lsm10_p.yaml`。
实体型号标签仍需核实；其他设备可用 `lidar_config:=/绝对路径/配置.yaml` 覆盖。

驱动发布 `/scan_raw`，`scan_self_filter.py` 按 TF 转到 `base_footprint`，
结合车体矩形和厂家遮挡角度剔除自身回波，再发布给 SLAM 与跟踪器共用的 `/scan`。
默认过滤范围为 x=[-0.140,0.140]、y=[-0.125,0.125] m，来自用户实测含轮长 26 cm、宽 23 cm 加 1 cm 测量余量；
该矩形以外的实际车体结构回波，通过下述厂家角度规则补充过滤。
被车体遮挡的测量设置为 NaN，保留原始时间戳，不视为已观测自由空间。

沿用厂家雷达坐标角度区间 70°～100°、120°～240°、260°～290°，默认 `factory_mask_mode:=full`，按厂家区间全距离屏蔽。可选 `near`：
仅过滤这些区间内距雷达不超过 `factory_near_range:=0.26` m 的有效回波，保留较远的有效测量。
26 cm 是距雷达的测距门槛；按本车尺寸与前置约 8.35 cm 的安装位置，车体后角距雷达最远约 24.3 cm，
预留约 2 cm 测量余量。它与车体中心的 `safety_radius:=0.25` m 是不同参数，后者保持不变。

| `factory_mask_mode` | 角度规则 | 矩形规则 |
|---|---|---|
| `near` | 厂家区间内只过滤 26 cm 内近点 | 保留 |
| `full`（默认） | 按厂家区间屏蔽全部距离 | 保留 |
| `off` | 关闭角度规则，复现旧矩形过滤 | 保留 |

默认 `lidar_crop_mode:=0` 在驱动中保留完整 `/scan_raw`，由过滤节点执行以上策略。
`lidar_crop_mode:=1` 会直接在驱动屏蔽厂家区间的所有距离，后续节点无法恢复被丢弃的测量。
遮挡过滤区在近场存在观测盲区，应按实物遮挡标定，缺少回波不代表自由空间。

更新后关闭旧的硬件和导航终端，再重新启动两者，使 SLAM 从新过滤扫描重建地图。
默认已使用厂家 full 规则；如需近点规则，硬件入口显式添加 `factory_mask_mode:=near`。

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


## 2026-10-17：bag5 跟踪与建图修正

`/scan_raw` 每圈点数随转速变化，不能直接作为 SLAM 固定型号的角度网格。
过滤节点先重排为 1800 个角度格（0.2°），同格保留最近有效点，缺测为 NaN，
然后执行原有车体矩形和厂家角度过滤。`/scan_raw` 仍保留原始数据。
实车 SLAM 扫描队列设为 10，允许等待 10 Hz EKF 的 TF；最小测距使用 0.2 m。

实车新增 `max_tracking_error:=0.20`：参考与实际平面位置偏差超过 20 cm，
从实际里程计位置、速度重新生成 B 样条，避免转向/停车时旧参考继续向前漂移。
这不是实车误差绝不会超过 20 cm 的保证，规划延迟及失败仍可能导致更大误差。

速度参数已开放：`max_vel:=0.12`（规划）、`max_speed:=0.15`（控制合速度上限）、
`max_yaw_rate:=0.25`（转向上限）。默认速度保持原值。
转向误差较大时仍按余弦缩小平移速度；不要只提高规划速度，否则会扩大跟踪滞后。
完成默认参数验证后，可尝试 `max_vel:=0.18 max_speed:=0.24 max_yaw_rate:=0.4`。

地图仍是累计 SLAM 地图，不是每帧雷达的动态障碍图；静止且未满足建图位移门槛时，
栅格内容可能不变化。实时扫描应查看 `/scan`，规划地图点云是 `/ego/cloud`。


## 2026-10-17 bag8 / bag9：实时规划障碍

实车 `ego_real.launch.py` 默认 `obstacle_source:=scan`。链路为：
`/scan_raw → 固定角度网格/车体与厂家过滤 → /scan → 扫描时刻 TF → /ego/cloud → EGO 膨胀地图`。
`wheeltec_ego_real` 的 scan_to_obstacles 使用同类逐帧点云；2VRosCar 也直接订阅扫描补充障碍，
但其叠加标记策略不适合解决本次旧障碍残留，因此不直接照搬。

桥最多等待原时刻 TF 0.5 秒，超时丢弃；没有扫描不重发旧点云。
合法空扫描会清除旧规划障碍；全 NaN 的无效扫描不会当作空旷，也不刷新感知。
厂家屏蔽方向包括 Inf 也转换为 NaN，表示该方向未知。
规划是当前观测的局部障碍快照，不提供遮挡区或历史障碍的持久记忆。

RViz 中 `Live Planning Obstacles` 是实时规划输入，`Inflated Obstacles` 是实际膨胀地图。
`SLAM History Map` 的 `/map` 是累计建图与定位用图，静止时可能不变，历史残影不会被本次
改动自动擦除；它在 scan 模式下不再作为局部规划障碍输入。需要旧行为时可传
`obstacle_source:=map`，仿真入口默认保留旧地图模式。

启动前退出旧终端，只保留一个硬件入口。默认厂家 full 已生效；重启导航重建 SLAM 地图。


## 2026-10-18：planning_horizon 生效与轨迹范围检查

实车入口已开放 `planning_horizon`，默认仍为 5.0 m。修改 advanced_param 中的默认值
不会覆盖外层 ego_real 的传参。推荐显式指定并重启导航进程，例如：

```bash
source /opt/ros/humble/setup.bash
source ~/wheeltec_ego_2d/install/setup.bash
ros2 launch robot_sim ego_real.launch.py rviz:=true enable_motion:=true planning_horizon:=1.5
```

底盘入口保持原来的单实例。1.5 m 是可用于复测的局部视野值，不代表经实车验证的最优值。
这是空间局部目标视野，不是整条曲线弧长；远目标由后续重规划推进。
新增控制点范围检查允许一个 ctrl_pt_dist 的边界余量（默认 0.4 m）。

运行后另开已 source 的终端核实：

```bash
ros2 param get /drone_0_ego_planner_node fsm/planning_horizon
ros2 param get /drone_0_ego_planner_node manager/planning_horizon
```

两项都应返回 1.5。通过 launch 参数修改无需重新编译，但必须重启对应 launch；
若修改 C++ 则必须重新编译。`ros2 param set` 不保证更新代码中初始化时缓存的参数，
因此这里使用启动参数。`ros2 pkg prefix robot_sim` 可核对所用工作区。

遇到“导航停车”日志，分别查看 scan、inflated_map 与 goal_distance；不要把碰撞停车
当成限速过低。本轮两端构建和无硬件预览已通过，尚无本轮实车闭环验证。


## 2026-10-24：bag14 速度、连续性与膨胀修正

实车默认控制点间距为 0.10 m，规划限速仍为 0.12 m/s，控制上限仍为 0.15 m/s。
滚动采样间距与时间步长同步，样条拟合固定起终端位置/速度/加速度；
局部中间目标使用受限巡航速度，最终目标保持停车要求。新旧曲线按有效段凸包检查范围。
`planning_horizon` 仍是局部空间视野，不是弧长或恒速保证。

点云膨胀改为圆与栅格相交判断，默认保留 bag14 的 0.01 m 设置。
该设置不等于完整车体安全包络，控制器仍独立使用 0.25 m 雷达安全圆，
因此规划可通过的近障位置仍可能被控制器拦停；不要通过关闭碰撞检查解决。
RViz 默认使用自动匹配格宽的 `inflated_cells` Marker，旧固定格宽点云显示关闭。

只保留一个硬件入口，退出旧导航后，在小车终端启动：

```bash
source /opt/ros/humble/setup.bash
source ~/wheeltec_ego_2d/install/setup.bash
ros2 launch robot_sim ego_real.launch.py rviz:=true enable_motion:=true planning_horizon:=1.5 control_points_distance:=0.10 obstacles_inflation:=0.01 map_resolution:=0.10
```

需要更细的膨胀栅格时，可改 `map_resolution:=0.05`；规划器、控制器和标记格宽同步更新。
圆形半径仍存在栅格离散余量，不是显示一个精确的连续圆。
参数都在启动时读取，修改后重启导航；命令行改这些参数无需重新编译。

```bash
ros2 param get /drone_0_ego_planner_node manager/control_points_distance
ros2 param get /drone_0_ego_planner_node grid_map/obstacles_inflation
ros2 param get /drone_0_ego_planner_node grid_map/resolution
ros2 param get /ground_traj_server grid_resolution
```

合成模型中 4 m 转向远目标和 2 m 单障碍绕行通过，详情在 `diagnostics/bag14/REPORT.md`。
本轮未做物理实车闭环验证，不能承诺实际速度恒等于规划上限。
