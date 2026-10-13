```zsh
cd /ros_workspace/wheeltec_ego_2d
source /opt/ros/humble/setup.zsh
source install/setup.zsh

ros2 launch robot_sim v550_sim.launch.py x:=1.0 y:=2.0
```
启动机器人仿真环境，设置初始位置为 (1.0, 2.0)。



```zsh
cd /ros_workspace/wheeltec_ego_2d
source /opt/ros/humble/setup.zsh
source install/setup.zsh

ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.0}, angular: {z: 0.5}}"
```

## 二维激光仿真

`urdf/v550_sim.urdf.xacro` 在 `laser` link 上添加了 Gazebo ray 传感器，
通过 `libgazebo_ros_ray_sensor.so` 发布 `/scan`（`sensor_msgs/msg/LaserScan`），
`frame_id` 为 `laser_scan`。仿真参数为 10 Hz、360 度、720 束、0.10–12 m，
用于功能验证，不代表实物雷达规格。世界物体必须有 collision 才能被检测。

修改源码后重新构建，并关闭旧仿真、重新生成机器人：

```zsh
cd /ros_workspace/wheeltec_ego_2d
source /opt/ros/humble/setup.zsh
colcon build --packages-select robot_sim
source install/setup.zsh
ros2 launch robot_sim v550_sim.launch.py
```

在另一个已加载 ROS 环境的终端分别检查（持续命令按 Ctrl+C 结束）：

```zsh
ros2 topic info /scan --verbose
ros2 topic hz /scan
ros2 topic echo /scan --qos-reliability best_effort --once
ros2 run tf2_ros tf2_echo base_link laser_scan
```

RViz 设置 Fixed Frame 为 `odom`，添加 LaserScan，Topic 选择 `/scan`，
Reliability 选择 Best Effort；建议 RViz 设置 use_sim_time=true。
应看到与墙壁位置一致的扫描点；机器人旋转后墙壁在 odom 中应保持原位。
无障碍物方向可能显示 inf，这是未命中量程内物体的正常结果。

这一步仅完成雷达仿真；EGO 不直接消费 LaserScan，后续仍需点云和坐标转换适配。

## RViz 点击目标 + 在线 SLAM + EGO 二维导航

构建本启动链及其源码依赖（不启动实车串口、硬件雷达或 Nav2）：

```zsh
cd /ros_workspace/wheeltec_ego_2d
source /opt/ros/humble/setup.zsh
CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS=-j2 colcon build --packages-up-to robot_sim --parallel-workers 2 --cmake-args -DBUILD_TESTING=OFF -DAMENT_CMAKE_SYMLINK_INSTALL=OFF
source install/setup.zsh
ros2 launch robot_sim ego_navigation.launch.py
```

关闭此前单独启动的 Gazebo、规划器和速度发布命令，避免重复实体或多个 `/cmd_vel` 发布者。
默认出生在 `(1, 2)`，可通过 `x:=... y:=...` 调整到世界空地。
等待 RViz 出现 `/map` 后，用工具栏 `2D Goal Pose` 在可达空地点击并拖动。
当前验收到达位置，麦轮车保持现有朝向并允许横移，不使用点击箭头作为最终朝向约束。

真实链路：

```text
Gazebo /scan + odom->base_link->laser
  -> slam_toolbox -> /map + map->odom
  -> ego_navigation_bridge -> /ego/cloud（odom，z=0）
RViz /goal_pose（map）-> bridge -> /move_base_simple/goal（odom）
/odom + /ego/cloud + goal -> EGO FSM -> /drone_0_planning/bspline
  -> traj_server -> /ego/position_cmd -> bridge 全向反馈控制 -> /cmd_vel
```

EGO 的局部地图仍是三维数据结构，但轨迹和障碍都限制在 z=0 平面。
SLAM 累计已观测地图，桥每 0.2 秒把车旁 10×10 m 已占据栅格变换到 odom 后交给 EGO。
未知栅格不视为障碍，支持朝未知区域推进；这不是保证全局可达的探索器，死胡同或复杂迷宫可能需要换目标。
定位仍使用仿真里程计，SLAM 的地图修正用于地图与目标转换。没有预存地图依赖。
SLAM 地图更新有延迟，当前雷达另做车体圆形包络短期扫掠检查，近障立即停车等重规划。
地图超过 3 秒、雷达/里程计/指令超过 0.5 秒未更新都会输出零速度。
基础跟踪器限合速度 0.3 m/s，EGO 规划速度 0.25 m/s；定位输入或速度输出话题不能混入实车驱动。
当前限速与近障停车不是完整动态障碍预测，也没有自动恢复倒车或全局拓扑搜索。

检查：

```zsh
ros2 topic hz /map
ros2 topic hz /ego/cloud
ros2 topic echo /drone_0_planning/bspline --once
ros2 topic echo /ego/position_cmd --once
ros2 topic echo /cmd_vel --once
ros2 run tf2_ros tf2_echo map base_link
```

B 样条所有控制点 z 应为零；位置命令 z、竖直速度和加速度应接近零。
RViz 中 EGO 标记位于 odom，由 TF 转到 map 显示，不能靠改 frame_id 假装完成坐标转换。
新增脚本逐行中文注释，解释 TF、QoS、地图投影、前馈反馈和停车分支。

仿真雷达的扫描平面现通过 `laser -> laser_scan` 固定关节抬高 0.15 m，避开 CAD 车壳；这是仿真安装适配，不代表实车雷达安装高度。原 `laser` 外形和 TF 保留。
