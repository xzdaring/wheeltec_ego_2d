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
行驶阶段车头跟随 B 样条切线，并保留麦轮横移纠偏能力；接近目标后原地转向，使用点击箭头作为最终朝向约束。

真实链路：

```text
Gazebo /scan + odom->base_link->laser
  -> slam_toolbox -> /map + map->odom
  -> ego_navigation_bridge -> /ego/cloud（odom，z=0）
RViz /goal_pose（map）-> bridge -> /move_base_simple/goal（odom）
/odom + /ego/cloud + goal -> EGO FSM -> /drone_0_planning/bspline
  -> ground_traj_server（B 样条求值 + 全向反馈控制）-> /cmd_vel
  -> /ego/position_cmd（期望状态调试输出，不参与控制转发）
```

EGO 的局部地图仍是三维数据结构，但轨迹和障碍都限制在 z=0 平面。
SLAM 累计已观测地图，桥每 0.2 秒把车旁 10×10 m 已占据栅格变换到 odom 后交给 EGO。
未知栅格不视为障碍，支持朝未知区域推进；这不是保证全局可达的探索器，死胡同或复杂迷宫可能需要换目标。
定位仍使用仿真里程计，SLAM 的地图修正用于地图与目标转换。没有预存地图依赖。
SLAM 地图更新有延迟，当前雷达另做车体圆形包络短期扫掠检查，近障立即停车等重规划。
地图超过 3 秒、雷达/里程计超过 0.5 秒、转换点云超过 0.7 秒未更新都会输出零速度。
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
Python 桥只负责 TF、QoS 和地图投影；新增 C++ 小车服务器负责轨迹求值、前馈反馈和停车分支。

仿真雷达的扫描平面现通过 `laser -> laser_scan` 固定关节抬高 0.15 m，避开 CAD 车壳；这是仿真安装适配，不代表实车雷达安装高度。原 `laser` 外形和 TF 保留。


## 小车版 EGO 轨迹服务器

`planner/plan_manage/src/ground_traj_server.cpp` 替代导航启动链中的原 `traj_server`。
原无人机 `traj_server.cpp` 保留；两者不能同时接入同一导航速度链。

```text
onSpline：读取 EGO 三次 B 样条，校验平面高度，解析求导。
onScan：将雷达点转换到 base_link，用于短期扫掠停车。
control：按节点时间求值 → 速度前馈加位置反馈 → 合速度限幅 → 切线转向与减速 → 车体坐标转换。
```

参数位于 `/ground_traj_server`：`kp_xy=0.8`、`max_speed=0.3`、
`goal_tolerance=0.12`、`safety_radius=0.22`，距离单位 m，速度单位 m/s。
`follow_trajectory_yaw=true` 时，使用前方 `yaw_lookahead=0.3` s 的轨迹切线作为车头参考，
由曲线速度和加速度计算角速度前馈，再叠加 `kp_yaw` 的最短角差反馈。
朝向误差增大时按 `max(0, cos(error))` 降低平移速度，超过 90° 时先转向；横移仍用于纠偏。
常值急停轨迹不会生成切线转向；到点后最终朝向仍以点击箭头为准。
雷达与膨胀地图预测均计入同时平移和转向的圆弧位移，车体包络仍使用外接圆。
仿真角速度上限为 0.6 rad/s，实车入口为 0.25 rad/s；尚未加入加速度限制。
实车入口可用 `follow_trajectory_yaw:=false` 恢复原平移方式，也可用 `yaw_lookahead:=0.5` 调整预瞄时间。
`/ego/position_cmd` 的 `yaw` 显示期望车头，`yaw_dot` 显示控制器下发的角速度。
轨迹时间仍按 ROS 时钟推进，大角差导致减速时需要靠滚动重规划与位置反馈重新收敛，不能保证任意急弯都能无误差跟踪。
节点关闭时发送零速度属于尽力操作，底盘插件没有速度看门狗时不能保证进程崩溃后自动停车。

重新构建后，启动命令不变：

```zsh
CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS=-j2 colcon build --packages-select ego_planner robot_sim --cmake-args -DBUILD_TESTING=OFF
source install/setup.zsh
ros2 launch robot_sim ego_navigation.launch.py
```

## 导航可视化图例

- 青色细线：EGO 全局参考轨迹，提供总体进度，不保证全局避障。
- 橙色粗线：按完整 B 样条节点向量采样的真实局部轨迹，不再用控制点连线代替。
- 白色点：局部 B 样条控制点，通常不在实际曲线上。
- 黄色箭头：局部速度方向，长度表示约半秒的速度尺度。
- 黄色小球：当前跟踪参考；绿色球：当前局部末端；紫色球：最终目标。
- 蓝色线：里程计记录的实际行驶路径，最多保留 2000 点。
- 半透明紫色体素：EGO 真正使用的膨胀占据地图。

小车旁文字实时读取规划节点参数：膨胀半径、地图分辨率、XY 膨胀格数、
规划视野和规划限速，并显示轨迹编号、控制点数量、局部长度、时长和采样峰值速度。
当前膨胀参数是距离（m），不是无量纲系数；代码采用 XY 方形栅格膨胀。
峰值速度为曲线采样有限差分估计值，不能替代动力学可行性检查。
可在 RViz 的 Displays 中关闭白色控制点或膨胀图层以减少遮挡。
全局参考由显示节点缓存重发，RViz 晚打开仍能看到；显示节点必须在目标下发前运行。

## 边界墙修复与最终朝向

本工作区新增 Humble 版 `src/slam_toolbox`，只修改 Karto 地图宽高计算：
向上取整后加一格，保留最大 X/Y 边界的激光命中。系统 /opt/ros 安装不变。
源码版本与修改说明见 `src/slam_toolbox/LOCAL_PATCHES.md`。

```zsh
source /opt/ros/humble/setup.zsh
CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS=-j2 colcon build --packages-up-to robot_sim --parallel-workers 2 --allow-overriding slam_toolbox --cmake-args -DBUILD_TESTING=OFF
source install/setup.zsh
ros2 pkg prefix slam_toolbox
ros2 launch robot_sim ego_navigation.launch.py
```

包前缀应为当前工作区的 `install/slam_toolbox`；旧进程必须关闭再启动才会使用新库。
已录制包中的旧 `/map` 不会自动改变，验证修复需要用原雷达与 odom TF 重新运行 SLAM。

RViz 点击拖动箭头后，位置与四元数同时转换到 odom。
小车先平移接近目标，再原地转向；最终位置容差 0.12 m，角度容差 0.05 rad（约 2.9°）。
转向阶段以更小的位置误差进入并保留滞回，避免平移/转向反复切换。
朝向参数：`kp_yaw=1.5`、`max_yaw_rate=0.6` rad/s、`yaw_tolerance=0.05` rad。
障碍包络内有雷达命中时禁止旋转，感知中断仍会停车。
紫色目标箭头显示已经变换后的最终方向。

平面底盘插件更新率现为 1000 Hz，与世界 0.001 s 物理步对应，
减少直接速度设定之间轮地摩擦对转向的削弱；它仍是简化平面模型，
不用于验证真实麦轮接触、轮速或打滑。
EGO 最终曲线自由末端可有少量位置偏差，因此最后一小段增加到点击位置的反馈收敛，
仅在轨迹结束且末端距最终目标小于 0.24 m 时启用，并保留雷达停车检查。

## ego_test_world2 的离线模型资源

`.world` 保存模型位姿与描述，但 `model://...` 引用的网格、材质和纹理需要单独存在。
本包已收录该世界用到的六类资源：纸箱、咖啡桌、车轮、易拉罐、垃圾桶和大理石桌。
资源位于 `models/`，来源版本记录在 `models/SOURCES.md`，随 robot_sim 一起安装。
启动文件将安装后的 models 目录加入 GAZEBO_MODEL_PATH，保留用户已有目录。
无需依赖个人 ~/.gazebo 缓存或开启在线模型数据库。

新增或修改模型资源后构建 `robot_sim`，关闭旧 Gazebo，再 source 和重新启动。
只重启界面、继续连接旧 gzserver，不会重新加载世界和新资源。
若直接运行 gazebo 命令而不使用 launch，需要自行设置 GAZEBO_MODEL_PATH。

## 实车运行

V550_mec + N10P 的传输、编译、硬件启动和 EGO 操作见 [REAL_ROBOT.md](REAL_ROBOT.md)。

### 三航点顺序仿真

```bash
source /opt/ros/humble/setup.bash
source /ros_workspace/wheeltec_ego_2d/install/setup.bash
ros2 launch robot_sim ego_navigation.launch.py run_waypoints:=true
```

航点位于 `config/waypoints.yaml`，默认按 `map` 坐标发送：
`(-0.5,-6.5,0.08,-1.57)` → `(-10,-3,0.08,3.14)` → `(-1,4,0.08,0)`。
每组为 x/y/z（米）与 yaw（弧度）。地面规划器仍使用二维 z=0；0.08是目标消息高度。

每个航点由控制器确认位置和朝向均已达标，并发布 `/ego/goal_reached`（PoseStamped，保留已接受目标的时间戳与位姿）。航点程序核对完成消息对应当前目标后，等待新鲜里程计的平移速度小于0.02米/秒、角速度小于0.03弧度/秒并持续1秒，再发送下一点。停车后的微小姿态漂移不会推翻控制器已经给出的完成结果；旧目标的完成消息不会触发新航点切换。到达判断使用桥转换后的目标及 odom，与控制器一致。状态通过 `/ego/waypoint_status` 发布。

普通启动默认 `run_waypoints:=false`，仍由 RViz 手动给目标。执行航点期间不要同时在RViz发送另一目标。单点超过300秒仿真时间时不再发送后续航点；当前导航目标仍有效，需要停止时关闭本次导航启动进程。

### 自动测试报告脚本

工作区根目录执行 `./scripts/test_ego_sim.sh`，脚本自动启动仿真并依次执行可修改的航点，结束后输出总耗时、各段耗时、失败航段及相关日志。详细用法见工作区 `scripts/README_sim_test.md`。目标配置可通过 `--waypoints /路径/配置.yaml` 指定；只检查配置用 `--validate-only`，不启动图形界面用 `--headless`。


### 仿真启动扫描保护

`v550_sim.launch.py` 启用 `scan_self_filter` 的 `sim_settling_gate`。扫描按采样时刻的odom→base_link TF检查：高度0～0.04m、roll/pitch绝对值不超过2°、垂直速度绝对值不超过0.003m/s，并连续稳定1秒后，才输出/scan。异常时不发布空白扫描，而是停止发布，使控制器的感知看门狗停车；姿态恢复后重新等待稳定。该高度条件适用于本项目平地V550模型。实车默认关闭该功能。

仿真航点节点设置 `wait_for_scan=true`，等待新鲜/scan后才发送目标，避免模型仍在落地时开始导航。初次启动等待落稳是正常现象。已有污染地图需要关闭旧导航并重新启动SLAM建图，本次保护不会追溯清除旧地图。


### 100次批量测试

`./scripts/test_ego_sim.sh --runs 100 --headless` 会独立重启并测试100次，失败后继续下一次；Ctrl+C停止整个批次。脚本默认次数已为100，单次调试用 `--runs 1`。批次目录下summary.txt/json为汇总，runs.csv、segments.csv为每轮/每段明细，run_001～run_100分别保留完整报错日志。目标配置使用批次启动时的快照，100轮保持一致。

### 轨迹图与图形界面

测试脚本每轮结束保存trajectory.png（实际轨迹、已发送目标、最后局部规划及可用地图背景）和trajectory.csv。批量时位于各run_NNN目录。`./scripts/test_ego_sim.sh --runs 1`自动打开Gazebo和RViz；增加`--headless`关闭界面，仍会保存轨迹图。绘图失败见plot_error.txt，不覆盖原导航结果。


### 当前测试输出位置与命名

后续测试统一默认保存到工作区 `test_data/<批次时间戳>/`。单轮结果按 `001_成功_123.45秒`、`002_失败`、`003_中断`命名；图片和日志放在各自目录。成功时间指首个目标发出到最后航点完成并停稳的实际全程时间，不含启动和清理。此前diagnostics中的历史结果不自动移动。此规则替代上文旧的输出路径说明。
