# plan_manage 中文阅读指南

## 1. 这个包到底负责什么

`plan_manage` 的 ROS 2 包名是 `ego_planner`。它不是只做某一种路径算法，而是整个规划系统的“总调度层”：接收里程计、目标、局部地图和其他无人机轨迹，决定何时开始规划、何时滚动重规划、何时紧急停止，再把生成的 B 样条交给轨迹服务器执行。

可以把各层理解成：

```text
目标 + 里程计 + 地图 + 其他无人机轨迹
                 │
                 ▼
        EGOReplanFSM（决定什么时候做）
                 │
                 ▼
      EGOPlannerManager（组织一次规划）
          │                  │
          ▼                  ▼
  path_searching/A*    bspline_opt/轨迹优化
          └──────────┬───────┘
                     ▼
               B 样条轨迹消息
                     ▼
        traj_server（按时间求位置、速度、加速度）
                     ▼
        position_cmd → 控制器/仿真器 → 里程计
```

## 2. 文件分工

| 文件 | 作用 |
|---|---|
| `src/ego_planner_node.cpp` | 规划进程的 `main()`，创建 ROS 2 节点、初始化 FSM 并进入事件循环。 |
| `include/ego_planner/ego_replan_fsm.h` | FSM 类、状态枚举、订阅器/发布器、定时器和运行数据的声明。 |
| `src/ego_replan_fsm.cpp` | 状态切换、目标处理、滚动重规划、碰撞检查和多机轨迹通信。 |
| `include/ego_planner/planner_manager.h` | 一次规划需要的参数、全局/局部轨迹数据和接口声明。 |
| `src/planner_manager.cpp` | 构造初始轨迹、调用优化器、检查可行性、更新时间参数。 |
| `src/traj_server.cpp` | 第二个独立程序；接收 B 样条并周期发布 `PositionCommand`。 |
| `launch/advanced_param.launch.py` | 规划节点最集中的参数表和话题重映射。 |
| `launch/run_in_sim.launch.py` | 一架无人机的可复用仿真单元。 |
| `launch/single_run_in_sim.launch.py` | 单机演示总入口。 |
| `launch/simulator.launch.py` | 控制器、动力学/简化模型、里程计显示和局部深度传感器。 |
| `launch/swarm.launch.py` | 10 机仿真入口。 |
| `launch/swarm_large.launch.py` | 21 机大规模仿真入口。 |
| `launch/rviz.launch.py` | 使用 `default.rviz` 启动 RViz2。 |
| `launch/default.rviz` | RViz 自动生成的显示配置，不是算法代码，通常在 RViz GUI 中修改。 |
| `CMakeLists.txt` | 把三个规划源码链接成 `ego_planner_node`，并构建 `traj_server`。 |
| `package.xml` | ROS 2 包名、构建方式和依赖清单。 |

## 3. 两个真正运行的程序

这个包会生成两个可执行程序。

### 3.1 `ego_planner_node`

入口是 `src/ego_planner_node.cpp::main()`。它自己几乎不做计算，只调用：

```text
main()
 ├─ rclcpp::init()
 ├─ 创建 rclcpp::Node
 ├─ EGOReplanFSM::init(node)
 └─ rclcpp::spin(node)
```

`init()` 注册一个 10 ms 的 FSM 定时器和一个 50 ms 的安全检查定时器。此后真正的在线逻辑都由 ROS 回调驱动。

### 3.2 `traj_server`

入口是 `src/traj_server.cpp::main()`。它订阅 `planning/bspline`，收到新轨迹后构造位置 B 样条及一、二阶导数。10 ms 定时器根据 `当前时间 - 轨迹开始时间` 求出：

- 期望位置，m；
- 期望速度，m/s；
- 期望加速度，m/s²；
- 偏航角与偏航角速度。

这些量被封装进 `quadrotor_msgs::msg::PositionCommand`，再交给 SO3 控制器或简化仿真器。

## 4. FSM 六个状态

| 状态 | 含义 | 常见转移 |
|---|---|---|
| `INIT` | 刚启动，等待有效里程计 | 有里程计 → `WAIT_TARGET` |
| `WAIT_TARGET` | 等待目标点/航点 | 收到目标 → `GEN_NEW_TRAJ` |
| `GEN_NEW_TRAJ` | 从当前状态生成第一条局部轨迹 | 成功 → `EXEC_TRAJ`；失败则稍后重试 |
| `REPLAN_TRAJ` | 以正在执行轨迹上的状态为起点重新规划 | 成功 → `EXEC_TRAJ`；失败继续尝试 |
| `EXEC_TRAJ` | 正常执行并判断是否该重规划 | 执行时间、距离或安全条件满足 → `REPLAN_TRAJ` |
| `EMERGENCY_STOP` | 预计很快碰撞时发布停留轨迹 | 若启用 fail-safe 且已减速 → 尝试恢复 |

FSM 每 10 ms 执行一次，但不表示每 10 ms 都优化轨迹。大多数周期只检查状态和条件，只有进入 `GEN_NEW_TRAJ` 或 `REPLAN_TRAJ` 才调用规划器。

## 5. 一次局部规划如何产生

1. `getLocalTarget()` 沿全局轨迹向前采样，选择距离当前位置约一个 `planning_horizon` 的局部目标；如果已经接近最终目标，就直接使用最终目标。
2. `callReboundReplan()` 决定初值来源。第一次通常用多项式连接当前状态和局部目标；滚动重规划优先复用旧轨迹的一段。
3. `EGOPlannerManager::reboundReplan()` 把初值离散成控制点和节点间隔 `ts`。
4. `BsplineOptimizer::BsplineOptimizeTrajRebound()` 优化平滑、碰撞、动力学可行性和多机间距等代价。
5. 若轨迹速度或加速度超限，`refineTrajAlgo()` 通过延长时间和重新参数化继续优化。
6. 成功后保存为 `local_data_`，FSM 发布 `Bspline` 消息，并在多机模式下广播给其他规划器。

这里的“rebound”可以粗略理解为：控制点进入障碍物附近后，优化器根据障碍物方向给它一个向安全侧弹开的梯度，同时兼顾平滑和物理限制。

## 6. 全局轨迹与局部轨迹

全局轨迹只负责提供从当前点经过所有航点的大方向和局部目标，不直接发给飞控。局部轨迹才是无人机短时间内真正执行的轨迹，并会不断向前滚动更新。

- `global_data_`：全局多段多项式轨迹、总时长、最后进度等；
- `local_data_`：当前局部 B 样条、导数、开始时间、轨迹编号和时长；
- `planning_horizon`：空间上的局部前视距离，不是整条航线长度。

## 7. 主要话题

下表写的是代码中的基础名；launch 后通常会带 `drone_<id>_` 前缀。

| 方向 | 基础话题 | 消息/作用 |
|---|---|---|
| 输入 | `odom_world` | `nav_msgs/Odometry`，当前位置、速度和姿态。 |
| 输入 | `/move_base_simple/goal` | RViz “2D Goal Pose” 给出的手动目标。 |
| 输入 | `/traj_start_trigger` | 实机模式的外部启动许可。 |
| 输入 | `planning/broadcast_bspline_to_planner` | 其他无人机广播的 B 样条。 |
| 输出 | `planning/bspline` | 本机局部 B 样条，供 traj_server 执行。 |
| 输出 | `planning/broadcast_bspline_from_planner` | 本机轨迹广播给全体规划器。 |
| 输出 | `planning/data_display` | 规划耗时等统计数据。 |
| 输出 | `planning/position_cmd` | traj_server 生成的实时位置指令。 |
| 输入 | `grid_map/depth` / `grid_map/cloud` | 深度图或点云，用于构建局部占据地图。 |
| 输出 | `grid_map/occupancy_inflate` | 膨胀后的占据点云，主要供 RViz 查看。 |

## 8. 多机逻辑

每架无人机独立运行一套规划器和轨迹服务器，但所有规划器共享 `/broadcast_bspline`。收到其他无人机轨迹后，本机优化器把它们当作随时间移动的安全约束。

这个实现有几个重要约定：

- `drone_id` 从 0 开始、唯一、连续；
- 无人机 i 在初次规划前会检查编号较小的无人机轨迹，形成顺序启动；
- 轨迹消息带 `start_time`，多机安全判断要求各主机时间同步；
- `swarm_clearance` 表示优化希望保持的机间净空，不应小于真实机体安全需求。

## 9. launch 调用层级

单机常用：

```text
single_run_in_sim.launch.py
 ├─ 地图生成器
 ├─ advanced_param.launch.py → ego_planner_node
 ├─ traj_server
 └─ simulator.launch.py
     ├─ SO3 控制器 + 动力学模型（use_dynamic=true）
     │  或 poscmd_2_odom（use_dynamic=false）
     ├─ odom_visualization
     └─ pcl_render_node
```

多机常用：

```text
swarm.launch.py / swarm_large.launch.py
 ├─ 一份共享地图
 └─ 针对每个 drone_config 调用 run_in_sim.launch.py
     ├─ advanced_param.launch.py
     ├─ traj_server
     └─ simulator.launch.py
```

## 10. 建议阅读顺序

基础较弱时建议按这个顺序阅读：

1. `ego_planner_node.cpp`：先确认程序如何启动；
2. `ego_replan_fsm.h`：认识状态和成员变量；
3. `EGOReplanFSM::init()`：看参数、话题和定时器；
4. `execFSMCallback()`：掌握状态流转；
5. `callReboundReplan()`：看 FSM 如何调用规划管理器；
6. `planner_manager.h`：区分全局数据、局部数据和配置参数；
7. `reboundReplan()`：理解一次规划的完整过程；
8. `traj_server.cpp`：理解规划轨迹怎样变成控制指令；
9. 最后再读 `bspline_opt` 和 `path_searching` 的算法细节。

## 11. 当前代码中值得留意的地方

- 参数名 `planning_horizen_time` 中的 `horizen` 是原代码拼写，配置时必须保持一致。
- `planning_horizen_time_` 当前被读取和保存，但局部目标选择主要使用空间 `planning_horizen_`。
- `manager/max_jerk` 当前主要被保存；主要可行性检查集中在速度和加速度。
- 动态物体预测器参数存在，但优化器中 moving-object 代价在当前版本被注释；不能只因设置了 `prediction/*` 就认为动态障碍避碰已启用。
- `run_in_sim.launch.py` 和两个 swarm launch 传递了 `use_dynamic_cmd`，而子文件声明的是 `use_dynamic`；动力学开关不生效时首先检查这一命名不一致。
- `run_in_sim.launch.py` 创建了 `obj_generator_node`，但 `add_action` 被注释，所以它不会运行。
- 手动目标回调把目标高度固定成 1 m；RViz 输入的 z 值当前不会直接成为最终高度。
- `default.rviz` 有数千行，是 RViz 保存出来的配置数据，不建议像 C++ 一样逐行阅读。

更完整的参数说明见 [PARAMETERS_ZH.md](PARAMETERS_ZH.md)。
