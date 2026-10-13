# plan_manage 参数中文手册

## 单位约定

| 类型 | 单位 |
|---|---|
| 坐标、距离、地图尺寸、分辨率 | m |
| 速度 | m/s |
| 加速度 | m/s² |
| jerk | m/s³ |
| 时间 | s |
| 频率 | Hz |
| 相机内参、图像边界 | pixel |
| 概率 | 0～1 |

## FSM 参数

| 参数 | 默认演示值 | 含义与调节影响 |
|---|---:|---|
| `fsm/flight_type` | 2 | 1 为 RViz 手动目标，2 为预设航点；3 的引用路径在当前版本未完整实现。 |
| `fsm/thresh_replan_time` | 1.0 s | 当前轨迹执行超过此时间后允许滚动重规划。减小会更灵敏，也会增加计算量和轨迹切换频率。 |
| `fsm/thresh_no_replan_meter` | 1.0 m | 距最终目标小于此值时减少重规划，避免终点附近反复变化。过大会过早停止更新。 |
| `fsm/planning_horizon` | 7.5 m | 从当前位置向全局轨迹前方选择局部目标的空间距离。过大增加规划难度，过小可能目光短浅。 |
| `fsm/planning_horizen_time` | 3.0 s | 原代码保留的时间前视参数；拼写必须保持 `horizen`。当前主流程以空间 horizon 为主。 |
| `fsm/emergency_time` | 1.0 s | 预测在这么短时间内发生碰撞时进入急停。增大更保守，也更容易提前停车。 |
| `fsm/realworld_experiment` | false | true 时还需收到 `/traj_start_trigger` 才接受目标，适合实机安全流程。 |
| `fsm/fail_safe` | true | 急停且速度降下来后，是否自动尝试恢复规划。 |
| `fsm/waypoint_num` | 1 或 4 | 有效预设航点个数。advanced launch 显式提供最多 5 个。 |
| `fsm/waypointN_x/y/z` | 由启动文件给定 | 第 N 个航点的世界坐标，单位 m。 |

## PlannerManager 参数

| 参数 | 演示值 | 含义与调节影响 |
|---|---:|---|
| `manager/max_vel` | 2.0 m/s | 各轴最大速度。必须符合真实飞行器能力；越大轨迹时间可越短。 |
| `manager/max_acc` | 3.0 或 6.0 m/s² | 各轴最大加速度。过大可能导致真实控制器跟不上。 |
| `manager/max_jerk` | 4.0 m/s³ | jerk 配置上限；当前 manager 主要保存它，核心检查仍以速度/加速度为主。 |
| `manager/control_points_distance` | 0.4 m | 初始轨迹相邻控制点的期望距离。减小会增加控制点和计算量，但表达更细。 |
| `manager/feasibility_tolerance` | 0.05 | 动力学可行性检查余量，0.05 约表示允许 5% 数值容差。 |
| `manager/planning_horizon` | 7.5 m | manager 保存的局部规划范围，应与 FSM 的同名空间参数一致。 |
| `manager/use_distinctive_trajs` | true | 是否生成从障碍物不同侧绕行的候选；开启可能提高成功率，也更耗时。 |
| `manager/drone_id` | 0…N-1 | 多机编号，也是数组索引和话题前缀的一部分。 |

## 优化器参数

| 参数 | 演示值 | 含义 |
|---|---:|---|
| `optimization/lambda_smooth` | 1.0 | 平滑代价权重，主要抑制控制点高阶差分和 jerk。 |
| `optimization/lambda_collision` | 0.5 | 静态障碍、机间避碰等碰撞相关代价权重。 |
| `optimization/lambda_feasibility` | 0.1 | 速度、加速度超限惩罚权重。 |
| `optimization/lambda_fitness` | 1.0 | 精修时贴近参考轨迹的权重。 |
| `optimization/dist0` | 0.5 m | 希望轨迹与静态障碍至少保持的优化净空。 |
| `optimization/swarm_clearance` | 0.5 m | 希望不同无人机之间保持的距离。实机要结合机体尺寸、定位误差和下洗气流重新设置。 |
| `optimization/max_vel` | 同 manager | 优化器用于计算可行性代价的速度限制。 |
| `optimization/max_acc` | 同 manager | 优化器用于计算可行性代价的加速度限制。 |

`lambda` 之间不能只比较数字大小，因为每项原始代价的量纲和数值范围不同。调参时应一次只改一类目标，并观察轨迹、最小距离、速度/加速度曲线和规划耗时。

## B 样条参数

| 参数 | 演示值 | 含义 |
|---|---:|---|
| `bspline/limit_vel` | 2.0 m/s | B 样条可行性检查的速度上限。 |
| `bspline/limit_acc` | 3.0 或 6.0 m/s² | B 样条可行性检查的加速度上限。 |
| `bspline/limit_ratio` | 1.1 | 超限后建议时间伸缩时使用的安全比例。越大越保守、轨迹越慢。 |

## GridMap 参数

| 参数 | 演示值 | 含义与注意事项 |
|---|---:|---|
| `grid_map/resolution` | 0.1 m | 体素边长。减半后体素数量大致按三维立方增长，内存和计算量会显著上升。 |
| `grid_map/map_size_x/y/z` | launch 给定 | 全局地图边界尺寸。应覆盖所有起点、目标和障碍物。 |
| `grid_map/local_update_range_x/y/z` | 5.5/5.5/4.5 m | 围绕当前传感器更新的局部范围。不能明显小于有效感知距离和规划需求。 |
| `grid_map/obstacles_inflation` | 0.099 m | 将障碍向外膨胀的距离。实际安全距离还受 `dist0` 与机体尺寸影响。 |
| `grid_map/local_map_margin` | 10 voxel | 局部边界额外保留的体素层数。 |
| `grid_map/ground_height` | -0.01 m | 地面的世界坐标高度。 |
| `grid_map/cx/cy/fx/fy` | 相机标定值 | 针孔相机内参。错误值会让深度点云位置发生系统偏差。 |
| `grid_map/use_depth_filter` | true | 是否对深度图做有效范围、边缘和一致性过滤。 |
| `grid_map/depth_filter_tolerance` | 0.15 m | 深度一致性容差；太小会丢点，太大可能保留噪声。 |
| `grid_map/depth_filter_maxdist` | 5.0 m | 最大有效深度。通常不应超过传感器可靠距离。 |
| `grid_map/depth_filter_mindist` | 0.2 m | 最小有效深度，过滤近距离无效值。 |
| `grid_map/depth_filter_margin` | 2 pixel | 不处理图像四周的像素边界。 |
| `grid_map/k_depth_scaling_factor` | 1000 | 将深度原始整数换算为 m；常见 uint16 深度以 mm 保存，因此除以 1000。 |
| `grid_map/skip_pixel` | 2 | 深度图采样步长。增大会降低计算量，也会漏掉细障碍物。 |
| `grid_map/p_hit` | 0.65 | 射线端点命中后提高占据概率的模型值。 |
| `grid_map/p_miss` | 0.35 | 射线穿过后降低占据概率的模型值。 |
| `grid_map/p_min/p_max` | 0.12/0.90 | 占据概率累计的上下限，防止历史证据无限饱和。 |
| `grid_map/p_occ` | 0.80 | 超过该概率的体素被认为占据。降低会更保守。 |
| `grid_map/min_ray_length` | 0.1 m | 短于此长度的射线不融合。 |
| `grid_map/max_ray_length` | 4.5 m | 射线融合的最大长度。 |
| `grid_map/virtual_ceil_height` | 2.9 m | 人工设置的飞行天花板。 |
| `grid_map/visualization_truncate_height` | 1.8 m | RViz 占据点显示截断高度，只影响显示理解，不等同于飞行上界。 |
| `grid_map/show_occ_time` | false | 是否输出/显示占据更新耗时信息。 |
| `grid_map/pose_type` | 1 | 1 使用 `PoseStamped` 相机位姿；2 使用 `Odometry`。 |
| `grid_map/frame_id` | `world` | 地图和规划使用的坐标系名称。 |

## 轨迹服务器参数

| 参数 | 演示值 | 含义 |
|---|---:|---|
| `traj_server/time_forward` | 1.0 s | 计算偏航方向时，用当前位置指向轨迹前方多少秒处；值大则机头更早朝向远处路径。它不是碰撞检查时间。源码默认 -1，实际 launch 提供 1.0。 |

traj_server 的位置指令发布周期在源码中固定为 10 ms，即约 100 Hz；偏航角速度上限在源码中设为 π rad/s，并对偏航角和角速度做了简单低通滤波。

## 动态物体预测参数

| 参数 | 演示值 | 含义 |
|---|---:|---|
| `prediction/obj_num` | 10 | 预测器管理的动态物体数量。 |
| `prediction/lambda` | 1.0 | 预测模型参数，具体含义由 `plan_env` 的预测实现决定。 |
| `prediction/predict_rate` | 1.0 Hz | 预测更新频率。 |

重要：当前版本 rebound 优化目标中的 moving-object 代价被注释，因此设置这些参数不等于动态障碍避碰已经实际生效。

## 仿真与传感器常用参数

| 参数 | 含义 |
|---|---|
| `use_dynamic` | true 使用 SO3 控制器和四旋翼动力学；false 使用 `poscmd_2_odom` 简化模型。 |
| `use_mockamap` | true 使用 mockamap；false 使用随机森林地图。 |
| `init_x/y/z` | 无人机初始位置，m。不同 launch 中有些名字带末尾下划线。 |
| `target_x/y/z` | 预设目标位置，m。 |
| `obj_num` | 计划生成/预测的移动物体数；当前通用 launch 中生成节点未加入执行。 |
| `sensing_horizon` | 模拟相机最远感知距离，m。 |
| `sensing_rate` | 深度/点云生成频率，Hz。 |
| `estimation_rate` | 仿真位姿估计更新频率，Hz。 |
| `rate/odom` | 动力学模型里程计发布频率，默认 100 Hz。 |
| `mass` | SO3 控制器使用的飞行器质量，默认 0.98 kg。 |

## 一个较稳妥的调参顺序

1. 先确认 `frame_id`、里程计、深度话题和相机内参正确；这些错了，后续调优化权重没有意义。
2. 按真实飞行器能力设置 `max_vel`、`max_acc`，并让 manager、optimization、bspline 三处一致。
3. 按机体半径和定位误差设置地图膨胀、`dist0`、`swarm_clearance`。
4. 调 `planning_horizon` 和 `control_points_distance`，在成功率与计算时间之间平衡。
5. 最后再小幅调整各 `lambda`，并保存每次实验的规划耗时、最小净空和动力学峰值。

不要一开始同时修改很多参数，否则即使结果变好，也很难判断究竟是哪一项产生了作用。
