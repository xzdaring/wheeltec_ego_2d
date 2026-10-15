import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

"""EGO-Planner 核心节点的统一参数与话题配置。

这个 launch 不负责启动仿真器，只启动 ego_planner_node，并把外层 launch
传入的地图、传感器、动力学和航点配置转换成节点参数。

参数单位约定：
  - 位置、距离、地图尺寸：m
  - 速度：m/s
  - 加速度：m/s^2
  - jerk：m/s^3
  - 时间：s
  - 相机内参：pixel
  - 概率：0~1
"""

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    planning_frame = LaunchConfiguration('planning_frame', default='odom')
    # LaunchConfiguration 是“延迟求值”的启动参数占位符；真正值可能由
    # single_run_in_sim.py、run_in_sim.py 或命令行覆盖。
    # 名字末尾的下划线是原项目接口的一部分，不要随意去掉。
    map_size_x = LaunchConfiguration('map_size_x_', default=42.0)
    map_size_y = LaunchConfiguration('map_size_y_', default=30.0)
    map_size_z = LaunchConfiguration('map_size_z_', default=5.0)
    
    # 下面是未添加 drone_<id>_ 前缀前的传感器话题后缀。
    odometry_topic = LaunchConfiguration('odometry_topic', default='odom')
    camera_pose_topic = LaunchConfiguration('camera_pose_topic', default='camera_pose')
    depth_topic = LaunchConfiguration('depth_topic', default='depth_image')
    cloud_topic = LaunchConfiguration('cloud_topic', default='/ego/cloud')
    
    # 针孔相机内参：cx/cy 为主点坐标，fx/fy 为像素焦距。
    cx = LaunchConfiguration('cx', default=321.04638671875)
    cy = LaunchConfiguration('cy', default=243.44969177246094)
    fx = LaunchConfiguration('fx', default=387.229248046875)
    fy = LaunchConfiguration('fy', default=387.229248046875)
    
    # 局部轨迹动力学限制及滚动规划空间视野。
    max_vel = LaunchConfiguration('max_vel', default=2.0)
    max_acc = LaunchConfiguration('max_acc', default=3.0)
    planning_horizon = LaunchConfiguration('planning_horizon', default=7.5)
    
    # 预设航点最多在本文件显式准备 5 个。只有前 point_num 个会被 FSM 读取。
    point_num = LaunchConfiguration('point_num', default=1)
    point0_x = LaunchConfiguration('point0_x', default=0.0)
    point0_y = LaunchConfiguration('point0_y', default=0.0)
    point0_z = LaunchConfiguration('point0_z', default=0.0)
    point1_x = LaunchConfiguration('point1_x', default=10.0)
    point1_y = LaunchConfiguration('point1_y', default=10.0)
    point1_z = LaunchConfiguration('point1_z', default=0.0)
    point2_x = LaunchConfiguration('point2_x', default=20.0)
    point2_y = LaunchConfiguration('point2_y', default=20.0)
    point2_z = LaunchConfiguration('point2_z', default=1.0)
    point3_x = LaunchConfiguration('point3_x', default=-10.0)
    point3_y = LaunchConfiguration('point3_y', default=-10.0)
    point3_z = LaunchConfiguration('point3_z', default=1.0)
    point4_x = LaunchConfiguration('point4_x', default=30.0)
    point4_y = LaunchConfiguration('point4_y', default=30.0)
    point4_z = LaunchConfiguration('point4_z', default=1.0)

    # flight_type：1=RViz 手动目标，2=预设航点；3 在枚举中存在但未完整实现。
    flight_type = LaunchConfiguration('flight_type', default=2)
    # true 时为碰撞区段生成不同绕障侧的候选并选择代价最低者，计算更重。
    use_distinctive_trajs = LaunchConfiguration('use_distinctive_trajs', default=True)
    
    # 动态物体预测器期望管理的对象数量；不等同于静态地图障碍数量。
    obj_num_set = LaunchConfiguration('obj_num_set', default=10)
    
    # 多机编号从 0 开始，决定节点名、话题前缀和顺序启动关系。
    drone_id = LaunchConfiguration('drone_id', default=0)

    # 将上述占位符正式暴露为 ros2 launch 可覆盖的命令行参数。
    map_size_x_arg = DeclareLaunchArgument('map_size_x_', default_value=map_size_x, description='Map size along X')
    map_size_y_arg = DeclareLaunchArgument('map_size_y_', default_value=map_size_y, description='Map size along Y')
    map_size_z_arg = DeclareLaunchArgument('map_size_z_', default_value=map_size_z, description='Map size along Z')
    odometry_topic_arg = DeclareLaunchArgument('odometry_topic', default_value=odometry_topic, description='Odometry topic')
    camera_pose_topic_arg = DeclareLaunchArgument('camera_pose_topic', default_value=camera_pose_topic, description='Camera pose topic')
    depth_topic_arg = DeclareLaunchArgument('depth_topic', default_value=depth_topic, description='Depth topic')
    cloud_topic_arg = DeclareLaunchArgument('cloud_topic', default_value=cloud_topic, description='Point cloud topic')
    cx_arg = DeclareLaunchArgument('cx', default_value=cx, description='Camera intrinsic cx')
    cy_arg = DeclareLaunchArgument('cy', default_value=cy, description='Camera intrinsic cy')
    fx_arg = DeclareLaunchArgument('fx', default_value=fx, description='Camera intrinsic fx')
    fy_arg = DeclareLaunchArgument('fy', default_value=fy, description='Camera intrinsic fy')
    max_vel_arg = DeclareLaunchArgument('max_vel', default_value=max_vel, description='Maximum velocity')
    max_acc_arg = DeclareLaunchArgument('max_acc', default_value=max_acc, description='Maximum acceleration')
    planning_horizon_arg = DeclareLaunchArgument('planning_horizon', default_value=planning_horizon, description='Planning horizon')
    
    point_num_arg = DeclareLaunchArgument('point_num', default_value=point_num, description='Number of waypoints')
    point0_x_arg = DeclareLaunchArgument('point0_x', default_value=point0_x, description='Waypoint 0 X coordinate')
    point0_y_arg = DeclareLaunchArgument('point0_y', default_value=point0_y, description='Waypoint 0 Y coordinate')
    point0_z_arg = DeclareLaunchArgument('point0_z', default_value=point0_z, description='Waypoint 0 Z coordinate')
    point1_x_arg = DeclareLaunchArgument('point1_x', default_value=point1_x, description='Waypoint 1 X coordinate')
    point1_y_arg = DeclareLaunchArgument('point1_y', default_value=point1_y, description='Waypoint 1 Y coordinate')
    point1_z_arg = DeclareLaunchArgument('point1_z', default_value=point1_z, description='Waypoint 1 Z coordinate')
    point2_x_arg = DeclareLaunchArgument('point2_x', default_value=point2_x, description='Waypoint 2 X coordinate')
    point2_y_arg = DeclareLaunchArgument('point2_y', default_value=point2_y, description='Waypoint 2 Y coordinate')
    point2_z_arg = DeclareLaunchArgument('point2_z', default_value=point2_z, description='Waypoint 2 Z coordinate')
    point3_x_arg = DeclareLaunchArgument('point3_x', default_value=point3_x, description='Waypoint 3 X coordinate')
    point3_y_arg = DeclareLaunchArgument('point3_y', default_value=point3_y, description='Waypoint 3 Y coordinate')
    point3_z_arg = DeclareLaunchArgument('point3_z', default_value=point3_z, description='Waypoint 3 Z coordinate')
    point4_x_arg = DeclareLaunchArgument('point4_x', default_value=point4_x, description='Waypoint 4 X coordinate')
    point4_y_arg = DeclareLaunchArgument('point4_y', default_value=point4_y, description='Waypoint 4 Y coordinate')
    point4_z_arg = DeclareLaunchArgument('point4_z', default_value=point4_z, description='Waypoint 4 Z coordinate')
    
    flight_type_arg = DeclareLaunchArgument('flight_type', default_value=flight_type, description='flight_type')
    use_distinctive_trajs_arg = DeclareLaunchArgument('use_distinctive_trajs', default_value=use_distinctive_trajs, description='Use distinctive trajectories')
    obj_num_set_arg = DeclareLaunchArgument('obj_num_set', default_value=obj_num_set, description='Number of objects')
    drone_id_arg = DeclareLaunchArgument('drone_id', default_value=drone_id, description='Drone ID')

    # 核心规划节点。它内部包含 FSM、PlannerManager、GridMap 和优化器。
    ego_planner_node = Node(
        package='ego_planner',
        executable='ego_planner_node',
        name=['drone_', drone_id, '_ego_planner_node'],
        output='screen',
        remappings=[
            # 本机状态与规划结果：每架无人机使用独立前缀，避免同名冲突。
            ('odom_world', odometry_topic),
            ('planning/bspline', ['drone_', drone_id, '_planning/bspline']),
            ('planning/data_display', ['drone_', drone_id, '_planning/data_display']),
            # 所有无人机共享广播话题；FSM 收到后用 drone_id 忽略自身消息。
            ('planning/broadcast_bspline_from_planner', '/broadcast_bspline'),
            ('planning/broadcast_bspline_to_planner', '/broadcast_bspline'),
            
            # 仅供 RViz 显示的 Marker 话题。
            ('goal_point', ['drone_', drone_id, '_plan_vis/goal_point']),
            ('global_list', ['drone_', drone_id, '_plan_vis/global_list']),
            ('init_list', ['drone_', drone_id, '_plan_vis/init_list']),
            ('optimal_list', ['drone_', drone_id, '_plan_vis/optimal_list']),
            ('a_star_list', ['drone_', drone_id, '_plan_vis/a_star_list']),
            
            # GridMap 的传感器输入和膨胀地图输出。
            ('grid_map/odom', odometry_topic),
            ('grid_map/cloud', cloud_topic), # 在线地图适配器已经把障碍转换到 odom。
            ('grid_map/pose', ['drone_', drone_id, '_', camera_pose_topic]),
            ('grid_map/depth', ['drone_', drone_id, '_', depth_topic]),
            ('grid_map/occupancy_inflate', ['drone_', drone_id, '_grid/grid_map/occupancy_inflate'])
        ],
        parameters=[
            # ---------------- FSM：何时规划、重规划或急停 ----------------
            {'fsm/flight_type': flight_type},
            {'fsm/thresh_replan_time': 1.0},       # 局部轨迹执行 1 s 后允许滚动重规划。
            {'fsm/thresh_no_replan_meter': 1.0},   # 距终点 1 m 内避免无意义频繁重规划。
            {'fsm/planning_horizon': planning_horizon}, # 局部空间视野，m。
            {'fsm/planning_horizen_time': 3.0},    # 沿全局轨迹向前看的时间视野，s。
            {'fsm/emergency_time': 1.0},           # 预计 1 s 内碰撞则进入急停。
            {'fsm/realworld_experiment': False},   # False 时无需外部遥控触发。
            {'fsm/fail_safe': True},               # 急停稳定后允许自动恢复规划。
            {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)},
            
            # PRESET_TARGET 模式使用的世界坐标航点，单位 m。
            {'fsm/waypoint_num': point_num},
            {'fsm/waypoint0_x': point0_x},
            {'fsm/waypoint0_y': point0_y},
            {'fsm/waypoint0_z': point0_z},
            {'fsm/waypoint1_x': point1_x},
            {'fsm/waypoint1_y': point1_y},
            {'fsm/waypoint1_z': point1_z},
            {'fsm/waypoint2_x': point2_x},
            {'fsm/waypoint2_y': point2_y},
            {'fsm/waypoint2_z': point2_z},
            {'fsm/waypoint3_x': point3_x},
            {'fsm/waypoint3_y': point3_y},
            {'fsm/waypoint3_z': point3_z},
            {'fsm/waypoint4_x': point4_x},
            {'fsm/waypoint4_y': point4_y},
            {'fsm/waypoint4_z': point4_z},
            
            # ---------------- GridMap：局部三维占据地图 ----------------
            {'grid_map/resolution': 0.1},          # 体素边长；越小越精细、耗时和内存越大。
            {'grid_map/map_size_x': map_size_x},
            {'grid_map/map_size_y': map_size_y},
            {'grid_map/map_size_z': 2.0},
            {'grid_map/local_update_range_x': 5.5}, # 每次围绕相机更新的局部半范围，m。
            {'grid_map/local_update_range_y': 5.5},
            {'grid_map/local_update_range_z': 4.5},
            {'grid_map/obstacles_inflation': 0.20}, # 障碍膨胀半径，约 9.9 cm。
            {'grid_map/local_map_margin': 10},       # 局部更新边界额外保留的体素层数。
            {'grid_map/ground_height': -0.5},       # 世界坐标地面高度，m。
            # 相机参数：必须与深度图实际标定一致。
            {'grid_map/cx': cx},
            {'grid_map/cy': cy},
            {'grid_map/fx': fx},
            {'grid_map/fy': fy},
            # 深度滤波：去掉边缘、过近、过远和跳变过大的像素。
            {'grid_map/use_depth_filter': True},
            {'grid_map/depth_filter_tolerance': 0.15}, # 前后帧深度容差，m。
            {'grid_map/depth_filter_maxdist': 5.0},    # 最大有效深度，m。
            {'grid_map/depth_filter_mindist': 0.2},    # 最小有效深度，m。
            {'grid_map/depth_filter_margin': 2},       # 忽略图像边缘 2 像素。
            {'grid_map/k_depth_scaling_factor': 1000.0}, # uint16 深度通常以 mm 存储，除 1000 转 m。
            {'grid_map/skip_pixel': 2},                # 每隔 2 像素投影一次，换速度换精度。
            # 概率占据融合：命中提高占据概率，射线穿过降低占据概率。
            {'grid_map/p_hit': 0.65},                  # 端点命中一次的占据概率模型。
            {'grid_map/p_miss': 0.35},                 # 射线穿过一次的空闲概率模型。
            {'grid_map/p_min': 0.12},                  # 概率下限，防止历史被无限强化为空闲。
            {'grid_map/p_max': 0.90},                  # 概率上限，防止历史被无限强化为占据。
            {'grid_map/p_occ': 0.80},                  # 超过该概率判为占据。
            {'grid_map/min_ray_length': 0.1},          # 太近的深度射线不融合，m。
            {'grid_map/max_ray_length': 4.5},          # 射线融合最大长度，m。
            
            {'grid_map/virtual_ceil_height': -1.0},      # 虚拟天花板高度，限制飞行上界，m。
            {'grid_map/visualization_truncate_height': 1.8}, # RViz 显示截断高度，不等于规划上界。
            {'grid_map/show_occ_time': False},
            {'grid_map/pose_type': 1},                 # 1=PoseStamped；2=Odometry，见 GridMap 枚举。
            {'grid_map/frame_id': planning_frame},
            {'planning_frame': planning_frame},
            # ---------------- PlannerManager：轨迹尺度与物理限制 ----------------
            {'manager/max_vel': max_vel},              # 各轴最大速度，m/s。
            {'manager/max_acc': max_acc},              # 各轴最大加速度，m/s²。
            {'manager/max_jerk': 4.0},                 # jerk 配置，m/s³。
            {'manager/control_points_distance': 0.4},  # 相邻控制点期望间距，m。
            {'manager/feasibility_tolerance': 0.05},   # 可行性检查允许约 5% 余量。
            {'manager/planning_horizon': planning_horizon},
            {'manager/use_distinctive_trajs': use_distinctive_trajs},
            {'manager/drone_id': drone_id},
            # ---------------- 控制点优化代价权重 ----------------
            # lambda 越大，对应目标在总目标函数中越重要，但各项量纲/数值尺度不同，
            # 不能仅凭数值大小直接比较“重要程度”。
            {'optimization/lambda_smooth': 1.0},       # jerk/高阶差分平滑代价。
            {'optimization/lambda_collision': 0.5},    # 静态障碍、多机和终端相关代价权重。
            {'optimization/lambda_feasibility': 0.1},  # 速度/加速度超限惩罚。
            {'optimization/lambda_fitness': 1.0},      # 精修时贴近参考曲线的权重。
            {'optimization/dist0': 0.5},               # 静态障碍期望净空，m。
            {'optimization/swarm_clearance': 0.5},     # 多机期望最小间距，m。
            {'optimization/max_vel': max_vel},
            {'optimization/max_acc': max_acc},

            # B-Spline 自身的物理限制检查参数。
            {'bspline/limit_vel': max_vel},
            {'bspline/limit_acc': max_acc},
            {'bspline/limit_ratio': 1.1},              # 超限时建议时间伸缩的安全系数。

            # 动态物体预测器参数。当前 rebound 主目标中 moving-object 代价被注释，
            # 因而这些参数存在并不代表动态障碍代价已真正启用。
            {'prediction/obj_num': obj_num_set},
            {'prediction/lambda': 1.0},
            {'prediction/predict_rate': 1.0}
        ]
    )

    # Create LaunchDescription
    ld = LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('planning_frame', default_value='odom')])

    # Add LaunchArguments
    ld.add_action(map_size_x_arg)
    ld.add_action(map_size_y_arg)
    ld.add_action(map_size_z_arg)
    ld.add_action(odometry_topic_arg)
    ld.add_action(camera_pose_topic_arg)
    ld.add_action(depth_topic_arg)
    ld.add_action(cloud_topic_arg)
    ld.add_action(cx_arg)
    ld.add_action(cy_arg)
    ld.add_action(fx_arg)
    ld.add_action(fy_arg)
    ld.add_action(max_vel_arg)
    ld.add_action(max_acc_arg)
    ld.add_action(planning_horizon_arg)
    
    ld.add_action(point_num_arg)
    ld.add_action(point0_x_arg)
    ld.add_action(point0_y_arg)
    ld.add_action(point0_z_arg)
    ld.add_action(point1_x_arg)
    ld.add_action(point1_y_arg)
    ld.add_action(point1_z_arg)
    ld.add_action(point2_x_arg)
    ld.add_action(point2_y_arg)
    ld.add_action(point2_z_arg)
    ld.add_action(point3_x_arg)
    ld.add_action(point3_y_arg)
    ld.add_action(point3_z_arg)
    ld.add_action(point4_x_arg)
    ld.add_action(point4_y_arg)
    ld.add_action(point4_z_arg)
    
    ld.add_action(flight_type_arg)
    ld.add_action(use_distinctive_trajs_arg)
    ld.add_action(obj_num_set_arg)
    ld.add_action(drone_id_arg)


    # Add Node
    ld.add_action(ego_planner_node)

    return ld
