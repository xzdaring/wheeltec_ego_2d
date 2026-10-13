import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PythonExpression
from launch.conditions import IfCondition, UnlessCondition

"""单机仿真的总启动文件。

它依次组织地图生成器、EGO-Planner、轨迹服务器和无人机仿真器。
本文件中位置/长度通常以 m 为单位，速度以 m/s 为单位，加速度以 m/s^2 为单位。
"""

def generate_launch_description():
    # LaunchConfiguration 是启动时可以被命令行覆盖的变量，例如：
    # ros2 launch ego_planner single_run_in_sim.launch.py drone_id:=1
    obj_num = LaunchConfiguration('obj_num', default=10)
    # 无人机编号；多机运行时每架无人机的编号必须唯一。
    drone_id = LaunchConfiguration('drone_id', default=0)
    
    # 地图三个方向的尺寸，单位 m。
    map_size_x = LaunchConfiguration('map_size_x', default = 50.0)
    map_size_y = LaunchConfiguration('map_size_y', default = 25.0)
    map_size_z = LaunchConfiguration('map_size_z', default = 2.0)
    # 规划器订阅的里程计话题，包含无人机的位置、速度和姿态。
    odom_topic = LaunchConfiguration('odom_topic', default = 'visual_slam/odom')
    
    
    # DeclareLaunchArgument 才会把上面的变量正式暴露为 launch 参数。
    obj_num_cmd = DeclareLaunchArgument('obj_num', default_value=obj_num, description='Number of objects')
    drone_id_cmd = DeclareLaunchArgument('drone_id', default_value=drone_id, description='Drone ID')
    
    map_size_x_cmd = DeclareLaunchArgument('map_size_x', default_value=map_size_x, description='Map size along x')
    map_size_y_cmd = DeclareLaunchArgument('map_size_y', default_value=map_size_y, description='Map size along y')
    map_size_z_cmd = DeclareLaunchArgument('map_size_z', default_value=map_size_z, description='Map size along z')
    odom_topic_cmd = DeclareLaunchArgument('odom_topic', default_value=odom_topic, description='Odometry topic')

    # 地图属性以及是否使用动力学仿真。
    # use_mockamap=false：启动随机森林；true：启动 mockamap。
    use_mockamap = LaunchConfiguration('use_mockamap', default=False) # map_generator or mockamap 
    
    use_mockamap_cmd = DeclareLaunchArgument('use_mockamap', default_value=use_mockamap, description='Choose map type, map_generator or mockamap')
    
    # false：位置指令直接积分为里程计；true：经过控制器和四旋翼动力学模型。
    use_dynamic = LaunchConfiguration('use_dynamic', default=False)  
    use_dynamic_cmd = DeclareLaunchArgument('use_dynamic', default_value=use_dynamic, description='Use Drone Simulation Considering Dynamics or Not')
    
    # 随机森林地图节点，仅在 use_mockamap=false 时启动。
    # 半径、高度、障碍物数量等参数共同决定地图密度和难度。
    map_generator_node = Node(
        package='map_generator',
        executable='random_forest',
        name='random_forest',
        output='screen',
        parameters=[
            {'map/x_size': 26.0},
            {'map/y_size': 20.0},
            {'map/z_size': 3.0},
            {'map/resolution': 0.1},       # 点云/地图分辨率，m
            {'ObstacleShape/seed': 1.0},
            {'map/obs_num': 250},          # 柱状障碍物数量
            {'ObstacleShape/lower_rad': 0.5},
            {'ObstacleShape/upper_rad': 0.7},
            {'ObstacleShape/lower_hei': 0.0},
            {'ObstacleShape/upper_hei': 3.0},
            {'map/circle_num': 250},       # 圆环/椭圆类障碍物数量
            {'ObstacleShape/radius_l': 0.7},
            {'ObstacleShape/radius_h': 0.5},
            {'ObstacleShape/z_l': 0.7},
            {'ObstacleShape/z_h': 0.8},
            {'ObstacleShape/theta': 0.5},
            {'pub_rate': 1.0},             # 地图发布频率，Hz
            {'min_distance': 0.8}          # 障碍物之间要求的最小距离，m
        ],
        condition = UnlessCondition(use_mockamap)
    )

    # mockamap 地图节点，仅在 use_mockamap=true 时启动。
    mockamap_node = Node(
        package='mockamap',
        executable='mockamap_node',
        name='mockamap_node',
        output='screen',
        remappings=[
            ('/mock_map', '/map_generator/global_cloud')
        ],
        parameters=[
            {'seed': 127},
            {'update_freq': 0.5},
            {'resolution': 0.1},           # 地图分辨率，m
            {'x_length': PythonExpression(['int(', map_size_x, ')'])},
            {'y_length': PythonExpression(['int(', map_size_y, ')'])},
            {'z_length': PythonExpression(['int(', map_size_z, ')'])},
            {'type': 1},
            {'complexity': 0.05},
            {'fill': 0.12},
            {'fractal': 1},
            {'attenuation': 0.1}
        ],
        condition = IfCondition(use_mockamap)
    )
    
    # 加载规划器主体 advanced_param.launch.py。
    # 这里的参数会继续传给 ego_planner_node、GridMap、优化器等模块。
    advanced_param_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ego_planner'), 'launch', 'advanced_param.launch.py')),
        launch_arguments={
            'drone_id': drone_id,
            'map_size_x_': map_size_x,
            'map_size_y_': map_size_y,
            'map_size_z_': map_size_z,
            'odometry_topic': odom_topic,
            'obj_num_set': obj_num,
            
            'camera_pose_topic': 'pcl_render_node/camera_pose',
            'depth_topic': 'pcl_render_node/depth',
            'cloud_topic': 'pcl_render_node/cloud',
            
            # 深度相机内参，单位 pixel；必须与实际/仿真相机一致。
            'cx': str(321.04638671875),
            'cy': str(243.44969177246094),
            'fx': str(387.229248046875),
            'fy': str(387.229248046875),
            'max_vel': str(2.0),            # 最大速度，m/s
            'max_acc': str(6.0),            # 最大加速度，m/s^2
            'planning_horizon': str(7.5),   # 局部规划空间范围，m
            'use_distinctive_trajs': 'True',# 是否尝试拓扑不同的候选轨迹
            'flight_type': str(2),          # 2：按下面的预设航点飞行
            'point_num': str(4),            # 实际使用 point0~point3
            # 预设航点坐标，单位 m。point4 被提供但 point_num=4，通常不会使用。
            'point0_x': str(15.0),
            'point0_y': str(0.0),
            'point0_z': str(1.0),
            
            'point1_x': str(-15.0),
            'point1_y': str(0.0),
            'point1_z': str(1.0),
            
            'point2_x': str(15.0),
            'point2_y': str(0.0),
            'point2_z': str(1.0),
            
            'point3_x': str(-15.0),
            'point3_y': str(0.0),
            'point3_z': str(1.0),
            
            'point4_x': str(15.0),
            'point4_y': str(0.0),
            'point4_z': str(1.0),
        }.items()
    )
    
    # 轨迹服务器：接收 B 样条，按当前时间求出期望位置/速度/加速度，
    # 再向控制器或仿真器发布 position_cmd。
    traj_server_node = Node(
        package='ego_planner',
        executable='traj_server',
        name=['drone_', drone_id, '_traj_server'],
        output='screen',
        remappings=[
            ('position_cmd', ['drone_', drone_id, '_planning/pos_cmd']),
            ('planning/bspline', ['drone_', drone_id, '_planning/bspline'])
        ],
        parameters=[
            # 计算偏航朝向时观察轨迹前方多远，单位 s；不是碰撞检查参数。
            {'traj_server/time_forward': 1.0}
        ]
    )
    
    # 加载仿真器；初始位置为 (-15, 0, 0.1) m。
    simulator_include = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('ego_planner'), 'launch', 'simulator.launch.py')),
        launch_arguments={
            'use_dynamic': use_dynamic,
            'drone_id': drone_id,
            'map_size_x_': map_size_x,
            'map_size_y_': map_size_y,
            'map_size_z_': map_size_z,
            'init_x_': str(-15.0),
            'init_y_': str(0.0),
            'init_z_': str(0.1),
            'odometry_topic': odom_topic
        }.items()
    )
    
    ld = LaunchDescription()
        
    ld.add_action(map_size_x_cmd)
    ld.add_action(map_size_y_cmd)
    ld.add_action(map_size_z_cmd)
    ld.add_action(odom_topic_cmd)
    ld.add_action(obj_num_cmd)
    ld.add_action(drone_id_cmd)
    ld.add_action(use_dynamic_cmd)
    ld.add_action(use_mockamap_cmd)

    # 添加顺序便于阅读；launch 系统会统一管理各节点进程。
    ld.add_action(map_generator_node)
    ld.add_action(mockamap_node)
    ld.add_action(advanced_param_include)
    ld.add_action(traj_server_node)
    ld.add_action(simulator_include)

    return ld
