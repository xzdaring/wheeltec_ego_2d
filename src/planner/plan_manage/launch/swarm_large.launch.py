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

"""21 架无人机的大规模集群仿真入口。

结构与 swarm.launch.py 相同，但地图更大、无人机更多，适合测试多机通信、
顺序启动和轨迹避碰；运行时需要更多 CPU 与内存资源。
"""

def generate_launch_description():
    # 集群共享地图尺寸，单位 m，以及里程计话题的公共后缀。
    map_size_x = LaunchConfiguration('map_size_x', default = 72.0)
    map_size_y = LaunchConfiguration('map_size_y', default = 30.0)
    map_size_z = LaunchConfiguration('map_size_z', default = 5.0)
    odom_topic = LaunchConfiguration('odom_topic', default = 'visual_slam/odom')
    
    # 声明可由父 launch 或命令行覆盖的公共参数。
    map_size_x_cmd = DeclareLaunchArgument('map_size_x', default_value=map_size_x, description='Map size along x')
    map_size_y_cmd = DeclareLaunchArgument('map_size_y', default_value=map_size_y, description='Map size along y')
    map_size_z_cmd = DeclareLaunchArgument('map_size_z', default_value=map_size_z, description='Map size along z')
    odom_topic_cmd = DeclareLaunchArgument('odom_topic', default_value=odom_topic, description='Odometry topic')

    # 地图类型开关；整个集群共享一个全局地图生成器。
    use_mockamap = LaunchConfiguration('use_mockamap', default=False) # map_generator or mockamap 
    
    use_mockamap_cmd = DeclareLaunchArgument('use_mockamap', default_value=use_mockamap, description='Choose map type, map_generator or mockamap')
    
    # 大规模多机默认关闭完整动力学，以减小计算负担。
    use_dynamic = LaunchConfiguration('use_dynamic', default=False)  
    use_dynamic_cmd = DeclareLaunchArgument('use_dynamic', default_value=use_dynamic, description='Use Drone Simulation Considering Dynamics or Not')

    # 随机森林地图；use_mockamap=false 时启动。
    map_generator_node = Node(
        package='map_generator',
        executable='random_forest',
        name='random_forest',
        output='screen',
        parameters=[
            {'map/x_size': 66.0},
            {'map/y_size': 30.0},
            {'map/z_size': 3.0},
            {'map/resolution': 0.1}, # 地图分辨率，m
            {'ObstacleShape/seed': 1.0},
            {'map/obs_num': 200},
            {'ObstacleShape/lower_rad': 0.5},
            {'ObstacleShape/upper_rad': 0.7},
            {'ObstacleShape/lower_hei': 0.0},
            {'ObstacleShape/upper_hei': 3.0},
            {'map/circle_num': 200},
            {'ObstacleShape/radius_l': 0.7},
            {'ObstacleShape/radius_h': 0.5},
            {'ObstacleShape/z_l': 0.7},
            {'ObstacleShape/z_h': 0.8},
            {'ObstacleShape/theta': 0.5},
            {'sensing/radius': 5.0},
            {'sensing/rate': 1.0},
            {'min_distance': 1.2}
        ],
        condition = UnlessCondition(use_mockamap)
    )

    # mockamap 地图；use_mockamap=true 时启动。
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
            {'resolution': 0.1},
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
    
    # 21 架无人机的编号、起点和终点，坐标单位均为 m。
    # drone_id 必须唯一且从 0 连续编号；C++ 多机逻辑依赖这个约定。
    drone_configs = [
        {'drone_id': 0, 'init_x': -35.0, 'init_y': -10.0, 'init_z': 0.1, 'target_x': 35.0, 'target_y': -10.0, 'target_z': 1.0},
        {'drone_id': 1, 'init_x': -35.0, 'init_y': -9.0, 'init_z': 0.1, 'target_x': 35.0, 'target_y': -9.0, 'target_z': 1.0},
        {'drone_id': 2, 'init_x': -35.0, 'init_y': -8.0, 'init_z': 0.1, 'target_x': 35.0, 'target_y': -8.0, 'target_z': 1.0},
        {'drone_id': 3, 'init_x': -35.0, 'init_y': -7.0, 'init_z': 0.1, 'target_x': 35.0, 'target_y': -7.0, 'target_z': 1.0},
        {'drone_id': 4, 'init_x': -35.0, 'init_y': -6.0, 'init_z': 0.1, 'target_x': 35.0, 'target_y': -6.0, 'target_z': 1.0},
        {'drone_id': 5, 'init_x': -35.0, 'init_y': -5.0, 'init_z': 0.1, 'target_x': 35.0, 'target_y': -5.0, 'target_z': 1.0},
        {'drone_id': 6, 'init_x': -35.0, 'init_y': -4.0, 'init_z': 0.1, 'target_x': 35.0, 'target_y': -4.0, 'target_z': 1.0},
        {'drone_id': 7, 'init_x': -35.0, 'init_y': -3.0, 'init_z': 0.1, 'target_x': 35.0, 'target_y': -3.0, 'target_z': 1.0},
        {'drone_id': 8, 'init_x': -35.0, 'init_y': -2.0, 'init_z': 0.1, 'target_x': 35.0, 'target_y': -2.0, 'target_z': 1.0},
        {'drone_id': 9, 'init_x': -35.0, 'init_y': -1.0, 'init_z': 0.1, 'target_x': 35.0, 'target_y': -1.0, 'target_z': 1.0},
        {'drone_id': 10, 'init_x': -35.0, 'init_y': 0.0, 'init_z': 0.1, 'target_x': 35.0, 'target_y': 0.0, 'target_z': 1.0},
        {'drone_id': 11, 'init_x': -35.0, 'init_y': 1.0, 'init_z': 0.1, 'target_x': 35.0, 'target_y': 1.0, 'target_z': 1.0},
        {'drone_id': 12, 'init_x': -35.0, 'init_y': 2.0, 'init_z': 0.1, 'target_x': 35.0, 'target_y': 2.0, 'target_z': 1.0},
        {'drone_id': 13, 'init_x': -35.0, 'init_y': 3.0, 'init_z': 0.1, 'target_x': 35.0, 'target_y': 3.0, 'target_z': 1.0},
        {'drone_id': 14, 'init_x': -35.0, 'init_y': 4.0, 'init_z': 0.1, 'target_x': 35.0, 'target_y': 4.0, 'target_z': 1.0},
        {'drone_id': 15, 'init_x': -35.0, 'init_y': 5.0, 'init_z': 0.1, 'target_x': 35.0, 'target_y': 5.0, 'target_z': 1.0},
        {'drone_id': 16, 'init_x': -35.0, 'init_y': 6.0, 'init_z': 0.1, 'target_x': 35.0, 'target_y': 6.0, 'target_z': 1.0},
        {'drone_id': 17, 'init_x': -35.0, 'init_y': 7.0, 'init_z': 0.1, 'target_x': 35.0, 'target_y': 7.0, 'target_z': 1.0},
        {'drone_id': 18, 'init_x': -35.0, 'init_y': 8.0, 'init_z': 0.1, 'target_x': 35.0, 'target_y': 8.0, 'target_z': 1.0},
        {'drone_id': 19, 'init_x': -35.0, 'init_y': 9.0, 'init_z': 0.1, 'target_x': 35.0, 'target_y': 9.0, 'target_z': 1.0},
        {'drone_id': 20, 'init_x': -35.0, 'init_y': 10.0, 'init_z': 0.1, 'target_x': 35.0, 'target_y': 10.0, 'target_z': 1.0}
    ]

    # 为每架无人机展开一份 run_in_sim.launch.py。
    drone_nodes = []

    for config in drone_configs:        
        drone_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('ego_planner'), 'launch', 'run_in_sim.launch.py')),
            launch_arguments={
                # 注意：子 launch 实际声明 use_dynamic；此处名称不一致可能导致
                # 外层开关不能覆盖默认值，排查动力学模式时要特别留意。
                'use_dynamic_cmd': use_dynamic,
                'drone_id': str(config['drone_id']),
                'init_x': str(config['init_x']),
                'init_y': str(config['init_y']),
                'init_z': str(config['init_z']),
                'target_x': str(config['target_x']),
                'target_y': str(config['target_y']),
                'target_z': str(config['target_z']),
                'map_size_x': map_size_x,
                'map_size_y': map_size_y,
                'map_size_z': map_size_z,
                'odom_topic': odom_topic
            }.items()
        )
        drone_nodes.append(drone_launch)
        
    ld = LaunchDescription()
        
    ld.add_action(map_size_x_cmd)
    ld.add_action(map_size_y_cmd)
    ld.add_action(map_size_z_cmd)
    ld.add_action(odom_topic_cmd)
    ld.add_action(use_mockamap_cmd)
    ld.add_action(use_dynamic_cmd)

    # 两种地图节点依据 use_mockamap 条件二选一。
    ld.add_action(map_generator_node)
    ld.add_action(mockamap_node)

    # 加入 21 套无人机子系统。
    for drone in drone_nodes:        
        ld.add_action(drone)

    return ld
