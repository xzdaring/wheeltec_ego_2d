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

"""10 架无人机的集群仿真入口。

全体无人机共享一张全局地图；每个配置项都会展开成一份 run_in_sim.launch.py，
从而拥有独立的规划器、轨迹服务器、仿真器和带 drone_id 前缀的话题。
"""

def generate_launch_description():
    # 集群共享地图尺寸，单位 m，以及各无人机里程计话题的公共后缀。
    map_size_x = LaunchConfiguration('map_size_x', default = 42.0)
    map_size_y = LaunchConfiguration('map_size_y', default = 30.0)
    map_size_z = LaunchConfiguration('map_size_z', default = 5.0)
    odom_topic = LaunchConfiguration('odom_topic', default = 'visual_slam/odom')
    
    # 将集群公共配置声明成可从命令行覆盖的 launch 参数。
    map_size_x_cmd = DeclareLaunchArgument('map_size_x', default_value=map_size_x, description='Map size along x')
    map_size_y_cmd = DeclareLaunchArgument('map_size_y', default_value=map_size_y, description='Map size along y')
    map_size_z_cmd = DeclareLaunchArgument('map_size_z', default_value=map_size_z, description='Map size along z')
    odom_topic_cmd = DeclareLaunchArgument('odom_topic', default_value=odom_topic, description='Odometry topic')

    # 地图类型开关；一张地图由整个集群共享，只启动一个地图节点。
    use_mockamap = LaunchConfiguration('use_mockamap', default=False) # map_generator or mockamap 
    
    use_mockamap_cmd = DeclareLaunchArgument('use_mockamap', default_value=use_mockamap, description='Choose map type, map_generator or mockamap')
    
    # 动力学仿真开关。多机完整动力学计算量较大，默认使用简化模型。
    use_dynamic = LaunchConfiguration('use_dynamic', default=False)  
    use_dynamic_cmd = DeclareLaunchArgument('use_dynamic', default_value=use_dynamic, description='Use Drone Simulation Considering Dynamics or Not')

    # 随机森林地图；use_mockamap=false 时启动。
    map_generator_node = Node(
        package='map_generator',
        executable='random_forest',
        name='random_forest',
        output='screen',
        parameters=[
            {'map/x_size': 36.0},
            {'map/y_size': 20.0},
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

    # mockamap 地图；use_mockamap=true 时替代随机森林。
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
    
    # 10 架无人机的编号、起点和终点，坐标单位均为 m。
    # 编号必须从 0 连续递增，因为 C++ 中会用 drone_id 作为轨迹数组下标，
    # 并且无人机 i 会等待编号比自己小的无人机完成初始化。
    drone_configs = [
        {'drone_id': 0, 'init_x': -20.0, 'init_y': -9.0, 'init_z': 0.1, 'target_x': 20.0, 'target_y': 9.0, 'target_z': 1.0},
        {'drone_id': 1, 'init_x': -20.0, 'init_y': -7.0, 'init_z': 0.1, 'target_x': 20.0, 'target_y': 7.0, 'target_z': 1.0},
        {'drone_id': 2, 'init_x': -20.0, 'init_y': -5.0, 'init_z': 0.1, 'target_x': 20.0, 'target_y': 5.0, 'target_z': 1.0},
        {'drone_id': 3, 'init_x': -20.0, 'init_y': -3.0, 'init_z': 0.1, 'target_x': 20.0, 'target_y': 3.0, 'target_z': 1.0},
        {'drone_id': 4, 'init_x': -20.0, 'init_y': -1.0, 'init_z': 0.1, 'target_x': 20.0, 'target_y': 1.0, 'target_z': 1.0},
        {'drone_id': 5, 'init_x': -20.0, 'init_y': 1.0,  'init_z': 0.1, 'target_x': 20.0, 'target_y': -1.0, 'target_z': 1.0},
        {'drone_id': 6, 'init_x': -20.0, 'init_y': 3.0,  'init_z': 0.1, 'target_x': 20.0, 'target_y': -3.0, 'target_z': 1.0},
        {'drone_id': 7, 'init_x': -20.0, 'init_y': 5.0,  'init_z': 0.1, 'target_x': 20.0, 'target_y': -5.0, 'target_z': 1.0},
        {'drone_id': 8, 'init_x': -20.0, 'init_y': 7.0,  'init_z': 0.1, 'target_x': 20.0, 'target_y': -7.0, 'target_z': 1.0},
        {'drone_id': 9, 'init_x': -20.0, 'init_y': 9.0,  'init_z': 0.1, 'target_x': 20.0, 'target_y': -9.0, 'target_z': 1.0}
    ]

    # 将每条配置转换成一份 run_in_sim.launch.py 子启动描述。
    drone_nodes = []

    for config in drone_configs:        
        drone_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('ego_planner'), 'launch', 'run_in_sim.launch.py')),
            launch_arguments={
                # 注意：run_in_sim.launch.py 声明的名称是 use_dynamic；这里使用
                # use_dynamic_cmd 可能无法覆盖其默认值，排查开关问题时应检查此处。
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

    # 地图节点二选一，并且全体无人机只共享这一份地图。
    ld.add_action(map_generator_node)
    ld.add_action(mockamap_node)

    # 加入每架无人机的完整子系统。
    for drone in drone_nodes:        
        ld.add_action(drone)

    return ld
