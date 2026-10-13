import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

"""一架无人机的通用仿真启动单元，主要由 swarm*.launch.py 重复调用。

与 single_run_in_sim.launch.py 相比，本文件把初始位置和目标位置都暴露成参数，
所以同一份启动逻辑可以用于编号和起终点不同的多架无人机。
"""

def generate_launch_description():
    # 地图尺寸，单位 m。
    map_size_x = LaunchConfiguration('map_size_x', default=42.0)
    map_size_y = LaunchConfiguration('map_size_y', default=30.0)
    map_size_z = LaunchConfiguration('map_size_z', default=5.0)
    # 本架无人机的初始位置，单位 m。
    init_x = LaunchConfiguration('init_x', default=0.0)
    init_y = LaunchConfiguration('init_y', default=0.0)
    init_z = LaunchConfiguration('init_z', default=0.0)
    # 本架无人机的目标位置，单位 m。
    target_x = LaunchConfiguration('target_x', default=20.0)
    target_y = LaunchConfiguration('target_y', default=20.0)
    target_z = LaunchConfiguration('target_z', default=1.0)
    # drone_id 同时参与节点名、话题名和多机轨迹数组索引，必须从 0 连续编号。
    drone_id = LaunchConfiguration('drone_id', default=0)
    # 该无人机的里程计输入话题。
    odom_topic = LaunchConfiguration('odom_topic', default='visual_slam/odom')
    # 环境中的移动物体数量；会传给规划器的预测模块。
    obj_num = LaunchConfiguration('obj_num', default=10)

    # 将上述配置正式声明为可由父 launch 或命令行传入的参数。
    map_size_x_cmd = DeclareLaunchArgument('map_size_x', default_value=map_size_x, description='Map size along x')
    map_size_y_cmd = DeclareLaunchArgument('map_size_y', default_value=map_size_y, description='Map size along y')
    map_size_z_cmd = DeclareLaunchArgument('map_size_z', default_value=map_size_z, description='Map size along z')
    init_x_cmd = DeclareLaunchArgument('init_x', default_value=init_x, description='Initial x position of the drone')
    init_y_cmd = DeclareLaunchArgument('init_y', default_value=init_y, description='Initial y position of the drone')
    init_z_cmd = DeclareLaunchArgument('init_z', default_value=init_z, description='Initial z position of the drone')
    target_x_cmd = DeclareLaunchArgument('target_x', default_value=target_x, description='Target x position')
    target_y_cmd = DeclareLaunchArgument('target_y', default_value=target_y, description='Target y position')
    target_z_cmd = DeclareLaunchArgument('target_z', default_value=target_z, description='Target z position')
    drone_id_cmd = DeclareLaunchArgument('drone_id', default_value=drone_id, description='ID of the drone')
    odom_topic_cmd = DeclareLaunchArgument('odom_topic', default_value=odom_topic, description='Odometry topic')
    obj_num_cmd = DeclareLaunchArgument('obj_num', default_value=obj_num, description='Number of moving objects')

    # true：使用控制器和动力学仿真；false：直接把位置指令转换为里程计。
    use_dynamic = LaunchConfiguration('use_dynamic', default=True)  
    use_dynamic_cmd = DeclareLaunchArgument('use_dynamic', default_value=use_dynamic, description='Use Drone Simulation Considering Dynamics or Not')

    # 启动 EGO-Planner 主节点，并向其传入当前无人机的参数。
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
            # 深度相机内参，单位 pixel。
            'cx': str(321.04638671875),
            'cy': str(243.44969177246094),
            'fx': str(387.229248046875),
            'fy': str(387.229248046875),
            'max_vel': str(2.0),           # 最大速度，m/s
            'max_acc': str(3.0),           # 最大加速度，m/s^2
            'planning_horizon': str(7.5),  # 局部规划范围，m
            'use_distinctive_trajs': 'True', # 是否尝试拓扑不同的候选轨迹
            'flight_type': str(2),         # 2：预设航点模式
            'point_num': str(1),           # 只使用 point0，也就是父 launch 给定的目标
            'point0_x': target_x,
            'point0_y': target_y,
            'point0_z': target_z,
            
            'point1_x': str(0.0),
            'point1_y': str(15.0),
            'point1_z': str(1.0),
            
            'point2_x': str(15.0),
            'point2_y': str(0.0),
            'point2_z': str(1.0),
            
            'point3_x': str(0.0),
            'point3_y': str(-15.0),
            'point3_z': str(1.0),
            
            'point4_x': str(-15.0),
            'point4_y': str(0.0),
            'point4_z': str(1.0),
        }.items()
    )

    # 轨迹服务器：把规划器的 B 样条转换成实时 position_cmd。
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
            # 计算机头朝向时观察轨迹前方多远，单位 s；不是碰撞检查参数。
            {'traj_server/time_forward': 1.0}
        ]
    )

    # 启动本架无人机对应的仿真组件。
    simulator_include = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('ego_planner'), 'launch', 'simulator.launch.py')),
        launch_arguments={
            # 注意：simulator.launch.py 声明的参数名是 use_dynamic，
            # 这里却传入 use_dynamic_cmd。按当前代码，该值可能不会覆盖 simulator
            # 中的默认值；若发现动力学开关无效，应优先检查这处命名是否需要统一。
            'use_dynamic_cmd': use_dynamic,
            'drone_id': drone_id,
            'map_size_x_': map_size_x,
            'map_size_y_': map_size_y,
            'map_size_z_': map_size_z,
            'init_x_': init_x,
            'init_y_': init_y,
            'init_z_': init_z,
            'odometry_topic': odom_topic
        }.items()
    )

    # 移动物体生成器。参数定义保留在这里，但文件末尾并未 add_action，
    # 因此按当前代码它不会真正启动。
    obj_generator_node = Node(
        package='plan_env',
        executable='obj_generator',
        name='obj_generator',
        output='screen',
        parameters=[
            {'obj_generator/obj_num': obj_num},
            {'obj_generator/x_size': 12.0},
            {'obj_generator/y_size': 12.0},
            {'obj_generator/h_size': 1.0},
            {'obj_generator/vel': 1.5},     # 平移速度量级，m/s
            {'obj_generator/yaw_dot': 2.0}, # 偏航角速度，rad/s
            {'obj_generator/acc_r1': 1.0},
            {'obj_generator/acc_r2': 1.0},
            {'obj_generator/acc_z': 0.0},
            {'obj_generator/scale1': 0.5},
            {'obj_generator/scale2': 1.0},
            {'obj_generator/interval': 100.0},
            {'obj_generator/input_type': 1}
        ]
    )

    # 构造启动描述：先注册参数，再加入节点和子 launch。
    ld = LaunchDescription()

    # Add arguments
    ld.add_action(map_size_x_cmd)
    ld.add_action(map_size_y_cmd)
    ld.add_action(map_size_z_cmd)
    ld.add_action(init_x_cmd)
    ld.add_action(init_y_cmd)
    ld.add_action(init_z_cmd)
    ld.add_action(target_x_cmd)
    ld.add_action(target_y_cmd)
    ld.add_action(target_z_cmd)
    ld.add_action(drone_id_cmd)
    ld.add_action(odom_topic_cmd)
    ld.add_action(obj_num_cmd)
    ld.add_action(use_dynamic_cmd)

    # Add nodes and includes
    ld.add_action(advanced_param_include)
    ld.add_action(traj_server_node)
    ld.add_action(simulator_include)
    # 当前被注释，因此上面创建的移动物体生成器不会运行。
    # ld.add_action(obj_generator_node)    

    return ld
