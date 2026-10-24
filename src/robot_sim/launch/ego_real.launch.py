"""接入已启动的真实底盘与 N10P；不启动 Gazebo，不打开串口。"""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    resources = Path(get_package_share_directory('robot_sim'))
    ego = Path(get_package_share_directory('ego_planner'))
    frame = LaunchConfiguration('planning_frame')
    base = LaunchConfiguration('base_frame')
    odom = LaunchConfiguration('odom_topic')
    scan = LaunchConfiguration('scan_topic')
    # 预览模式仍运行完整规划、跟踪，只把速度输出到不连接底盘的话题。
    output = PythonExpression(["'/cmd_vel' if '", LaunchConfiguration('enable_motion'),
                               "'.lower() == 'true' else '/ego/cmd_vel_preview'"])
    remaps = [('/odom', odom), ('/scan', scan)]
    return LaunchDescription([
        DeclareLaunchArgument('map_resolution', default_value='0.1'),
        DeclareLaunchArgument('map_size_x_', default_value='42.0'),
        DeclareLaunchArgument('map_size_y_', default_value='30.0'),

        DeclareLaunchArgument('odom_topic', default_value='/odom_combined'),
        DeclareLaunchArgument('planning_frame', default_value='odom_combined'),
        DeclareLaunchArgument('base_frame', default_value='base_footprint'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('enable_motion', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('rviz', default_value='false'),
        # 规划速度与控制上限分别配置；控制上限应留出位置纠偏余量。
        DeclareLaunchArgument('max_vel', default_value='0.12'),
        DeclareLaunchArgument('max_speed', default_value='0.15'),
        DeclareLaunchArgument('max_yaw_rate', default_value='0.25'),
        DeclareLaunchArgument('max_tracking_error', default_value='0.20'),

        DeclareLaunchArgument('follow_trajectory_yaw', default_value='true', # 默认车头沿轨迹走。
                              choices=['true', 'false']), # 可关闭行驶转向，保留最终对齐。
        DeclareLaunchArgument('yaw_lookahead', default_value='0.3'), # 切线预瞄时间，单位秒。
        DeclareLaunchArgument('start_slam', default_value='true'),
        DeclareLaunchArgument('safety_radius', default_value='0.25',
                              description='车体平面外接圆半径加余量，按实物测量调整'),
        Node(package='slam_toolbox', executable='async_slam_toolbox_node', name='slam_toolbox',
             parameters=[str(resources / 'config/slam.yaml'),
                         {'use_sim_time': False, 'odom_frame': frame, 'base_frame': base,
                          'scan_topic': scan,
                          # 雷达约 12 Hz、EKF 10 Hz，允许扫描等待下一帧 TF。
                          'scan_queue_size': 10, 'min_laser_range': 0.2}],
             condition=IfCondition(LaunchConfiguration('start_slam')), output='screen'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            str(ego / 'launch/advanced_param.launch.py')),
            launch_arguments={'use_sim_time': 'false', 'planning_frame': frame,
                              'odometry_topic': odom, 'cloud_topic': '/ego/cloud',
                              'map_resolution': LaunchConfiguration('map_resolution'),
                              'flight_type': '1', 'drone_id': '0', 'max_vel': LaunchConfiguration('max_vel'),
                              'max_tracking_error': LaunchConfiguration('max_tracking_error'),
                              'max_acc': '0.2', 'planning_horizon': '5.0',
                              'use_distinctive_trajs': 'False', 'obj_num_set': '0'}.items()),
        Node(package='ego_planner', executable='ground_traj_server', name='ground_traj_server',
             parameters=[{'use_sim_time': False, 'planning_frame': frame, 'base_frame': base,
                          'grid_origin_x': ParameterValue(PythonExpression(['-0.5 * ', LaunchConfiguration('map_size_x_')]), value_type=float),
                          'grid_origin_y': ParameterValue(PythonExpression(['-0.5 * ', LaunchConfiguration('map_size_y_')]), value_type=float),
                          'grid_origin_z': -0.5,
                          'grid_resolution': ParameterValue(LaunchConfiguration('map_resolution'), value_type=float),
                          'max_speed': ParameterValue(LaunchConfiguration('max_speed'), value_type=float),
                          'max_yaw_rate': ParameterValue(LaunchConfiguration('max_yaw_rate'), value_type=float), # 实车保持原有低速上限。
                          'follow_trajectory_yaw': ParameterValue( # 显式转换布尔 launch 参数。
                              LaunchConfiguration('follow_trajectory_yaw'), value_type=bool), # 允许切换。
                          'yaw_lookahead': ParameterValue( # 预瞄时间需要作为浮点参数传入。
                              LaunchConfiguration('yaw_lookahead'), value_type=float), # 避免字符串类型。
                          'safety_radius': ParameterValue(LaunchConfiguration('safety_radius'), value_type=float)}],
             remappings=remaps + [('/cmd_vel', output)], output='screen'),
        Node(package='robot_sim', executable='ego_navigation_bridge.py',
             parameters=[{'use_sim_time': False, 'planning_frame': frame}],
             remappings=remaps, output='screen'),
        Node(package='robot_sim', executable='navigation_visualizer.py',
             parameters=[{'use_sim_time': False, 'planning_frame': frame}],
             remappings=remaps, output='screen'),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', str(resources / 'config/navigation.rviz')],
             parameters=[{'use_sim_time': False}],
             condition=IfCondition(LaunchConfiguration('rviz')), output='screen'),
    ])
