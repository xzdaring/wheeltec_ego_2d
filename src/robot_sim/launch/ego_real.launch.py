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
        DeclareLaunchArgument('odom_topic', default_value='/odom_combined'),
        DeclareLaunchArgument('planning_frame', default_value='odom_combined'),
        DeclareLaunchArgument('base_frame', default_value='base_footprint'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('enable_motion', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('start_slam', default_value='true'),
        DeclareLaunchArgument('safety_radius', default_value='0.25',
                              description='车体平面外接圆半径加余量，按实物测量调整'),
        Node(package='slam_toolbox', executable='async_slam_toolbox_node', name='slam_toolbox',
             parameters=[str(resources / 'config/slam.yaml'),
                         {'use_sim_time': False, 'odom_frame': frame, 'base_frame': base,
                          'scan_topic': scan}],
             condition=IfCondition(LaunchConfiguration('start_slam')), output='screen'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            str(ego / 'launch/advanced_param.launch.py')),
            launch_arguments={'use_sim_time': 'false', 'planning_frame': frame,
                              'odometry_topic': odom, 'cloud_topic': '/ego/cloud',
                              'flight_type': '1', 'drone_id': '0', 'max_vel': '0.12',
                              'max_acc': '0.2', 'planning_horizon': '5.0',
                              'use_distinctive_trajs': 'False', 'obj_num_set': '0'}.items()),
        Node(package='ego_planner', executable='ground_traj_server', name='ground_traj_server',
             parameters=[{'use_sim_time': False, 'planning_frame': frame, 'base_frame': base,
                          'max_speed': 0.15, 'max_yaw_rate': 0.25,
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
