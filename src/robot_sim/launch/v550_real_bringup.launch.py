"""V550_mec + N10P UART 实车底盘入口；仅在小车主机运行。"""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    driver = Path(get_package_share_directory('turn_on_wheeltec_robot'))
    lidar = Path(get_package_share_directory('lslidar_driver'))
    model = Path(get_package_share_directory('wheeltec_robot_urdf'))
    # 用原始 CAD 模型发布真实安装 TF，不包含 Gazebo 中抬高的 laser_scan。
    robot_description = (model / 'urdf/V550_mec_robot.urdf').read_text()
    return LaunchDescription([
        DeclareLaunchArgument('base_port', default_value='/dev/wheeltec_controller'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/wheeltec_lidar'),
        DeclareLaunchArgument('baud', default_value='115200'),
        DeclareLaunchArgument('lidar_config', default_value=str(
            lidar / 'params/lidar_uart_ros2/lsn10p.yaml')),
        Node(package='turn_on_wheeltec_robot', executable='wheeltec_robot_node',
             parameters=[{'use_sim_time': False,
                          'usart_port_name': LaunchConfiguration('base_port'),
                          'serial_baud_rate': ParameterValue(LaunchConfiguration('baud'), value_type=int),
                          'robot_frame_id': 'base_footprint',
                          'odom_frame_id': 'odom_combined',
                          'gyro_frame_id': 'gyro_link',
                          'odom_x_scale': 1.0, 'odom_y_scale': 1.0,
                          'odom_z_scale_positive': 1.0, 'odom_z_scale_negative': 1.0}],
             output='screen'),
        # EKF 是 odom_combined -> base_footprint 的唯一发布者，保留麦轮 vy 融合。
        Node(package='robot_localization', 
             executable='ekf_node', 
             name='ekf_filter_node',
             parameters=[str(driver / 'config/ekf.yaml'), 
                         {'use_sim_time': False}],
             remappings=[('/odometry/filtered', '/odom_combined')], 
             output='screen'),
        Node(package='robot_state_publisher', 
             executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description, 
                          'use_sim_time': False}]),
        Node(package='joint_state_publisher',
             executable='joint_state_publisher',
             parameters=[{'use_sim_time': False}]),
        Node(package='tf2_ros', 
             executable='static_transform_publisher', 
             name='base_to_link',
             arguments=['--x', '0', '--y', '0', '--z', '0.02027',
                        '--frame-id', 'base_footprint', '--child-frame-id', 'base_link']),
        Node(package='tf2_ros', 
             executable='static_transform_publisher', 
             name='base_to_gyro',
             arguments=['--frame-id', 'base_footprint', 
                        '--child-frame-id', 'gyro_link']),
        # 复用 N10P 厂家参数和驱动，串口可覆盖，不选择 wheeltec_lidar 的 M10P 默认分支。
        Node(package='lslidar_driver', 
             executable='lslidar_driver_node', 
             name='lslidar_driver_node',
             parameters=[LaunchConfiguration('lidar_config'),
                         {'use_sim_time': False, 
                          'serial_port_': LaunchConfiguration('lidar_port')}],
             output='screen'),
    ])
