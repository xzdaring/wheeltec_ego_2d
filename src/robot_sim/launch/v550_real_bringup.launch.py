"""V550_mec + UART 雷达实车底盘入口；仅在小车主机运行。"""
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
        DeclareLaunchArgument('lidar_crop_mode', default_value='0', choices=['0', '1'],
                              description='0 保留原始扫描供过滤；1 驱动直接执行厂家全距离屏蔽'),
        DeclareLaunchArgument('factory_mask_mode', default_value='full',
                              choices=['near', 'full', 'off'],
                              description='厂家角度过滤：near 仅近点，full 全距离，off 仅矩形'),
        DeclareLaunchArgument('factory_near_range', default_value='0.26',
                              description='厂家遮挡角度内的近距回波门槛，距雷达的米数'),
        DeclareLaunchArgument('lidar_config', default_value=str(
            # 本车串口实测为 500000 波特率、M10_P 数据格式；其他设备可覆盖。
            lidar / 'params/lidar_uart_ros2/lsm10_p.yaml')),
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
        # 复用厂家 UART 驱动，并显式覆盖输出和屏蔽参数。
        Node(package='lslidar_driver', 
             executable='lslidar_driver_node', 
             name='lslidar_driver_node',
             parameters=[LaunchConfiguration('lidar_config'),
                         {'use_sim_time': False, 
                          'scan_topic': '/scan_raw',
                          'truncated_mode_': ParameterValue(
                              LaunchConfiguration('lidar_crop_mode'), value_type=int),
                          'serial_port_': LaunchConfiguration('lidar_port')}],
             output='screen'),
        # 车体矩形与厂家遮挡角度共同过滤；SLAM、规划器和跟踪器使用同一份 /scan。
        Node(package='robot_sim', executable='scan_self_filter.py',
             parameters=[{'use_sim_time': False,
                          # off 在 YAML 中会被推断为布尔值，必须显式保留字符串类型。
                          'factory_mask_mode': ParameterValue(
                              LaunchConfiguration('factory_mask_mode'), value_type=str),
                          'factory_near_range': ParameterValue(
                              LaunchConfiguration('factory_near_range'), value_type=float)}],
             output='screen'),
    ])
