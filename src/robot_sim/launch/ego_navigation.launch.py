import os  # 通过包资源目录定位配置，避免依赖当前工作目录。
from ament_index_python.packages import get_package_share_directory  # 获取安装后的资源位置。
from launch import LaunchDescription  # 管理整条在线导航启动链。
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument  # 复用仿真与 EGO 的已有入口。
from launch.launch_description_sources import PythonLaunchDescriptionSource  # 加载 Python launch。
from launch.substitutions import LaunchConfiguration  # 允许无界面运行和选择出生位置。
from launch_ros.actions import Node  # 启动 SLAM、轨迹服务器、适配器和 RViz。
from launch.conditions import IfCondition  # 无界面验证时可关闭 RViz。


def generate_launch_description():  # ROS 2 launch 入口。
    sim = get_package_share_directory('robot_sim')  # 仿真包资源根目录。
    ego = get_package_share_directory('ego_planner')  # 使用当前 overlay 编译的二维 EGO。
    return LaunchDescription([  # 各节点等待话题和 TF 就绪，无需固定启动延时。
        DeclareLaunchArgument('gui', default_value='true'),  # 控制 Gazebo 图形窗口。
        DeclareLaunchArgument('rviz', default_value='true'),  # 控制 RViz 导航界面。
        DeclareLaunchArgument('x', default_value='1.0'),  # 默认出生在已有世界的房间内。
        DeclareLaunchArgument('y', default_value='2.0'),  # 可按世界空地调整出生位置。
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(sim, 'launch', 'v550_sim.launch.py')), 
                launch_arguments={'gui': LaunchConfiguration('gui'), 
                                  'x': LaunchConfiguration('x'), 
                                  'y': LaunchConfiguration('y')}.items()),  # Gazebo 提供底盘、雷达、里程计和 TF。
        Node(package='slam_toolbox', 
             executable='async_slam_toolbox_node', 
             name='slam_toolbox', 
             parameters=[os.path.join(sim, 'config', 'slam.yaml'),
                         {'use_sim_time': True}], 
                         output='screen'),  # 在线建图，不读取预存地图文件。
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ego, 'launch', 'advanced_param.launch.py')),
                launch_arguments={  'flight_type': '1', 
                                    'drone_id': '0', 
                                    'max_vel': '0.25', 
                                    'max_acc': '0.4', 
                                    'planning_horizon': '2.0', 
                                    'use_distinctive_trajs': 'False', 
                                    'obj_num_set': '0'}.items()),  # 复用 EGO 状态机，低速调试平面重规划。
        Node(package='ego_planner', 
             executable='traj_server', 
             name='traj_server', 
             parameters=[{'use_sim_time': True, 'traj_server/time_forward': 1.0}], 
             remappings=[('planning/bspline', '/drone_0_planning/bspline'), 
                         ('/position_cmd', '/ego/position_cmd')], 
             output='screen'),  # 原 EGO 采样器将 B 样条转为期望状态。
        Node(package='robot_sim', 
             executable='ego_navigation_bridge.py', 
             parameters=[{'use_sim_time': True}], 
             output='screen'),  # 地图与目标转换、全向反馈控制和近障停车。
        Node(package='rviz2', 
             executable='rviz2', 
             arguments=['-d', os.path.join(sim, 'config', 'navigation.rviz')], 
             parameters=[{'use_sim_time': True}], 
             condition=IfCondition(LaunchConfiguration('rviz')), 
             output='screen'),  # Goal Pose 在 map 中点击并显示在线建图。
    ])  # 返回完整导航流程。
