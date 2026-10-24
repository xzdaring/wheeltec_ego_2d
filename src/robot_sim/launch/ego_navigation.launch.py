from launch_ros.parameter_descriptions import ParameterValue
import os  # 通过包资源目录定位配置，避免依赖当前工作目录。
from ament_index_python.packages import get_package_share_directory  # 获取安装后的资源位置。
from launch import LaunchDescription  # 管理整条在线导航启动链。
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument  # 复用仿真与 EGO 的已有入口。
from launch.launch_description_sources import PythonLaunchDescriptionSource  # 加载 Python launch。
from launch.substitutions import LaunchConfiguration, PythonExpression  # 允许无界面运行和选择出生位置。
from launch_ros.actions import Node  # 启动 SLAM、轨迹服务器、适配器和 RViz。
from launch.conditions import IfCondition  # 无界面验证时可关闭 RViz。


def generate_launch_description():  # ROS 2 launch 入口。
    sim = get_package_share_directory('robot_sim')  # 仿真包资源根目录。
    ego = get_package_share_directory('ego_planner')  # 使用当前 overlay 编译的二维 EGO。
    return LaunchDescription([
        DeclareLaunchArgument('map_resolution', default_value='0.1'),
        DeclareLaunchArgument('map_size_x_', default_value='42.0'),
        DeclareLaunchArgument('map_size_y_', default_value='30.0'),
        # 各节点等待话题和 TF 就绪，无需固定启动延时。
        DeclareLaunchArgument('gui', default_value='true'),  # 控制 Gazebo 图形窗口。
        DeclareLaunchArgument('rviz', default_value='true'),  # 控制 RViz 导航界面。
        DeclareLaunchArgument('x', default_value='-4.1'),  # 默认出生在已有世界的房间内。
        DeclareLaunchArgument('y', default_value='-3.3'),  # 可按世界空地调整出生位置。
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
                                    'planning_horizon': '5.0', 
                                    'use_distinctive_trajs': 'False', 
                                    'obj_num_set': '0'}.items()),  # 复用 EGO 状态机，低速调试平面重规划。
        Node( # 小车版直接采样轨迹并闭环输出 /cmd_vel。
            package='ego_planner', # 使用原 EGO 包的新增可执行程序。
            executable='ground_traj_server', # 原 traj_server 仍保留但不同时启动。
            name='ground_traj_server', # 独立名称方便查看参数。
            parameters=[{'use_sim_time': True, 'grid_origin_x': ParameterValue(PythonExpression(['-0.5 * ', LaunchConfiguration('map_size_x_')]), value_type=float),
                          'grid_origin_y': ParameterValue(PythonExpression(['-0.5 * ', LaunchConfiguration('map_size_y_')]), value_type=float),
                          'grid_origin_z': -0.5,
                          'grid_resolution': ParameterValue(LaunchConfiguration('map_resolution'), value_type=float)}], # 与规划器共用栅格几何。
            output='screen'), # 输出到点与运行状态。
        Node(package='robot_sim', 
             executable='ego_navigation_bridge.py', 
             parameters=[{'use_sim_time': True}], 
             output='screen'),  # 仅负责地图与目标转换，车速由 C++ 节点唯一发布。
        Node( # 显示真实曲线、全局参考和运行参数，不参与控制。
            package='robot_sim', # 与仿真一起安装。
            executable='navigation_visualizer.py', # 独立低频可视化节点。
            parameters=[{'use_sim_time': True}], # 所有标记跟随仿真时间。
            output='screen'), # 显示失败可在终端诊断。
        Node(package='rviz2', 
             executable='rviz2', 
             arguments=['-d', os.path.join(sim, 'config', 'navigation.rviz')], 
             parameters=[{'use_sim_time': True}], 
             condition=IfCondition(LaunchConfiguration('rviz')), 
             output='screen'),  # Goal Pose 在 map 中点击并显示在线建图。
    ])  # 返回完整导航流程。
