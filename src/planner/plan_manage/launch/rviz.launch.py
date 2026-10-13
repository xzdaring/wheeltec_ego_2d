import os

from launch import LaunchDescription
import launch_ros.actions
import launch_ros.descriptions
from ament_index_python.packages import get_package_share_directory

"""启动 RViz2，并自动加载本包提供的 default.rviz 显示配置。"""

def generate_launch_description():
    # 安装后通过 ROS 包索引定位共享目录，不能假定源码目录的绝对位置。
    rviz_config_path = os.path.join(get_package_share_directory('ego_planner'), 'launch', 'default.rviz')
    # --display-config 指定启动时加载的话题、坐标系、颜色和视角配置。
    rviz_node = launch_ros.actions.Node(
            package='rviz2', executable='rviz2', output='screen',
            arguments=['--display-config', rviz_config_path])

    # 定义 LaunchDescription，并把 RViz 进程交给 launch 系统管理。
    ld = LaunchDescription()

    # 添加节点。
    ld.add_action(rviz_node)

    return ld
