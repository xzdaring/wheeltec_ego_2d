# os.path 用来拼接跨目录路径；不把本机源码绝对路径写进启动文件。
import os

# xacro 将包含外部模型和仿真插件的描述展开为完整 URDF 字符串。
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 查安装后的资源目录；因此首次启动前必须 colcon build 并 source。
    sim_share = get_package_share_directory('robot_sim')
    gazebo_share = get_package_share_directory('gazebo_ros')
    # 使用包内模型资源，同时保留用户原有搜索目录。
    bundled_models = os.path.join(sim_share, 'models')
    previous_models = os.environ.get('GAZEBO_MODEL_PATH', '')
    model_path = os.pathsep.join(  # 包内资源优先，避免依赖个人缓存。
        path for path in (bundled_models, previous_models) if path)

    # launch 参数允许更换世界、关闭 GUI 和调整出生位置，无需修改源码。
    world = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui')
    spawn_x = LaunchConfiguration('x')
    spawn_y = LaunchConfiguration('y')
    spawn_z = LaunchConfiguration('z')

    # 只展开一次，使 Gazebo 和状态发布器使用完全相同的 link/joint 定义。
    description_path = os.path.join(sim_share, 'urdf', 'v550_sim.urdf.xacro')
    description = xacro.process_file(description_path)
    # Gazebo Classic 会将 package:// 视为模型资源；若未配置模型搜索目录，
    # 就会找不到 STL，甚至反复尝试联网搜索。这里通过 ROS 包索引找到本地文件。
    # 只改展开后的描述，原 wheeltec_robot_urdf 文件仍使用可移植的 package://。
    for mesh in description.getElementsByTagName('mesh'):
        uri = mesh.getAttribute('filename')
        if uri.startswith('package://'):
            package_name, relative_path = uri[len('package://'):].split('/', 1)
            mesh_path = os.path.join(get_package_share_directory(package_name), relative_path)
            # 缺失资源时在启动阶段明确报错，避免 Gazebo 静默生成没有碰撞外形的车。
            if not os.path.isfile(mesh_path):
                raise FileNotFoundError(f'Robot mesh not found: {mesh_path}')
            mesh.setAttribute('filename', 'file://' + mesh_path)
    robot_description = description.toxml()

    return LaunchDescription([
        # Gazebo 的 model:// 引用从这些目录查找网格、材质和纹理。
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', model_path),
        # GUI 的模型面板默认会联网刷新模型库；网络慢时会卡在 Preparing your world。
        # 空 URI 禁用在线模型数据库，已保存的世界与本地模型仍可正常加载。
        # 若确实需要联网下载模型，可用 model_database_uri:=http://models.gazebosim.org/。
        DeclareLaunchArgument('model_database_uri', default_value='',
                              description='Online model database URI; empty means offline'),
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI',
                               LaunchConfiguration('model_database_uri')),
        # 默认使用你保存的世界；也可 world:=/绝对路径/其他.world。
        DeclareLaunchArgument(
            'world', default_value=os.path.join(sim_share, 'worlds', 'ego_test_world2.world'),
            description='Gazebo world file'),
        # gui:=false 只运行服务器，适合无界面检查。
        DeclareLaunchArgument('gui', default_value='true', description='Start Gazebo GUI'),
        # 单位为米；x/y 应选在世界中的空地，z 给模型留出初始落地高度。
        DeclareLaunchArgument('x', default_value='0.0', description='Initial X in metres'),
        DeclareLaunchArgument('y', default_value='0.0', description='Initial Y in metres'),
        DeclareLaunchArgument('z', default_value='0.15', description='Initial Z in metres'),

        # gazebo_ros 启动服务器并加载 ROS 初始化与生成实体插件，提供 /clock。
        # 不要同时另开一个占用相同端口的 Gazebo 服务器。
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_share, 'launch', 'gazebo.launch.py')),
            launch_arguments={'world': world, 'gui': gui}.items(),
        ),

        # 固定关节发布到 /tf_static，活动关节根据 /joint_states 发布到 /tf。
        # use_sim_time 使时间跟随 /clock；暂停仿真时 ROS 时间也暂停。
        Node(
            package='robot_state_publisher', 
            executable='robot_state_publisher',
            name='robot_state_publisher', 
            output='screen',
            parameters=[{'robot_description': robot_description, 
                         'use_sim_time': True}],
        ),

        # 等待生成实体服务，再从 robot_description 话题读取 URDF，放入 Gazebo。
        # -entity 是 Gazebo 中的唯一模型名；-x/-y/-z 是模型初始位置。
        # 这等价于前面介绍的 ros2 run gazebo_ros spawn_entity.py 命令。
        Node(
            package='gazebo_ros', 
            executable='spawn_entity.py', 
            output='screen',
            arguments=['-topic', 'robot_description', '-entity', 'v550_mec',
                       '-x', spawn_x, '-y', spawn_y, '-z', spawn_z],
            parameters=[{'use_sim_time': True}],
        ),
    ])
