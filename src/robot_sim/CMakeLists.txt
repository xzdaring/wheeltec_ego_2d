# 指定最低 CMake 版本，并使用与 package.xml 一致的包名。
cmake_minimum_required(VERSION 3.5)
project(robot_sim)

# 加载 ROS 2 的 CMake 构建支持；本包没有需要编译的 C++ 节点。
find_package(ament_cmake REQUIRED)

# 安装启动、机器人和世界资源，使 ros2 launch 能从安装目录找到它们。
install(DIRECTORY launch urdf worlds config models # 同时安装在线建图参数和 RViz 配置。
  DESTINATION share/${PROJECT_NAME}
)

# 注册包索引和环境文件；必须放在依赖与安装规则之后。
install(PROGRAMS # 安装导航适配和独立显示节点。
  scripts/ego_navigation_bridge.py # 地图与目标转换。
  scripts/navigation_visualizer.py # 曲线、行驶路径和参数文字。
  scripts/scan_self_filter.py # 按车体轮廓过滤实车自身回波。
  DESTINATION lib/${PROJECT_NAME}) # ros2 launch 的可执行查找目录。
install(FILES REAL_ROBOT.md DESTINATION share/${PROJECT_NAME})
ament_package()
