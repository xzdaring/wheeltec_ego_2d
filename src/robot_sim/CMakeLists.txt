# 指定最低 CMake 版本，并使用与 package.xml 一致的包名。
cmake_minimum_required(VERSION 3.5)
project(robot_sim)

# 加载 ROS 2 的 CMake 构建支持；本包没有需要编译的 C++ 节点。
find_package(ament_cmake REQUIRED)

# 安装启动、机器人和世界资源，使 ros2 launch 能从安装目录找到它们。
install(DIRECTORY launch urdf worlds
  DESTINATION share/${PROJECT_NAME}
)

# 注册包索引和环境文件；必须放在依赖与安装规则之后。
ament_package()
