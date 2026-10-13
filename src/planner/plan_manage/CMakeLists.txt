cmake_minimum_required(VERSION 3.8)
# 虽然源码目录叫 plan_manage，ROS 2 包名和最终可执行文件所属包都是 ego_planner。
project(ego_planner)

# 本包使用智能指针、lambda 等现代 C++ 特性，统一要求 C++17。
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# Release + O3 用于提高在线规划速度；-g 同时保留调试符号，便于定位崩溃。
set(CMAKE_BUILD_TYPE "Release")

add_compile_options(-Wall -O3 -g)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找 ROS 2 构建系统、消息包和本工程其他功能包。
# CMake 在这里找到依赖后，下面才能获得相应的头文件、库和链接参数。
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED) # 小车雷达需要真实坐标转换。
find_package(sensor_msgs REQUIRED) # 雷达和障碍点云看门狗。
find_package(geometry_msgs REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(plan_env REQUIRED)
find_package(path_searching REQUIRED)
find_package(bspline_opt REQUIRED)
find_package(traj_utils REQUIRED COMPONENTS traj_utils_lib)
find_package(cv_bridge REQUIRED)

# Eigen 用于向量/矩阵运算；PCL 用于点云数据；pcl_conversions 负责 ROS/PCL 转换。
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)

if(BUILD_TESTING)
  # 仅在启用测试时配置 ROS 2 的静态检查工具。
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(
  # 本包公开头文件位于 include/ego_planner/。
  include
  SYSTEM
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

add_executable(ego_planner_node
  # 主入口 + 有限状态机 + 规划管理器共同链接为规划节点。
  src/ego_planner_node.cpp 
  src/ego_replan_fsm.cpp
  src/planner_manager.cpp
)

# target_link_libraries(ego_planner_node
#   traj_utils_lib
# )

ament_target_dependencies(ego_planner_node 
                          # ament 自动补齐这些包的 include 路径和链接选项。
                          rclcpp nav_msgs visualization_msgs 
                          geometry_msgs quadrotor_msgs 
                          plan_env path_searching 
                          bspline_opt traj_utils 
                          cv_bridge pcl_conversions )

# traj_utils 还提供一个需要显式链接的实际库文件。
find_library(TRAJ_UTILS_LIB traj_utils_lib PATHS ${traj_utils_LIBRARY_DIRS})

target_link_libraries(ego_planner_node
    ${TRAJ_UTILS_LIB}
)

# 第二个可执行程序：读取 B 样条并连续发布飞控所需的位置指令。
add_executable(traj_server src/traj_server.cpp)

ament_target_dependencies(traj_server 
                          rclcpp nav_msgs 
                          visualization_msgs geometry_msgs 
                          quadrotor_msgs plan_env 
                          path_searching bspline_opt 
                          traj_utils cv_bridge pcl_conversions)

# 小车版服务器直接对 B 样条求值并输出全向底盘速度。
add_executable(ground_traj_server src/ground_traj_server.cpp)
ament_target_dependencies(ground_traj_server
  rclcpp tf2_ros sensor_msgs geometry_msgs nav_msgs # 反馈与感知接口。
  quadrotor_msgs bspline_opt traj_utils) # 复用 EGO 轨迹求值和消息。

# 安装后可通过 `ros2 run ego_planner ego_planner_node` 和
# `ros2 run ego_planner traj_server` 启动这两个程序。
install(
  TARGETS ego_planner_node traj_server ground_traj_server # 同时保留无人机原版。
  DESTINATION lib/${PROJECT_NAME}
)

# 把 Python launch 与 default.rviz 一起复制到 install/ego_planner/share/ego_planner/launch。
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# 生成 ament 包索引和环境配置；通常必须放在 CMakeLists.txt 末尾。
ament_package()
