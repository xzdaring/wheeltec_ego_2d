#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <iostream>

#include <ego_planner/ego_replan_fsm.h>

using namespace ego_planner;

/**
 * @brief EGO-Planner 的 ROS 2 进程入口。
 *
 * 这个 main 函数本身不执行规划算法。它只完成三件事：
 * 1. 创建 ROS 2 节点；
 * 2. 创建并初始化 EGOReplanFSM（重规划有限状态机）；
 * 3. 进入 rclcpp 事件循环，让定时器和订阅回调持续工作。
 *
 * 继续阅读时应跳转到 EGOReplanFSM::init()。其中注册的 10 ms 定时器会
 * 周期调用 execFSMCallback()，那才是在线规划流程的实际入口。
 */
int main(int argc, char **argv)
{
  // 初始化 ROS 2 客户端库，解析并移除 ROS 专用命令行参数。
  rclcpp::init(argc, argv);

  // 此处是节点的默认名称；launch 文件通常会把它重命名为
  // drone_<id>_ego_planner_node，以便多架无人机各自运行一个规划器。
  auto node = std::make_shared<rclcpp::Node>("ego_planner_node");

  // 状态机负责“何时规划/重规划/急停”；具体优化工作由其内部的
  // EGOPlannerManager 和 BsplineOptimizer 完成。
  EGOReplanFSM rebo_replan;
  rebo_replan.init(node);

  // 单线程事件循环。里程计、目标、多机轨迹、地图数据和两个定时器都在
  // 回调中驱动系统运行；spin() 会一直阻塞到节点收到退出请求。
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}

// #include <ros/ros.h>
// #include <csignal>
// #include <visualization_msgs/Marker.h>

// #include <plan_manage/ego_replan_fsm.h>

// using namespace ego_planner;

// void SignalHandler(int signal) {
//   if(ros::isInitialized() && ros::isStarted() && ros::ok() && !ros::isShuttingDown()){
//     ros::shutdown();
//   }
// }

// int main(int argc, char **argv) {

//   signal(SIGINT, SignalHandler);
//   signal(SIGTERM,SignalHandler);

//   ros::init(argc, argv, "ego_planner_node", ros::init_options::NoSigintHandler);
//   ros::NodeHandle nh("~");

//   EGOReplanFSM rebo_replan;

//   rebo_replan.init(nh);

//   // ros::Duration(1.0).sleep();
//   ros::AsyncSpinner async_spinner(4);
//   async_spinner.start();
//   ros::waitForShutdown();

//   return 0;
// }
