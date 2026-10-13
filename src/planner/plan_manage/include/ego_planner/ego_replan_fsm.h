#ifndef _REBO_REPLAN_FSM_H_
#define _REBO_REPLAN_FSM_H_

/**
 * @file ego_replan_fsm.h
 * @brief EGO-Planner 在线重规划状态机的声明。
 *
 * 初学者阅读提示：本类主要负责“事件和流程”，不直接实现 B 样条代价函数。
 * 订阅回调把外部数据写入成员变量，两个定时器分别推进状态机和检查安全，
 * 需要真正生成轨迹时再调用 EGOPlannerManager。
 */

#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include <vector>
#include "visualization_msgs/msg/marker.hpp"

#include "bspline_opt/bspline_optimizer.h"
#include "plan_env/grid_map.h"
#include "traj_utils/msg/bspline.hpp"
#include "traj_utils/msg/multi_bsplines.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "traj_utils/msg/data_disp.hpp"
#include "ego_planner/planner_manager.h"
#include "traj_utils/planning_visualization.h"

using std::vector;

namespace ego_planner
{

  /**
   * @brief 在线重规划有限状态机。
   *
   * 职责边界：
   * - 本类决定“什么时候规划”，并管理 ROS 话题、定时器与状态转换；
   * - EGOPlannerManager 决定“一次规划怎样完成”；
   * - BsplineOptimizer 负责控制点的数值优化；
   * - traj_server（另一个可执行节点）负责按时间对最终 B 样条求值。
   */
  class EGOReplanFSM
  {

  private:
    /* ---------- flag ---------- */
    enum FSM_EXEC_STATE
    {
      INIT,             ///< 等待第一帧有效里程计。
      WAIT_TARGET,      ///< 已有状态，等待目标点和启动触发。
      GEN_NEW_TRAJ,     ///< 从全局参考轨迹截取局部段并生成新轨迹。
      REPLAN_TRAJ,      ///< 从当前正在执行的轨迹状态平滑地重新规划。
      EXEC_TRAJ,        ///< 正在执行局部 B 样条，同时判断是否需要重规划。
      EMERGENCY_STOP,   ///< 发布位置恒定的 B 样条，请求无人机悬停。
      SEQUENTIAL_START  ///< 多机首次启动：等待较小编号无人机先发布轨迹。
    };
    enum TARGET_TYPE
    {
      MANUAL_TARGET = 1, ///< 订阅 RViz 的 /move_base_simple/goal。
      PRESET_TARGET = 2, ///< 使用 launch 参数中的 waypoint0、waypoint1……。
      REFENCE_PATH = 3   ///< 预留的参考路径模式，当前初始化逻辑并未完整实现。
    };

    /* planning utils */
    // planner_manager_ 保存地图、全局/局部轨迹并组织每次 B 样条优化。
    EGOPlannerManager::Ptr planner_manager_;
    // visualization_ 只负责发布 RViz Marker，不参与规划结果计算。
    PlanningVisualization::Ptr visualization_;
    traj_utils::msg::DataDisp data_disp_; ///< 规划耗时等调试统计数据。
    traj_utils::msg::MultiBsplines multi_bspline_msgs_buf_; ///< 多机首次启动时沿编号链传递的轨迹集合。

    /* parameters */
    int target_type_; ///< 目标来源：1=RViz 手动点选，2=launch 预设航点。
    // 距终点小于 no_replan_thresh_ 时避免无意义重规划；
    // 轨迹执行超过 replan_thresh_ 秒后触发滚动重规划。
    double no_replan_thresh_, replan_thresh_;
    double waypoints_[50][3]; ///< launch 参数读入的航点缓存，最多 50 个，每个为 x/y/z（米）。
    int waypoint_num_;        ///< 有效预设航点数量。
    int wp_id_;               ///< 当前准备飞向的航点下标。
    // 局部规划的空间视野与时间视野。源码沿用了 horizen 的拼写。
    double planning_horizen_, planning_horizen_time_;
    double emergency_time_;          ///< 预计碰撞剩余时间小于该值时执行急停，单位秒。
    bool flag_realworld_experiment_; ///< 真机模式需要外部触发；仿真模式可自动置为已触发。
    bool enable_fail_safe_;          ///< 急停稳定后是否允许自动重新尝试规划。

    /* planning data */
    // 这些门控标志决定状态机能否离开等待状态。
    bool have_trigger_, have_target_, have_odom_, have_new_target_, have_recv_pre_agent_;
    FSM_EXEC_STATE exec_state_; ///< 当前有限状态机状态。
    int continously_called_times_{0};

    Eigen::Vector3d odom_pos_, odom_vel_, odom_acc_; // 最新里程计状态
    Eigen::Quaterniond odom_orient_; ///< 最新机体姿态四元数，当前规划主流程主要使用位置和速度。

    // end_pt_ 是最终目标；local_target_pt_ 是本次有限规划视野内的局部目标。
    // start_* 则是本次新轨迹必须连接的起始状态。
    Eigen::Vector3d init_pt_, start_pt_, start_vel_, start_acc_, start_yaw_;
    Eigen::Vector3d end_pt_, end_vel_;
    Eigen::Vector3d local_target_pt_, local_target_vel_;
    std::vector<Eigen::Vector3d> wps_; ///< 将二维数组 waypoints_ 转成 Eigen 向量后的航点序列。
    int current_wp_;                   ///< 历史遗留成员；当前航点推进主要使用 wp_id_。

    bool flag_escape_emergency_; ///< 从急停恢复后的保护标志，避免状态立即再次陷入异常分支。

    /* ROS utils */
    rclcpp::Node::SharedPtr node_; ///< main() 创建的 ROS 2 节点，所有通信资源都挂在它上面。
    rclcpp::TimerBase::SharedPtr exec_timer_;   ///< 10 ms 主状态机定时器（约 100 Hz）。
    rclcpp::TimerBase::SharedPtr safety_timer_; ///< 50 ms 轨迹安全检查定时器（约 20 Hz）。

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr waypoint_sub_; ///< RViz 手动目标。
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;              ///< 本机里程计。
    rclcpp::Subscription<traj_utils::msg::MultiBsplines>::SharedPtr swarm_trajs_sub_; ///< 前一编号无人机传来的首次启动轨迹链。
    rclcpp::Subscription<traj_utils::msg::Bspline>::SharedPtr broadcast_bspline_sub_; ///< 公共广播中的邻机最新轨迹。
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr trigger_sub_;    ///< 预设航点模式启动触发。

    // rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr replan_pub_;
    // rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr new_pub_;
    rclcpp::Publisher<traj_utils::msg::Bspline>::SharedPtr bspline_pub_; ///< 本机轨迹，供 traj_server 执行。
    rclcpp::Publisher<traj_utils::msg::DataDisp>::SharedPtr data_disp_pub_; ///< 调试统计。
    rclcpp::Publisher<traj_utils::msg::MultiBsplines>::SharedPtr swarm_trajs_pub_; ///< 首次启动轨迹链。
    rclcpp::Publisher<traj_utils::msg::Bspline>::SharedPtr broadcast_bspline_pub_; ///< 本机最新轨迹的公共广播。

    /* helper functions */
    /// 计算局部目标并调用规划管理器完成一次“初始化 + 反弹优化 + 精修”。
    bool callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj);
    /// 生成所有控制点均等于 stop_pos 的常值 B 样条。
    bool callEmergencyStop(Eigen::Vector3d stop_pos);
    /// 从全局参考轨迹开始生成局部轨迹，常用于首次规划或急停恢复。
    bool planFromGlobalTraj(const int trial_times = 1);
    /// 从旧局部轨迹的未来状态开始，保证新旧轨迹位置/速度/加速度尽量连续。
    bool planFromCurrentTraj(const int trial_times = 1);

    /* return value: std::pair< Times of the same state be continuously called, current continuously called state > */
    void changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call);
    std::pair<int, EGOReplanFSM::FSM_EXEC_STATE> timesOfConsecutiveStateCalls();
    void printFSMExecState();

    void readGivenWps();
    void planNextWaypoint(const Eigen::Vector3d next_wp);
    /// 沿全局参考轨迹向前搜索，选出规划视野边界内的局部目标。
    void getLocalTarget();

    /* ROS functions */
    /// 10 ms 周期的主状态机回调，是整个在线规划流程的实际入口。
    void execFSMCallback();
    /// 50 ms 周期的安全回调，沿当前轨迹向前采样检查地图碰撞和深度超时。
    void checkCollisionCallback();
    void waypointCallback(const std::shared_ptr<const geometry_msgs::msg::PoseStamped> &msg);
    void triggerCallback(const std::shared_ptr<const geometry_msgs::msg::PoseStamped> &msg);
    void odometryCallback(const std::shared_ptr<const nav_msgs::msg::Odometry> &msg);
    void swarmTrajsCallback(const std::shared_ptr<const traj_utils::msg::MultiBsplines> &msg);
    void BroadcastBsplineCallback(const std::shared_ptr<const traj_utils::msg::Bspline> &msg);

    bool checkCollision();
    /// 发布本机 B 样条；startup_pub=true 时还更新多机首次启动的链式消息。
    void publishSwarmTrajs(bool startup_pub);

  public:
    EGOReplanFSM(/* args */)
    {
    }
    ~EGOReplanFSM()
    {
    }

    /**
     * @brief 读取参数、构造规划模块、创建定时器并建立 ROS 话题连接。
     * @param node main() 中创建的共享 ROS 2 节点。
     */
    void init(rclcpp::Node::SharedPtr &node);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

} // namespace ego_planner

#endif
