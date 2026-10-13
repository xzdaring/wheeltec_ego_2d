#ifndef _PLANNER_MANAGER_H_
#define _PLANNER_MANAGER_H_

/**
 * @file planner_manager.h
 * @brief 全局参考轨迹、局部 B 样条、地图和优化器之间的编排接口。
 */

#include <stdlib.h>

#include <bspline_opt/bspline_optimizer.h>
#include <bspline_opt/uniform_bspline.h>
#include <traj_utils/msg/data_disp.hpp>
#include <plan_env/grid_map.h>
#include <plan_env/obj_predictor.h>
#include <traj_utils/plan_container.hpp>
#include <rclcpp/rclcpp.hpp>
#include <traj_utils/planning_visualization.h>

namespace ego_planner
{

  /**
   * @brief 单次轨迹规划的编排器。
   *
   * 状态机调用本类的公共规划接口。本类依次组织：
   * 初始轨迹生成 -> B 样条参数化 -> 障碍物约束初始化 ->
   * 控制点优化 -> 动力学可行性检查 -> 时间重分配与精修。
   *
   * 它本身不处理“何时重规划”，也不直接向控制器发布逐时刻指令。
   */
  class EGOPlannerManager
  {
    // SECTION stable
  public:
    EGOPlannerManager();
    ~EGOPlannerManager();

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /* main planning interface */
    /**
     * @brief 生成并优化一条从当前状态到局部目标的三次均匀 B 样条。
     *
     * 该接口只负责“一次局部规划”，不决定局部目标的位置，也不决定何时触发
     * 下一次重规划。FSM 在调用本函数前已经计算好 start_* 和 local_target_*。
     *
     * 原注释：flag_polyInit 为 true 时使用多项式/全局参考初始化；false 时优先继承旧轨迹。
     * 原注释：flag_randomPolyTraj 在连续失败时可扰动中间点，尝试跳出差的初值。
     * 原注释：完成避障优化和必要精修时返回 true。
     *
     * @param start_pt 本次轨迹的起始位置，世界坐标系，单位 m。首次规划通常来自
     *        里程计；滚动重规划通常来自旧轨迹上当前时刻的状态。
     * @param start_vel 起始速度，世界坐标系，单位 m/s，用作 B 样条起端一阶导约束。
     * @param start_acc 起始加速度，世界坐标系，单位 m/s^2，用作起端二阶导约束。
     * @param local_target_pt 本次有限视野轨迹的末端位置，世界坐标系，单位 m。
     *        它通常距 start_pt 约一个 planning_horizon，并不一定是全局终点。
     * @param local_target_vel 希望到达局部目标时具有的速度，单位 m/s。距离最终终点
     *        较远时通常沿全局参考轨迹前进；临近最终终点时通常设为零。
     * @param flag_polyInit true 时放弃继承旧局部轨迹，使用连接起点和局部目标的
     *        多项式作为离散初值；false 时优先复用旧轨迹未执行完的部分。
     * @param flag_randomPolyTraj 仅在多项式初始化分支生效。true 时在起终点之间
     *        加入随机扰动的中间点，用于连续失败后尝试改变绕障方向。
     * @return 初始化、避障优化和必要的动力学精修均成功时返回 true；返回 false 时
     *         local_data_ 保持原轨迹，不会被本次失败结果覆盖。
     */
    bool reboundReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel, Eigen::Vector3d start_acc,
                       Eigen::Vector3d end_pt, Eigen::Vector3d end_vel, bool flag_polyInit, bool flag_randomPolyTraj);
    /// 用重复控制点构造位置恒定、导数为零的紧急悬停轨迹。
    bool EmergencyStop(Eigen::Vector3d stop_pos);
    /// 构造从起点到终点的全局多项式参考轨迹。
    bool planGlobalTraj(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                        const Eigen::Vector3d &end_pos, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc);
    /// 构造依次经过多个航点的全局多项式参考轨迹。
    bool planGlobalTrajWaypoints(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                 const std::vector<Eigen::Vector3d> &waypoints, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc);

    /// 读取参数并创建 GridMap、BsplineOptimizer 和 AStar。
    void initPlanModules(rclcpp::Node::SharedPtr &node, PlanningVisualization::Ptr vis = NULL);

    // 优化器只保存该缓冲区的非拥有指针；manager 生命周期内缓冲区一直有效。
    void deliverTrajToOptimizer(void) { bspline_optimizer_->setSwarmTrajs(&swarm_trajs_buf_); };

    void setDroneIdtoOpt(void) { bspline_optimizer_->setDroneId(pp_.drone_id); }

    /// 返回优化器使用的多机安全距离，单位米。
    double getSwarmClearance(void) { return bspline_optimizer_->getSwarmClearance(); }

    /// 按绝对时间对齐本机与指定邻机轨迹，检查两机间距是否小于安全距离。
    bool checkCollision(int drone_id);
    

    PlanParameters pp_;           ///< 速度、加速度、规划视野和无人机编号等配置。
    LocalTrajData local_data_;    ///< 当前真正执行的局部 B 样条及其导数。
    GlobalTrajData global_data_;  ///< 通往最终目标的全局参考轨迹。
    GridMap::Ptr grid_map_; ///< 局部膨胀占据地图，供安全检查、A* 和优化器查询。
    fast_planner::ObjPredictor::Ptr obj_predictor_; ///< 动态物体预测器接口；当前主优化组合中动态代价被注释。
    SwarmTrajData swarm_trajs_buf_; ///< 按 drone_id 索引的其他无人机时空轨迹。

  private:
    /* main planning algorithms & modules */
    PlanningVisualization::Ptr visualization_; ///< RViz Marker 发布器，不影响规划数值结果。

    // ros::Publisher obj_pub_; //zx-todo 

    BsplineOptimizer::Ptr bspline_optimizer_; ///< 控制点优化、静态/多机避碰和可行性代价的实现者。

    int continous_failures_count_{0}; ///< 连续规划失败次数，用于增大随机初值扰动。

    /// 保存新位置轨迹，并自动生成速度/加速度 B 样条和递增轨迹编号。
    void updateTrajInfo(const UniformBspline &position_traj, const rclcpp::Time time_now);

    /// 按 ratio 拉长轨迹时间并重新采样、重新参数化为 B 样条。
    void reparamBspline(UniformBspline &bspline, vector<Eigen::Vector3d> &start_end_derivative, double ratio, Eigen::MatrixXd &ctrl_pts, double &dt,
                        double &time_inc);

    /// 时间伸缩后再次优化，使轨迹满足物理限制且尽量贴近原曲线。
    bool refineTrajAlgo(UniformBspline &traj, vector<Eigen::Vector3d> &start_end_derivative, double ratio, double &ts, Eigen::MatrixXd &optimal_control_points);

    // !SECTION stable

    // SECTION developing

  public:
    typedef unique_ptr<EGOPlannerManager> Ptr;

    // !SECTION
  };
} // namespace ego_planner

#endif
