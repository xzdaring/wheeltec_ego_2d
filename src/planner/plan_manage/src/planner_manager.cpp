// #include <fstream>
#include <ego_planner/planner_manager.h>
#include <thread>
#include "visualization_msgs/msg/marker.hpp" // zx-todo

namespace ego_planner
{
  constexpr double kGroundZ = 0.0;
  
  EGOPlannerManager::EGOPlannerManager() {}

  EGOPlannerManager::~EGOPlannerManager() {}

  void EGOPlannerManager::initPlanModules(rclcpp::Node::SharedPtr &node, PlanningVisualization::Ptr vis)
  {
    // manager 统一持有一次规划所需的模块。GridMap 从传感器回调持续更新；
    // BsplineOptimizer 在每次重规划时读取该地图和 swarm_trajs_buf_。
    node->declare_parameter("manager/max_vel", -1.0);                 // 各轴允许最大速度，m/s。
    node->declare_parameter("manager/max_acc", -1.0);                 // 各轴允许最大加速度，m/s²。
    node->declare_parameter("manager/max_jerk", -1.0);                // 期望 jerk 上限，m/s³；当前主要用于参数保存。
    node->declare_parameter("manager/feasibility_tolerance", 0.0);    // 可行性检查允许的相对超限余量。
    node->declare_parameter("manager/control_points_distance", -1.0); // 相邻控制点期望空间间距，米。
    node->declare_parameter("manager/planning_horizon", 5.0);         // 每次局部规划的空间视野，米。
    node->declare_parameter("manager/use_distinctive_trajs", false);  // 是否生成障碍物不同侧的多拓扑候选。
    node->declare_parameter("manager/drone_id", -1);                  // 多机从 0 编号；设计上负数表示单机。

    node->get_parameter("manager/max_vel", pp_.max_vel_);
    node->get_parameter("manager/max_acc", pp_.max_acc_);
    node->get_parameter("manager/max_jerk", pp_.max_jerk_);
    node->get_parameter("manager/feasibility_tolerance", pp_.feasibility_tolerance_);
    node->get_parameter("manager/control_points_distance", pp_.ctrl_pt_dist);
    node->get_parameter("manager/planning_horizon", pp_.planning_horizen_);
    node->get_parameter("manager/use_distinctive_trajs", pp_.use_distinctive_trajs);
    node->get_parameter("manager/drone_id", pp_.drone_id);

    // 每发布一条新局部轨迹，traj_id_ 在 updateTrajInfo() 中递增。
    local_data_.traj_id_ = 0;
    grid_map_.reset(new GridMap);
    // grid_map_->initMap(nh);
    grid_map_->initMap(node);

    bspline_optimizer_.reset(new BsplineOptimizer);
    // bspline_optimizer_->setParam(nh);
    bspline_optimizer_->setParam(node);
    bspline_optimizer_->setEnvironment(grid_map_, obj_predictor_);
    bspline_optimizer_->a_star_.reset(new AStar);
    // A* 预分配 100³ 个搜索节点。它的搜索步长由每次 AstarSearch() 传入，
    // 这里的 100 表示索引池尺寸而不是 100 米。
    bspline_optimizer_->a_star_->initGridMap(grid_map_, Eigen::Vector3i(100, 100, 100));

    visualization_ = vis;
  }

  bool EGOPlannerManager::reboundReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel,
                                        Eigen::Vector3d start_acc, Eigen::Vector3d local_target_pt,
                                        Eigen::Vector3d local_target_vel, bool flag_polyInit, bool flag_randomPolyTraj)
  {
    /*
     * 参数和单位：
     *   start_pt              当前局部轨迹的起点，[m]，世界坐标系。
     *   start_vel             起点速度，[m/s]，同时作为轨迹起端一阶导约束。
     *   start_acc             起点加速度，[m/s^2]，同时作为轨迹起端二阶导约束。
     *   local_target_pt       本次局部规划终点，[m]；通常不是整次任务的最终目标。
     *   local_target_vel      局部终点期望速度，[m/s]；临近最终目标时一般为零。
     *   flag_polyInit         true：用多项式新建初值；false：优先继承旧轨迹。
     *   flag_randomPolyTraj   true：多项式初值经过随机中间点，尝试改变绕障侧。
     *
     * 需要特别区分三组“点”：
     *   point_set  是 B 样条参数化之前的离散路径样本；
     *   ctrl_pts   是由 point_set 解算得到、随后交给优化器移动的 B 样条控制点；
     *   最终轨迹  是 ctrl_pts 定义的连续 B 样条，并非逐点连接 point_set。
     */

    // 完整流水线：
    // 1) 构造离散初值和边界导数；
    // 2) 参数化为三次均匀 B 样条控制点；
    // 3) 用栅格/A* 初始化障碍物推出方向；
    // 4) LBFGS 优化平滑、避障、动力学和多机代价；
    // 5) 若速度/加速度超限，则拉长时间并进行贴合精修；
    // 6) 保存可发布的局部轨迹及其导数。
    // 仅用于终端日志编号；static 使编号在本进程的多次函数调用之间持续递增。

    // 起点和局部目标保持同一高度，规划只考虑平面运动。
    start_pt.z() = kGroundZ;
    local_target_pt.z() = kGroundZ;

    // 固定高度的轨迹不能有竖直速度、加速度，
    // 否则多项式初始化仍可能生成上下运动。
    start_vel.z() = 0.0;
    start_acc.z() = 0.0;
    local_target_vel.z() = 0.0;

    static int count = 0;
    printf("\033[47;30m\n[drone %d replan %d]==============================================\033[0m\n", pp_.drone_id, count++);

    // 起终点距离过小时，数值上难以构造有意义且至少含 7 个样本的局部轨迹。
    // 这里把它当作本轮规划失败，由上层 FSM 决定继续执行旧轨迹或再次尝试。
    if ((start_pt - local_target_pt).norm() < 0.2)
    {
      cout << "Close to goal" << endl;
      continous_failures_count_++;
      return false;
    }

    // 优化器的终点吸引/终端代价会使用该点。这里只保存目标，不会立即开始优化。
    bspline_optimizer_->setLocalTargetPt(local_target_pt);

    // t_start 是当前流水线阶段的计时起点；三个 Duration 分别统计初始化、
    // 控制点优化和动力学精修耗时，只用于性能输出，不参与轨迹计算。
    rclcpp::Time t_start = rclcpp::Clock().now();
    rclcpp::Duration t_init(0, 0), t_opt(0, 0), t_refine(0, 0);

    /*** STEP 1: INIT
    根据起始点和目标点的距离计算首个时间步长ts,向量的模大于0.1则用1.5倍否则用5倍
    ***/
    /************************ STEP 1：构造离散初值 ************************/
    // ts 是相邻均匀 B 样条节点的时间间隔，[s]，不是空间分辨率。
    // 对一般距离：ts = ctrl_pt_dist / max_vel * 1.5。默认参数 0.4 m 和
    // 2.0 m/s 得到 0.3 s，因此匀速 2.0 m/s 区段的 point_set 间距约 0.6 m。
    // 若距离极小则使用更大的倍率，使时间参数不至于过紧；不过当前函数会先把
    // 小于 0.2 m 的情况返回，因此实际运行基本进入前一分支。
    double ts = (start_pt - local_target_pt).norm() > 0.1 ? pp_.ctrl_pt_dist / pp_.max_vel_ * 1.5 : pp_.ctrl_pt_dist / pp_.max_vel_ * 5; // pp_.ctrl_pt_dist / pp_.max_vel_ is too tense, and will surely exceed the acc/vel limits

    // point_set：参数化前的离散路径样本，单位 m；它不是 B 样条控制点。
    // start_end_derivatives 的固定顺序为：起点速度、终点速度、起点加速度、
    // 终点加速度。parameterizeToBspline() 用它们约束曲线两端导数。
    vector<Eigen::Vector3d> point_set, start_end_derivatives;

    // flag_first_call：本进程第一次调用没有可继承的旧轨迹，必须使用多项式。
    // flag_force_polynomial：继承旧轨迹产生异常长点集时，在同一次调用的下一轮
    // do-while 中强制改用多项式。两者是 static，状态跨函数调用保留。
    static bool flag_first_call = true, flag_force_polynomial = false;

    // true 表示当前继承初值不合理，需要重新进入 do-while 构造一次多项式初值。
    bool flag_regenerate = false;
    do
    {
      point_set.clear();
      start_end_derivatives.clear();
      flag_regenerate = false;

      // 三种情况进入多项式分支：进程首次规划、调用者明确要求，或继承旧轨迹
      // 得到的点集异常长。正常滚动重规划则进入下面的旧轨迹继承分支。
      // 原注释：这里如果正常进入if（通常为初次生成），则do部分只进行一次，即只清空一次点集；若进入else则有可能对异常情况重置flag_regenerate并再do一次
      if (flag_first_call || flag_polyInit || flag_force_polynomial /*|| ( start_pt - local_target_pt ).norm() < 1.0*/) // Initial path generated from a min-snap traj by order.
      {
        flag_first_call = false;
        flag_force_polynomial = false;
        // 用于存储生成的轨迹
        // gl_traj 是只为生成 point_set 而构造的连续多项式初值，最终不会直接发布。
        PolynomialTraj gl_traj;

        // 按“加速—可能匀速—减速”的三角形/梯形速度模型估算飞行时间。
        // dist 是本次局部规划起终点直线距离，[m]。
        double dist = (start_pt - local_target_pt).norm();

        // 判断 速度的平方/加速度 是否大于dist，并决定如何计算时间
        // time 是多项式初值的启发式总时长，[s]。短距离采用与 sqrt(dist/acc)
        // 同量级的时长；长距离加入按 max_vel 匀速飞行的时间。它只是初始时间
        // 分配，后续若速度/加速度超限还会重新拉长。
        double time = pow(pp_.max_vel_, 2) / pp_.max_acc_ > dist ? sqrt(dist / pp_.max_acc_) : (dist - pow(pp_.max_vel_, 2) / pp_.max_acc_) / pp_.max_vel_ + 2 * pp_.max_vel_ / pp_.max_acc_;

        if (!flag_randomPolyTraj)
        {
          // false生成一段单一的多项式轨迹，true生成一个包含随机插入点的轨迹
          // 普通情况：直接用一段多项式连接完整起始状态和局部目标状态。
          // 末端加速度固定为零，使到达目标时的状态更平稳。
          gl_traj = PolynomialTraj::one_segment_traj_gen(start_pt, start_vel, start_acc, local_target_pt, local_target_vel, Eigen::Vector3d::Zero(), time);
        }
        else
        {
          // 连续失败时，在起终点连线的水平法向和竖直法向上随机扰动中点。
          // 失败次数越多，扰动系数逐渐变大，以尝试不同绕障侧。
          // horizen_dir：与起终点连线垂直的水平方向单位向量；
          // vertical_dir：再与前两者正交的方向，用于产生三维随机偏移。
          Eigen::Vector3d horizen_dir = ((start_pt - local_target_pt).cross(Eigen::Vector3d(0, 0, 1))).normalized();

          // random_inserted_pt 以起终点中点为中心随机偏移。偏移尺度随起终点距离
          // 和连续失败次数增大，从而逐渐尝试更明显的绕行初值。
          Eigen::Vector3d random_inserted_pt =
              (start_pt + local_target_pt) / 2 +
              (((double)rand()) / RAND_MAX - 0.5) *
              (start_pt - local_target_pt).norm() *
              horizen_dir * 0.8 *
              (-0.978 / (continous_failures_count_ + 0.989) + 0.989);

          random_inserted_pt.z() = kGroundZ;
          // pos 的三列依次为起点、随机中间点、局部目标。

          // 随机中间点也必须位于固定规划平面。
          random_inserted_pt.z() = kGroundZ;
          Eigen::MatrixXd pos(3, 3);
          pos.col(0) = start_pt;
          pos.col(1) = random_inserted_pt;
          pos.col(2) = local_target_pt;
          // 两段多项式各分配一半总时长。
          Eigen::VectorXd t(2);
          t(0) = t(1) = time / 2;
          gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, local_target_vel, start_acc, Eigen::Vector3d::Zero(), t);
        }

        // t：沿多项式采样的时间，[s]；循环结束后用于取得最后一个有效样本状态。
        // flag_too_far：只要存在相邻点距大于 1.5*ctrl_pt_dist，就需要加密采样。
        double t;
        bool flag_too_far;

        // 预乘 1.5 是为了抵消下面第一次循环开头的除法，使第一次尝试仍使用
        // 前面计算的原始 ts；后续失败才依次变为 ts/1.5、ts/1.5^2……。
        ts *= 1.5; // ts will be divided by 1.5 in the next
        // 自适应减小 ts，直到相邻采样点不超过 1.5 倍控制点间距，且至少
        // 得到 7 个点，保证三次 B 样条参数化有足够控制点。
        do
        {
          ts /= 1.5;
          point_set.clear();
          flag_too_far = false;
          // last_pt 用来检查相邻离散样本的实际空间距离，而非检查控制点距离。
          Eigen::Vector3d last_pt = gl_traj.evaluate(0);
          for (t = 0; t < time; t += ts)
          {
            // pt 是时刻 t 的多项式位置样本，[m]。
            Eigen::Vector3d pt = gl_traj.evaluate(t);
            if ((last_pt - pt).norm() > pp_.ctrl_pt_dist * 1.5)
            {
              flag_too_far = true;
              break;
            }
            last_pt = pt;
            point_set.push_back(pt);
          }
        } while (flag_too_far || point_set.size() < 7); // To make sure the initial path has enough points.
        // for 正常结束时 t 已经越过最后一个有效采样时刻，因此退回一个 ts。
        t -= ts;

        // 记录参数化所需的四个边界导数。末端加速度取多项式最后采样时刻
        // 的实际值，而不是强行设为零。
        start_end_derivatives.push_back(gl_traj.evaluateVel(0));
        start_end_derivatives.push_back(local_target_vel);
        start_end_derivatives.push_back(gl_traj.evaluateAcc(0));
        start_end_derivatives.push_back(gl_traj.evaluateAcc(t));
      }
      else // Initial path generated from previous trajectory.
      {
        // 滚动重规划：采样旧轨迹尚未执行的部分，再用一段多项式连接到
        // 新局部目标，最后按近似弧长重新均匀采样。
        // t 是旧轨迹/连接多项式的采样时间；t_cur 是旧轨迹已经执行的时间，[s]。
        double t;
        double t_cur = (rclcpp::Clock().now() - local_data_.start_time_).seconds();

        // segment_point 保存“旧轨迹剩余部分 + 通往新局部目标的连接段”的稠密样本。
        // pseudo_arc_length[i] 是 segment_point[0] 到 segment_point[i] 的累计折线
        // 长度，[m]。后面利用它把按时间采样的数据重新变成近似等弧长样本。
        vector<double> pseudo_arc_length;
        vector<Eigen::Vector3d> segment_point;
        pseudo_arc_length.push_back(0.0);
        for (t = t_cur; t < local_data_.duration_ + 1e-3; t += ts)
        {
          segment_point.push_back(local_data_.position_traj_.evaluateDeBoorT(t));
          if (t > t_cur)
          {
            pseudo_arc_length.push_back((segment_point.back() - segment_point[segment_point.size() - 2]).norm() + pseudo_arc_length.back());
          }
        }
        t -= ts;

        // poly_time 是从旧轨迹末端连接到新局部目标的预估时长，[s]。距离除以
        // max_vel 后再乘 2，为连接段留出比匀速直飞更宽松的时间。
        double poly_time = (local_data_.position_traj_.evaluateDeBoorT(t) - local_target_pt).norm() / pp_.max_vel_ * 2;
        if (poly_time > ts)
        {
          PolynomialTraj gl_traj = PolynomialTraj::one_segment_traj_gen(local_data_.position_traj_.evaluateDeBoorT(t),
                                                                        local_data_.velocity_traj_.evaluateDeBoorT(t),
                                                                        local_data_.acceleration_traj_.evaluateDeBoorT(t),
                                                                        local_target_pt, local_target_vel, Eigen::Vector3d::Zero(), poly_time);

          for (t = ts; t < poly_time; t += ts)
          {
            if (!pseudo_arc_length.empty())
            {
              segment_point.push_back(gl_traj.evaluate(t));
              pseudo_arc_length.push_back((segment_point.back() - segment_point[segment_point.size() - 2]).norm() + pseudo_arc_length.back());
            }
            else
            {
              RCLCPP_ERROR(rclcpp::get_logger("ego_planner"), "pseudo_arc_length is empty, return!");
              continous_failures_count_++;
              return false;
            }
          }
        }

        // sample_length 是下一个 point_set 样本在累计弧长轴上的位置，[m]；
        // cps_dist 是期望弧长间隔，[m]；id 指向 sample_length 所在的折线段。
        // 默认 ctrl_pt_dist=0.4 m 时，第一次循环的 cps_dist 正好为 0.4 m。
        double sample_length = 0;
        double cps_dist = pp_.ctrl_pt_dist * 1.5; // cps_dist will be divided by 1.5 in the next
        size_t id = 0;
        do
        {
          cps_dist /= 1.5;
          point_set.clear();
          sample_length = 0;
          id = 0;
          while ((id <= pseudo_arc_length.size() - 2) && sample_length <= pseudo_arc_length.back())
          {
            if (sample_length >= pseudo_arc_length[id] && sample_length < pseudo_arc_length[id + 1])
            {
              // 在累计弧长包围 sample_length 的两个稠密样本之间做线性插值。
              point_set.push_back((sample_length - pseudo_arc_length[id]) / (pseudo_arc_length[id + 1] - pseudo_arc_length[id]) * segment_point[id + 1] +
                                  (pseudo_arc_length[id + 1] - sample_length) / (pseudo_arc_length[id + 1] - pseudo_arc_length[id]) * segment_point[id]);
              sample_length += cps_dist;
            }
            else
              id++;
          }
          // 无论等弧长采样是否恰好落到终点，都显式加入准确的局部目标。
          point_set.push_back(local_target_pt);
        } while (point_set.size() < 7); // If the start point is very close to end point, this will help

        // 滚动重规划的起端导数取旧轨迹当前时刻，保证切换时速度、加速度连续；
        // 新局部目标使用调用者给定速度，末端加速度设为零。
        start_end_derivatives.push_back(local_data_.velocity_traj_.evaluateDeBoorT(t_cur));
        start_end_derivatives.push_back(local_target_vel);
        start_end_derivatives.push_back(local_data_.acceleration_traj_.evaluateDeBoorT(t_cur));
        start_end_derivatives.push_back(Eigen::Vector3d::Zero());

        // 合理点数的量级约为 planning_horizon/ctrl_pt_dist。这里允许到其 3 倍；
        // 超过说明旧轨迹拼接/弧长采样异常，改用全新多项式，而非继续优化巨量点。
        if (point_set.size() > pp_.planning_horizen_ / pp_.ctrl_pt_dist * 3) // The initial path is unnormally too long!
        {
          // 点数远超“视野/控制点间距”的合理量级，说明旧轨迹拼接异常；
          // 下一轮 do-while 放弃继承，改为全新多项式初值。
          flag_force_polynomial = true;
          flag_regenerate = true;
        }
      }
    } while (flag_regenerate);

    // 将 point_set 与起终端导数约束共同参数化为 B 样条控制点。
    // ctrl_pts 是当前采用/最终选中的 3×N 控制点矩阵，每列为一个三维位置；
    // ctrl_pts_temp 是多拓扑模式下单个候选优化结果的临时输出缓冲区。
    Eigen::MatrixXd ctrl_pts, ctrl_pts_temp;
    UniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, ctrl_pts);

    // 找出初始控制多边形中穿越障碍物的连续区段。initControlPoints()
    // 会借助 A* 为这些区段建立 base_point/direction 几何约束。
    // segments 中每对索引表示一段与障碍物冲突的连续控制点区间。
    vector<std::pair<int, int>> segments;
    segments = bspline_optimizer_->initControlPoints(ctrl_pts, true);
    // 计算时间差并更新时间
    // 截止此处的耗时归入初始化阶段，然后重置秒表供优化阶段使用。
    auto now = rclcpp::Clock().now();
    t_init = now - t_start;
    t_start = now;

    /*** STEP 2: OPTIMIZE ***/
    // flag_step_1_success 表示至少有一条候选通过 rebound 优化。
    // vis_trajs 只收集候选控制多边形用于 RViz 展示，不参与候选评分。
    bool flag_step_1_success = false;
    vector<vector<Eigen::Vector3d>> vis_trajs;

    if (pp_.use_distinctive_trajs)
    {
      // 对每个碰撞区段尝试障碍物不同侧的拓扑候选，逐条优化并保留
      // 成功候选中代价最低的一条。这会增加计算量，但能减轻局部最优问题。
      // cout << "enter" << endl;
      std::vector<ControlPoints> trajs = bspline_optimizer_->distinctiveTrajs(segments);
      cout << "\033[1;33m"
           << "multi-trajs=" << trajs.size() << "\033[1;0m" << endl;

      // final_cost 是当前候选的优化目标值；min_cost 记录成功候选中的最小值。
      double final_cost, min_cost = 999999.0;
      for (int i = trajs.size() - 1; i >= 0; i--)
      {
        if (bspline_optimizer_->BsplineOptimizeTrajRebound(ctrl_pts_temp, final_cost, trajs[i], ts))
        {

          cout << "traj " << trajs.size() - i << " success." << endl;

          flag_step_1_success = true;
          if (final_cost < min_cost)
          {
            min_cost = final_cost;
            ctrl_pts = ctrl_pts_temp;
          }

          // visualization
          // 将当前成功候选的控制点复制成 vector，仅用于可视化。
          point_set.clear();
          for (int j = 0; j < ctrl_pts_temp.cols(); j++)
          {
            point_set.push_back(ctrl_pts_temp.col(j));
          }
          vis_trajs.push_back(point_set);
        }
        else
        {
          cout << "traj " << trajs.size() - i << " failed." << endl;
        }
      }

      t_opt = rclcpp::Clock().now() - t_start;

      visualization_->displayMultiInitPathList(vis_trajs, 0.2);
    }
    else
    {
      // 普通模式只从当前这一组控制点开始一次 rebound 优化。
      flag_step_1_success = bspline_optimizer_->BsplineOptimizeTrajRebound(ctrl_pts, ts);
      t_opt = rclcpp::Clock().now() - t_start;
      // static int vis_id = 0;
      visualization_->displayInitPathList(point_set, 0.2, 0);
    }

    cout << "plan_success=" << flag_step_1_success << endl;
    if (!flag_step_1_success)
    {
      visualization_->displayOptimalList(ctrl_pts, 0);
      continous_failures_count_++;
      return false;
    }

    t_start = rclcpp::Clock().now();

    // 3 表示三次 B 样条，它通常具有二阶连续性，适合生成位置及其导数。
    // pos 是优化后的连续位置轨迹；ts 为均匀节点间隔。
    // setPhysicalLimits() 只设置后续检查所用上限，本身不会立刻修改曲线。
    UniformBspline pos = UniformBspline(ctrl_pts, 3, ts);
    pos.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_, pp_.feasibility_tolerance_);

    /*** STEP 3: REFINE(RE-ALLOCATE TIME) IF NECESSARY ***/
    // 空间无碰撞不代表动力学可执行。若控制点差分对应的速度/加速度超限，
    // 先增大节点时间间隔（同一路径走慢一些），再优化以贴近原曲线。
    // Note: Only adjust time in single drone mode. But we still allow drone_0 to adjust its time profile.
    if (pp_.drone_id <= 0)
    {

      // ratio > 1 时表示为了满足速度/加速度限制，时间轴至少需要放大的比例。
      double ratio = 1.0;  // 可行性检查通过引用返回所需的时间缩放比例。
      bool flag_step_2_success = true;
      if (!pos.checkFeasibility(ratio, false))
      {
        cout << "Need to reallocate time." << endl;

        // optimal_control_points 保存时间重分配后、以原轨迹形状为参考再次优化的结果。
        Eigen::MatrixXd optimal_control_points;
        flag_step_2_success = refineTrajAlgo(pos, start_end_derivatives, ratio, ts, optimal_control_points);
        if (flag_step_2_success)
          pos = UniformBspline(optimal_control_points, 3, ts);
      }

      if (!flag_step_2_success)
      {
        printf("\033[34mThis refined trajectory hits obstacles. It doesn't matter if appeares occasionally. But if continously appearing, Increase parameter \"lambda_fitness\".\n\033[0m");
        continous_failures_count_++;
        return false;
      }
    }
    else
    {
      static bool print_once = true;
      if (print_once)
      {
        print_once = false;
        RCLCPP_ERROR(rclcpp::get_logger("ego_planner"), "IN SWARM MODE, REFINE DISABLED!");
      }
    }

    // t_refine = ros::Time::now() - t_start;
    t_refine = rclcpp::Clock().now() - t_start;

    // save planned results
    // 只有初始化、优化以及必要精修全部成功，才替换当前执行轨迹。
    updateTrajInfo(pos, rclcpp::Clock().now());

    // 以下静态变量只累计成功规划的总耗时和次数，用于输出平均耗时。
    static double sum_time = 0;
    static int count_success = 0;

    sum_time += (t_init + t_opt + t_refine).seconds();

    count_success++;

    // cout << "total time:\033[42m" << (t_init + t_opt + t_refine).toSec() << "\033[0m,optimize:" << (t_init + t_opt).toSec() << ",refine:" << t_refine.toSec() << ",avg_time=" << sum_time / count_success << endl;
    cout << "total time:\033[42m" << (t_init + t_opt + t_refine).seconds() << "\033[0m,optimize:" << (t_init + t_opt).seconds() << ",refine:" << t_refine.seconds() << ",avg_time=" << sum_time / count_success << endl;

    // success. YoY
    continous_failures_count_ = 0;
    return true;
  }

  bool EGOPlannerManager::EmergencyStop(Eigen::Vector3d stop_pos)
  {
    // 三次 B 样条至少需要若干控制点。六个相同控制点定义位置恒定曲线，
    // 对时间求导后速度和加速度自然为零。
    Eigen::MatrixXd control_points(3, 6);
    for (int i = 0; i < 6; i++)
    {
      control_points.col(i) = stop_pos;
    }

    updateTrajInfo(UniformBspline(control_points, 3, 1.0), rclcpp::Clock().now());

    return true;
  }

  bool EGOPlannerManager::checkCollision(int drone_id)
  {
    // 按绝对时间对齐本机和指定邻机轨迹，每 0.03 s 采样一次。
    // 只有两条轨迹有效时间的交集需要检查；几何路径相交但通过时间不同
    // 不一定构成多机碰撞。
    // if (local_data_.start_time_.toSec() < 1e9) // It means my first planning has not started
    if (local_data_.start_time_.seconds() < 1e9)
      return false;

    // double my_traj_start_time = local_data_.start_time_.toSec();
    // double other_traj_start_time = swarm_trajs_buf_[drone_id].start_time_.toSec();
    double my_traj_start_time = local_data_.start_time_.seconds();
    double other_traj_start_time = swarm_trajs_buf_[drone_id].start_time_.seconds();

    double t_start = max(my_traj_start_time, other_traj_start_time);
    double t_end = min(my_traj_start_time + local_data_.duration_ * 2 / 3, other_traj_start_time + swarm_trajs_buf_[drone_id].duration_);

    for (double t = t_start; t < t_end; t += 0.03)
    {
      if ((local_data_.position_traj_.evaluateDeBoorT(t - my_traj_start_time) - swarm_trajs_buf_[drone_id].position_traj_.evaluateDeBoorT(t - other_traj_start_time)).norm() < bspline_optimizer_->getSwarmClearance())
      {
        return true;
      }
    }

    return false;
  }

  bool EGOPlannerManager::planGlobalTrajWaypoints(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                                  const std::vector<Eigen::Vector3d> &waypoints, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
  {
    Eigen::Vector3d planar_start = start_pos;
    Eigen::Vector3d planar_start_vel = start_vel;
    Eigen::Vector3d planar_start_acc = start_acc;
    Eigen::Vector3d planar_end_vel = end_vel;
    Eigen::Vector3d planar_end_acc = end_acc;

    planar_start.z() = kGroundZ;
    planar_start_vel.z() = 0.0;
    planar_start_acc.z() = 0.0;
    planar_end_vel.z() = 0.0;
    planar_end_acc.z() = 0.0;

    // 航点也要全部投影到同一个规划平面。
    std::vector<Eigen::Vector3d> planar_waypoints = waypoints;
    if (planar_waypoints.empty())
        return false;

    for (auto &point : planar_waypoints)
        point.z() = kGroundZ;

    // 以下是原有代码……
    // 生成经过多个航点的全局多项式参考；它提供总体进度，不直接承担避障。

    vector<Eigen::Vector3d> points;
    points.push_back(planar_start);

    for (const auto &point : planar_waypoints)
        points.push_back(point);

    double total_len = (planar_start - planar_waypoints.front()).norm();
    for (size_t i = 1; i < planar_waypoints.size(); ++i)
    {
        total_len +=
            (planar_waypoints[i] - planar_waypoints[i - 1]).norm();
    }

    // insert intermediate points if too far
    vector<Eigen::Vector3d> inter_points;
    // 长段补点，避免最小 snap 多项式跨越过长距离。阈值至少为 4 m，
    // 同时让整条路径大约分成不超过八个尺度相近的区段。
    double dist_thresh = max(total_len / 8, 4.0);

    for (size_t i = 0; i < points.size() - 1; ++i)
    {
      inter_points.push_back(points.at(i));
      double dist = (points.at(i + 1) - points.at(i)).norm();

      if (dist > dist_thresh)
      {
        int id_num = floor(dist / dist_thresh) + 1;

        for (int j = 1; j < id_num; ++j)
        {
          Eigen::Vector3d inter_pt =
              points.at(i) * (1.0 - double(j) / id_num) + points.at(i + 1) * double(j) / id_num;
          inter_points.push_back(inter_pt);
        }
      }
    }

    inter_points.push_back(points.back());

    int pt_num = inter_points.size();
    Eigen::MatrixXd pos(3, pt_num);
    for (int i = 0; i < pt_num; ++i)
      pos.col(i) = inter_points[i];

    Eigen::Vector3d zero(0, 0, 0);
    Eigen::VectorXd time(pt_num - 1);
    for (int i = 0; i < pt_num - 1; ++i)
    {
      // 以“距离/最大速度”估算段时间；首尾乘 2，为起步和刹停留余量。
      time(i) = (pos.col(i + 1) - pos.col(i)).norm() / (pp_.max_vel_);
    }

    time(0) *= 2.0;
    time(time.rows() - 1) *= 2.0;

    PolynomialTraj gl_traj;
    if (pos.cols() >= 3)
    {
        gl_traj = PolynomialTraj::minSnapTraj(
            pos, planar_start_vel, planar_end_vel,
            planar_start_acc, planar_end_acc, time);
    }
    else if (pos.cols() == 2)
    {
        gl_traj = PolynomialTraj::one_segment_traj_gen(
            planar_start, planar_start_vel, planar_start_acc,
            pos.col(1), planar_end_vel, planar_end_acc, time(0));
    }
    else
    {
        return false;
    }

    // 保存生成的全局参考轨迹，供状态机提取局部目标和推进航点。
    global_data_.setGlobalTraj(gl_traj, rclcpp::Clock().now());
    return true;
  }

  bool EGOPlannerManager::planGlobalTraj(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                         const Eigen::Vector3d &end_pos, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
  {

        // 保留输入不变，使用副本构造平面全局参考轨迹。
    Eigen::Vector3d planar_start = start_pos;
    Eigen::Vector3d planar_end = end_pos;
    Eigen::Vector3d planar_start_vel = start_vel;
    Eigen::Vector3d planar_start_acc = start_acc;
    Eigen::Vector3d planar_end_vel = end_vel;
    Eigen::Vector3d planar_end_acc = end_acc;

    // 固定位置高度，并清除会引起上下运动的边界导数。
    planar_start.z() = kGroundZ;
    planar_end.z() = kGroundZ;
    planar_start_vel.z() = 0.0;
    planar_start_acc.z() = 0.0;
    planar_end_vel.z() = 0.0;
    planar_end_acc.z() = 0.0;  
    
    // 单终点版本：起终点距离超过 4 m 时沿直线补中间点，再生成多项式。

    vector<Eigen::Vector3d> points;
    points.push_back(planar_start);
    points.push_back(planar_end);

    // insert intermediate points if too far
    vector<Eigen::Vector3d> inter_points;
    const double dist_thresh = 4.0;

    for (size_t i = 0; i < points.size() - 1; ++i)
    /*挨个读取点并计算点距判断是否需要插点，随后计算插点并写入矩阵，最后根据插点数量生成全局轨迹
      最终返回值为是否规划成功的布尔值 */
    {
      inter_points.push_back(points.at(i));
      double dist = (points.at(i + 1) - points.at(i)).norm();

      if (dist > dist_thresh)
      {
        int id_num = floor(dist / dist_thresh) + 1;

        for (int j = 1; j < id_num; ++j)
        {
          Eigen::Vector3d inter_pt =
              points.at(i) * (1.0 - double(j) / id_num) + points.at(i + 1) * double(j) / id_num;
          inter_points.push_back(inter_pt);
        }
      }
    }

    inter_points.push_back(points.back());

    // write position matrix
    int pt_num = inter_points.size();
    Eigen::MatrixXd pos(3, pt_num);
    for (int i = 0; i < pt_num; ++i)
      pos.col(i) = inter_points[i];

    Eigen::Vector3d zero(0, 0, 0);
    Eigen::VectorXd time(pt_num - 1);
    for (int i = 0; i < pt_num - 1; ++i)
    {
      time(i) = (pos.col(i + 1) - pos.col(i)).norm() / (pp_.max_vel_);
    }

    time(0) *= 2.0;
    time(time.rows() - 1) *= 2.0;

    PolynomialTraj gl_traj;
    if (pos.cols() >= 3)
    {
        gl_traj = PolynomialTraj::minSnapTraj(
            pos, planar_start_vel, planar_end_vel,
            planar_start_acc, planar_end_acc, time);
    }
    else if (pos.cols() == 2)
    {
        gl_traj = PolynomialTraj::one_segment_traj_gen(
            planar_start, planar_start_vel, planar_start_acc,
            planar_end, planar_end_vel, planar_end_acc, time(0));
    }
    else
    {
        return false;
    }

    // 保存生成的全局参考轨迹，供状态机提取局部目标和推进航点。
    global_data_.setGlobalTraj(gl_traj, rclcpp::Clock().now());
    return true;
  }

  bool EGOPlannerManager::refineTrajAlgo(UniformBspline &traj, vector<Eigen::Vector3d> &start_end_derivative, double ratio, double &ts, Eigen::MatrixXd &optimal_control_points)
  {
    // ratio 由 UniformBspline::checkFeasibility() 给出；大于 1 表示需要放慢。
    double t_inc;

    Eigen::MatrixXd ctrl_pts; // = traj.getControlPoint()

    // std::cout << "ratio: " << ratio << std::endl;
    reparamBspline(traj, start_end_derivative, ratio, ctrl_pts, ts, t_inc);

    traj = UniformBspline(ctrl_pts, 3, ts);

    // 每个有效 B 样条段取一个参考点。精修优化通过 fitness 代价约束新轨迹
    // 不要偏离时间拉伸后的原几何路径。
    double t_step = traj.getTimeSum() / (ctrl_pts.cols() - 3);
    bspline_optimizer_->ref_pts_.clear();
    for (double t = 0; t < traj.getTimeSum() + 1e-4; t += t_step)
      bspline_optimizer_->ref_pts_.push_back(traj.evaluateDeBoorT(t));

    bool success = bspline_optimizer_->BsplineOptimizeTrajRefine(ctrl_pts, ts, optimal_control_points);

    return success;
  }

  void EGOPlannerManager::updateTrajInfo(const UniformBspline &position_traj, const rclcpp::Time time_now)
  {
    // 位置轨迹是唯一独立保存的优化结果；速度/加速度由解析求导产生，
    // 从而保证三者在数学上严格一致。
    local_data_.start_time_ = time_now;
    local_data_.position_traj_ = position_traj;
    local_data_.velocity_traj_ = local_data_.position_traj_.getDerivative();
    local_data_.acceleration_traj_ = local_data_.velocity_traj_.getDerivative();
    local_data_.start_pos_ = local_data_.position_traj_.evaluateDeBoorT(0.0);
    local_data_.duration_ = local_data_.position_traj_.getTimeSum();
    local_data_.traj_id_ += 1;
  }

  void EGOPlannerManager::reparamBspline(UniformBspline &bspline, vector<Eigen::Vector3d> &start_end_derivative, double ratio,
                                         Eigen::MatrixXd &ctrl_pts, double &dt, double &time_inc)
  {
    double time_origin = bspline.getTimeSum();
    int seg_num = bspline.getControlPoint().cols() - 3;

    // 先拉长节点时间，不主动改变控制点的空间形状。
    bspline.lengthenTime(ratio);
    double duration = bspline.getTimeSum();
    dt = duration / double(seg_num);
    time_inc = duration - time_origin;

    vector<Eigen::Vector3d> point_set;
    for (double time = 0.0; time <= duration + 1e-4; time += dt)
    {
      point_set.push_back(bspline.evaluateDeBoorT(time));
    }
    // 使用拉长后的采样点以及原起终端导数重新求控制点。
    UniformBspline::parameterizeToBspline(dt, point_set, start_end_derivative, ctrl_pts);
  }

} // namespace ego_planner
