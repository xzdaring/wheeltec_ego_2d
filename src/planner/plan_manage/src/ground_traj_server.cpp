#include <csignal> // 先停车，再关闭 ROS 上下文。
#include <chrono>
#include <thread>
#include <string>
#include <algorithm> // clamp 用于限制轨迹求值时间。
#include <cmath> // 平面旋转与圆形碰撞检查。
#include <memory> // 智能指针保存 ROS 接口。
#include <vector> // 保存雷达命中点。
#include <rclcpp/rclcpp.hpp> // 使用节点时钟和周期回调。
#include <tf2_ros/buffer.h> // 查询雷达到底盘的真实 TF。
#include <tf2_ros/transform_listener.h> // 持续接收安装变换。
#include <geometry_msgs/msg/pose_stamped.hpp> // 已转换到 odom 的目标。
#include <geometry_msgs/msg/twist.hpp> // 麦轮车速度接口。
#include <nav_msgs/msg/odometry.hpp> // 位置反馈。
#include <sensor_msgs/msg/laser_scan.hpp> // 近障停车输入。
#include <sensor_msgs/msg/point_cloud2.hpp> // 感知链路看门狗。
#include <sensor_msgs/point_cloud2_iterator.hpp> // 读取膨胀点云的坐标字段。
#include <quadrotor_msgs/msg/position_command.hpp> // 保留期望状态供调试。
#include <traj_utils/msg/bspline.hpp> // 原 EGO 轨迹消息。
#include <bspline_opt/uniform_bspline.h> // 复用原 EGO 的 B 样条求值。

// 该节点同时完成轨迹采样和反馈控制；地图转换仍在 Python 桥中。
class GroundTrajServer : public rclcpp::Node
{
public:
  GroundTrajServer() : Node("ground_traj_server"),
    buffer_(get_clock()), listener_(buffer_)
  {
    // 参数集中定义，调参不需要进入控制公式。
    planning_frame_ = declare_parameter<std::string>("planning_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    kp_ = declare_parameter("kp_xy", 0.8);
    max_speed_ = declare_parameter("max_speed", 0.3);
    tolerance_ = declare_parameter("goal_tolerance", 0.12);
    yaw_gain_ = declare_parameter("kp_yaw", 1.5); // 到点后的角度反馈增益。
    max_yaw_rate_ = declare_parameter("max_yaw_rate", 0.6); // 转向限速 rad/s。
    yaw_tolerance_ = declare_parameter("yaw_tolerance", 0.05); // 朝向容差 rad。
    follow_yaw_ = declare_parameter("follow_trajectory_yaw", true); // 默认沿轨迹转动车头。
    yaw_lookahead_ = declare_parameter("yaw_lookahead", 0.3); // 提前看 0.3 s，减轻弯道滞后。
    radius_ = declare_parameter("safety_radius", 0.22);
    grid_resolution_ = declare_parameter("grid_resolution", 0.1); // 与规划器共用分辨率。
    // 所有订阅器保存在成员中，避免构造结束后被释放。
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {
        if (m->header.frame_id != planning_frame_ || m->child_frame_id != base_frame_) return;
        odom_ = *m; // 位姿已经表达在 odom 中。
        odom_time_ = now().seconds(); // 反馈中断立即停车。
      });
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/ego/cloud", 10, [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr) {
        cloud_time_ = now().seconds(); // 桥只发布新鲜地图产生的点云。
      });
    
    inflated_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>( // 接收规划器实际使用的禁行区。
      "/drone_0_grid/grid_map/occupancy_inflate", 1, // 只保留最新地图，避免处理积压。
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr m) { // 收到地图后更新缓存。
        onInflated(*m); // 坐标已经在 odom 中，不再重复变换。
      }); // 保存订阅器，确保回调持续生效。
    click_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 10, [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr) {
        active_ = have_goal_ = aligning_ = false; // 点击新目标立即废弃旧目标。
        stop(); // 等待桥完成目标坐标转换。
      });
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/move_base_simple/goal", 10,
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr m) {
        active_ = aligning_ = false; // 新目标必须等待新的规划结果。
        have_goal_ = m->header.frame_id == planning_frame_;
        goal_ = m->pose.position; // 位置与朝向都已经转换到 odom。
        goal_yaw_ = yaw(m->pose.orientation); // 保存点击箭头要求的最终朝向。
        goal_time_ = rclcpp::Time(m->header.stamp).seconds();
        required_id_ = last_id_; // 拒绝上一目标的轨迹编号。
        stop(); // 规划等待期间不沿旧轨迹运动。
      });
    spline_sub_ = create_subscription<traj_utils::msg::Bspline>(
      "/drone_0_planning/bspline", 10,
      [this](traj_utils::msg::Bspline::ConstSharedPtr m) { onSpline(*m); });
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::LaserScan::ConstSharedPtr m) { onScan(*m); });
    velocity_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    reference_pub_ = create_publisher<quadrotor_msgs::msg::PositionCommand>(
      "/ego/position_cmd", 10); // 调试接口，不再经消息转发参与反馈控制。
    timer_ = create_wall_timer(std::chrono::milliseconds(50),
      [this]() { control(); }); // 20 Hz 求值并控制，使用同一份反馈。
  }

  void stop()
  {
    velocity_pub_->publish(geometry_msgs::msg::Twist()); // 统一零速度出口。
  }

private:
  static double yaw(const geometry_msgs::msg::Quaternion &q)
  {
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
      1.0 - 2.0 * (q.y * q.y + q.z * q.z)); // 提取平面偏航角。
  }

  void onSpline(const traj_utils::msg::Bspline &m)
  {
    last_id_ = std::max(last_id_, m.traj_id); // 保存已接收的最大编号。
    const double start = rclcpp::Time(m.start_time).seconds();
    if (!have_goal_ || m.traj_id <= required_id_ || start < goal_time_ - 0.2)
      return; // 容许 /clock 传播延迟，但不能接受旧目标轨迹。
    const int count = static_cast<int>(m.pos_pts.size());
    if (m.order != 3 || count < 4 || m.knots.size() != m.pos_pts.size() + 4)
      return; // 普通三次 B 样条必须有正确的控制点和节点数量。
    Eigen::MatrixXd points(3, count); // 保持原 EGO 的三维存储接口。
    Eigen::VectorXd knots(m.knots.size()); // 使用规划器实际节点向量。
    for (int i = 0; i < count; ++i)
    {
      const auto &p = m.pos_pts[i]; // 每个控制点都应位于规划平面。
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || std::abs(p.z) > 1e-6)
        return; // 不把非法或非平面轨迹直接截断后执行。
      points.col(i) = Eigen::Vector3d(p.x, p.y, 0.0);
    }
    for (size_t i = 0; i < m.knots.size(); ++i)
    {
      if (!std::isfinite(m.knots[i]) ||
          (i > 0 && m.knots[i] <= m.knots[i - 1]))
        return; // 当前 EGO 使用严格递增均匀节点，拒绝零除风险。
      knots(i) = m.knots[i]; // 完整保留优化后的时间分配。
    }
    position_ = ego_planner::UniformBspline(points, m.order, 0.1);
    position_.setKnot(knots); // 临时间隔由消息节点向量覆盖。
    velocity_ = position_.getDerivative(); // 解析求导获得速度前馈。
    acceleration_ = velocity_.getDerivative(); // 曲线加速度用于计算切线转动速度。
    start_time_ = start; // 与当前节点的仿真时钟同源。
    duration_ = position_.getTimeSum(); // 末端后保持位置、速度归零。
    active_ = duration_ > 0.0; // 只有有效轨迹才解除停车。
  }

  void onScan(const sensor_msgs::msg::LaserScan &m)
  {
    geometry_msgs::msg::TransformStamped tr; // 保存雷达安装变换。
    try {
      tr = buffer_.lookupTransform(base_frame_, m.header.frame_id,
        rclcpp::Time(m.header.stamp)); // 按扫描时刻查询真实 TF。
    } catch (const tf2::TransformException &) {
      return; // 缺失 TF 不刷新看门狗，保持停车。
    }
    const double a = yaw(tr.transform.rotation); // 平面雷达安装方向。
    const auto &p = tr.transform.translation; // 雷达相对车体的偏移。
    scan_points_.clear(); // 每帧替换，不累积动态障碍残影。
    for (size_t i = 0; i < m.ranges.size(); ++i)
    {
      const double r = m.ranges[i]; // 无穷远表示未命中障碍。
      if (!std::isfinite(r) || r < m.range_min || r > m.range_max)
        continue; // 丢弃量程外测距。
      const double angle = m.angle_min + i * m.angle_increment + a;
      scan_points_.emplace_back(p.x + r * std::cos(angle),
        p.y + r * std::sin(angle)); // 转到车体坐标进行扫掠检查。
    }
    scan_time_ = now().seconds(); // 仅有效变换后的扫描刷新时间。
  }
  void onInflated(const sensor_msgs::msg::PointCloud2 &m) // 更新膨胀栅格缓存。
  {
    if (m.header.frame_id != planning_frame_) return; // 坐标不一致时不刷新地图。

    inflated_points_.clear(); // 每帧替换，避免已经消失的障碍残留。
    sensor_msgs::PointCloud2ConstIterator<float> x(m, "x"); // 读取栅格中心 x。
    sensor_msgs::PointCloud2ConstIterator<float> y(m, "y"); // 读取栅格中心 y。
    sensor_msgs::PointCloud2ConstIterator<float> z(m, "z"); // 选择规划平面附近的栅格。

    for (; x != x.end(); ++x, ++y, ++z) // 同步遍历三个坐标字段。
    {
      if (!std::isfinite(*x) || !std::isfinite(*y) || // 排除无效水平坐标。
          !std::isfinite(*z)) continue; // 排除无效高度。
      if (std::abs(*z) > grid_resolution_ * 0.51) continue; // 保留 z=0 邻近层。
      inflated_points_.emplace_back(*x, *y); // 保存 odom 下的禁行栅格中心。
    }

    inflated_time_ = now().seconds(); // 空地图也有效，因此仍更新时间。
  }

  static Eigen::Vector2d motionOffset( // 同时平移和旋转时，中心沿圆弧运动。
    double vx, double vy, double wz, double t) // 速度表达在预测起点的坐标轴下。
  {
    if (std::abs(wz) < 1e-6) return {vx * t, vy * t}; // 小角速度退化为直线。
    const double s = std::sin(wz * t) / wz; // 积分旋转后的前向速度。
    const double c = (1.0 - std::cos(wz * t)) / wz; // 积分旋转后的横向速度。
    return {s * vx - c * vy, c * vx + s * vy}; // 支持麦轮横移的恒定 Twist 预测。
  }

  bool blockedByMap(double vx, double vy, double wz = 0.0) const // 输入为规划坐标速度。
  {
    const auto &p = odom_.pose.pose.position; // 从实际车位开始预测。
    const double half = grid_resolution_ * 0.5 + 0.01; // 栅格半宽加 1 cm 余量。

    for (int i = 0; i <= 12; ++i) // 检查当前到未来 0.6 秒的位置。
    {
      const double t = i * 0.05; // 每 50 ms 检查一次，缩小采样间隙。
      const auto offset = motionOffset(vx, vy, wz, t); // 计入转向引起的圆弧位移。
      const double x = p.x + offset.x(); // 在规划坐标下预测中心 x。
      const double y = p.y + offset.y(); // 同时预测横移，保留全向运动。

      for (const auto &cell : inflated_points_) // 检查预测中心是否进入禁行格。
      {
        if (std::abs(x - cell.x()) <= half && // 判断是否进入栅格的 x 范围。
            std::abs(y - cell.y()) <= half) // 判断是否同时进入 y 范围。
          return true; // 预测进入膨胀区，拒绝该速度。
      }
    }

    return false; // 当前候选速度没有进入已知膨胀栅格。
  }


  bool blocked(double vx, double vy, double wz = 0.0) const // 速度表达在当前车体坐标中。
  {
    for (const double t : {0.0, 0.2, 0.4, 0.6}) // 保留原来的短期预测范围。
    {
      const auto offset = motionOffset(vx, vy, wz, t); // 先计算该时刻的圆弧位移。
      for (const auto &p : scan_points_) // 雷达点和预测位移使用同一坐标系。
        if ((p - offset).norm() < radius_) // 外接圆包络同时覆盖平移与转向。
          return true; // 预测车体包络碰到真实障碍，拒绝整个速度指令。
    }
    return false; // 当前雷达未检测到短期碰撞。
  }

  void followHeading(double t, bool final_approach, // 最终小范围收敛单独选择朝向。
    double &vx, double &vy, geometry_msgs::msg::Twist &cmd) // 同步调整平移和角速度。
  {
    if (!follow_yaw_) return; // 可恢复原来的全向平移方式，最终朝向仍然有效。
    const double future = std::min(duration_, t + std::max(0.0, yaw_lookahead_)); // 预瞄不越界。
    Eigen::Vector3d direction = velocity_.evaluateDeBoorT(future); // 采用真实曲线切线。
    double feed_yaw = 0.0; // 静止或末端收敛时不使用曲率前馈。
    if (final_approach) direction = Eigen::Vector3d(vx, vy, 0.0); // 朝剩余位置误差前进。
    else if (direction.head<2>().squaredNorm() >= 1e-4) // 低速时避免除法放大噪声。
    {
      const auto acc = acceleration_.evaluateDeBoorT(future); // 取同一时刻的加速度。
      feed_yaw = (direction.x() * acc.y() - direction.y() * acc.x()) // 切线角速度分子。
        / direction.head<2>().squaredNorm(); // d atan2(vy,vx)/dt，单位 rad/s。
      feed_yaw = std::clamp(feed_yaw, -max_yaw_rate_, max_yaw_rate_); // 限制曲率前馈。
    }
    else direction = position_.evaluateDeBoorT(future) // 起步速度为零时看前方曲线。
      - position_.evaluateDeBoorT(t); // 用短弦方向补充切线，避免 atan2(0,0)。
    if (direction.head<2>().squaredNorm() < 1e-8) return; // 急停常值轨迹不触发转向。
    reference_yaw_ = std::atan2(direction.y(), direction.x()); // 车头沿轨迹前进方向。
    const double delta = reference_yaw_ - yaw(odom_.pose.pose.orientation); // 实际车头误差。
    const double error = std::atan2(std::sin(delta), std::cos(delta)); // 跨过 ±π 也走短角。
    const double scale = std::max(0.0, std::cos(error)); // 偏差越大越慢，超过 90°先转向。
    vx *= scale; // 同比例降低规划坐标下的速度，保留路径纠偏方向。
    vy *= scale; // 保留横移能力，不强行把车改为非完整约束底盘。
    cmd.angular.z = std::clamp(scale * feed_yaw + yaw_gain_ * error, // 前馈加角度反馈。
      -max_yaw_rate_, max_yaw_rate_); // 行驶转向与最终对齐共用角速度上限。
  }

  void control()
  {
    const double time = now().seconds(); // 进度与超时都使用节点仿真时间。
    const bool fresh = time - odom_time_ < 0.5 && // 里程计必须持续更新。
      time - scan_time_ < 0.5 && // 雷达必须持续更新。
      time - cloud_time_ < 0.7 && // 在线建图链路必须正常。
      time - inflated_time_ < 0.5; // 膨胀地图中断时也停车。
    if (!have_goal_ || !fresh) // 到点转向不依赖新的平移轨迹。
    {
      stop(); // 等待目标、轨迹或感知时都主动输出零速度。
      return;
    }
    const auto &p = odom_.pose.pose.position; // 当前真实底盘位置。
    const double distance = std::hypot(goal_.x - p.x, goal_.y - p.y);
    if (distance < tolerance_ * 0.5 || (aligning_ && distance < tolerance_))
    // 进入转向用更严格的位置阈值，退出保留原容差，避免阈值附近来回切换。
    {
      aligning_ = true; // 已进入最终转向阶段，允许容差内的小幅漂移。
      const double error = goal_yaw_ - yaw(odom_.pose.pose.orientation);
      const double angle = std::atan2(std::sin(error), std::cos(error)); // 最短角差。
      geometry_msgs::msg::Twist turn; // 到点后平移归零，仅调整朝向。
      if (std::abs(angle) > yaw_tolerance_) // 角度未满足则保持导航状态。
      {
        if (!blocked(0.0, 0.0) && // 雷达确认圆形包络内没有障碍。
            !blockedByMap(0.0, 0.0)) // 当前中心也必须位于膨胀禁行区外。 // 圆形包络内有障碍时禁止原地旋转。
          turn.angular.z = std::clamp(yaw_gain_ * angle,
            -max_yaw_rate_, max_yaw_rate_); // 限制角速度，保留最短旋转方向。
      }
      else // 位置和朝向同时达标才宣布成功。
      {
        active_ = have_goal_ = false; // 下一次点击重新启动。
        RCLCPP_INFO(get_logger(), "位置与最终朝向均已到达，已停车。");
      }
      reference_yaw_ = goal_yaw_; // 最终朝向优先于轨迹切线。
      reference_yaw_rate_ = turn.angular.z; // 调试输出对应实际下发的转向速度。
      publishReference(Eigen::Vector3d(goal_.x, goal_.y, 0.0), // 显示最终位置要求。
        Eigen::Vector3d::Zero()); // 原地对齐阶段期望平移速度为零。
      velocity_pub_->publish(turn); // 旋转、等待或完成都显式发布速度。
      return;
    }
    aligning_ = false; // 位置超出容差时重新平移，不能只满足角度。
    if (!active_ || time < start_time_) // 平移阶段必须等待有效的新轨迹。
    {
      stop(); // 避免新目标执行旧平移参考。
      return;
    }
    const double t = std::clamp(time - start_time_, 0.0, duration_);
    Eigen::Vector3d target = position_.evaluateDeBoorT(t); // 当前曲线参考点。
    const bool ended = time - start_time_ >= duration_; // 是否已到轨迹末端。
    const bool final_segment = std::hypot(target.x() - goal_.x,
      target.y() - goal_.y) < tolerance_ * 2.0; // 区分最终段与滚动局部段。
    if (ended && final_segment) // EGO 自由末端可能略偏离最终目标。
    {
      target.x() = goal_.x; // 最后小范围用反馈收敛到点击位置。
      target.y() = goal_.y; // 同样经过限速与雷达扫掠停车检查。
    }
    Eigen::Vector3d feed = velocity_.evaluateDeBoorT(t); // 直接复用轨迹导数。
    if (time - start_time_ >= duration_)
      feed.setZero(); // 超出轨迹时长后仅靠位置反馈收敛到末端。
    double vx = feed.x() + kp_ * (target.x() - p.x);
    double vy = feed.y() + kp_ * (target.y() - p.y);
    const double scale = std::min(1.0,
      max_speed_ / std::max(std::hypot(vx, vy), 1e-9)); // 限合速度。
    vx *= scale; // 前后与横移同比例缩放，保持运动方向。
    vy *= scale; // 防止对角移动超速。
    const double a = yaw(odom_.pose.pose.orientation);
    reference_yaw_ = a; // 无有效切线或关闭跟随时，以当前车头作为调试参考。
    geometry_msgs::msg::Twist cmd; // 从零角速度开始计算本周期转向。
    followHeading(t, ended && final_segment, vx, vy, cmd); // 跟随切线并按角差减速。
    cmd.linear.x = std::cos(a) * vx + std::sin(a) * vy;
    cmd.linear.y = -std::sin(a) * vx + std::cos(a) * vy;
    const bool scan_blocked = blocked(cmd.linear.x, cmd.linear.y, cmd.angular.z);
    const bool map_blocked = blockedByMap(vx, vy, cmd.angular.z);
    if (scan_blocked || map_blocked)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "导航停车: scan=%s inflated_map=%s goal_distance=%.3f m",
        scan_blocked ? "blocked" : "clear", map_blocked ? "blocked" : "clear", distance);
      cmd = geometry_msgs::msg::Twist(); // 保留原有碰撞检查，不为提速绕过停车。
    }
    reference_yaw_rate_ = cmd.angular.z; // 近障停车后，调试角速度也应归零。
    publishReference(target, feed); // 同时发布位置、速度和车头参考，便于检查。
    velocity_pub_->publish(cmd); // 此节点是导航链唯一速度发布者。
  }

  void publishReference(const Eigen::Vector3d &p, const Eigen::Vector3d &v)
  {
    quadrotor_msgs::msg::PositionCommand ref; // 仅用于可视化与调试。
    ref.header.stamp = now(); // 使用采样时刻。
    ref.header.frame_id = planning_frame_; // 期望状态已经位于 odom。
    ref.position.x = p.x(); // 保留轨迹期望位置。
    ref.position.y = p.y(); // 高度默认零。
    ref.velocity.x = v.x(); // 水平速度前馈。
    ref.velocity.y = v.y(); // 横移速度前馈。
    ref.yaw = reference_yaw_; // 输出期望车头，方便和里程计姿态对比。
    ref.yaw_dot = reference_yaw_rate_; // 输出控制器下发的角速度，单位 rad/s。
    ref.trajectory_id = last_id_; // 对应当前轨迹编号。
    reference_pub_->publish(ref); // 不再依赖此消息闭环控制。
  }

  std::string planning_frame_, base_frame_;
  tf2_ros::Buffer buffer_; // 与节点时钟绑定的 TF 缓冲。
  tf2_ros::TransformListener listener_; // 生命周期覆盖整个节点。
  ego_planner::UniformBspline position_, velocity_; // 位置曲线及其导数。
  ego_planner::UniformBspline acceleration_; // 切线角速度前馈所需的二阶导数。
  nav_msgs::msg::Odometry odom_; // 保存完整姿态反馈。
  geometry_msgs::msg::Point goal_; // 最终位置目标。
  std::vector<Eigen::Vector2d> scan_points_; // 车体坐标下的障碍。
  std::vector<Eigen::Vector2d> inflated_points_; // odom 下的膨胀栅格中心。
  double inflated_time_ = -100.0; // 未收到膨胀地图前保持停车。
  double grid_resolution_ = 0.1; // 与 EGO 的 grid_map/resolution 保持一致。
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr inflated_sub_; // 保持订阅器存活。
  bool aligning_ = false; // 最终朝向阶段的滞回状态。
  bool active_ = false, have_goal_ = false; // 默认禁止运动。
  int64_t last_id_ = -1, required_id_ = -1; // 区分连续目标。
  double goal_time_ = 0.0, start_time_ = 0.0, duration_ = 0.0;
  double odom_time_ = -100.0, scan_time_ = -100.0, cloud_time_ = -100.0;
  double goal_yaw_ = 0.0; // 已变换到 odom 的最终朝向。
  bool follow_yaw_ = true; // 行驶时沿轨迹切线控制车头。
  double yaw_lookahead_ = 0.3; // 切线预瞄时间，单位秒。
  double reference_yaw_ = 0.0, reference_yaw_rate_ = 0.0; // 调试输出的角度与角速度。
  double yaw_gain_, max_yaw_rate_, yaw_tolerance_; // 到点原地转向参数。
  double kp_, max_speed_, tolerance_, radius_; // 控制与停车参数。
  rclcpp::TimerBase::SharedPtr timer_; // 保持控制周期回调存活。
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr click_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<traj_utils::msg::Bspline>::SharedPtr spline_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_pub_;
  rclcpp::Publisher<quadrotor_msgs::msg::PositionCommand>::SharedPtr reference_pub_;
};

namespace {
volatile std::sig_atomic_t stop_requested = 0;
void request_stop(int) { stop_requested = 1; } // 信号回调只置位，不调用 ROS API。
}

int main(int argc, char **argv)
{
  // 默认 rclcpp 信号处理会立即关闭发布者，改为在主循环中先发送零速度。
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, request_stop);
  std::signal(SIGTERM, request_stop);
  auto node = std::make_shared<GroundTrajServer>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  while (rclcpp::ok() && !stop_requested)
    executor.spin_once(std::chrono::milliseconds(10));
  if (rclcpp::ok()) {
    for (int i = 0; i < 3; ++i) {
      node->stop(); // 保持上下文有效，尽力将停车消息送到底盘。
      std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }
  }
  rclcpp::shutdown();
  return 0;
}
