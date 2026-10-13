/**
 * @file traj_server.cpp
 * @brief 接收规划器发布的 B 样条，在控制周期内求位置/速度/加速度并发布 PositionCommand。
 *
 * 规划节点发布的是整条连续轨迹；本节点依据当前 ROS 时间减 start_time 得到轨迹
 * 相对时间，再通过 De Boor 求值产生控制器需要的逐时刻参考量。
 */
#include "bspline_opt/uniform_bspline.h"
#include "nav_msgs/msg/odometry.hpp"
#include "traj_utils/msg/bspline.hpp"
#include "quadrotor_msgs/msg/position_command.hpp"
#include "std_msgs/msg/empty.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <rclcpp/rclcpp.hpp>

// 本文件使用全局变量是原项目的过程式写法；若重构为 Node 子类，可把这些
// 状态收进类成员。这里保持原行为，只解释每个变量的用途。
rclcpp::Clock::SharedPtr trajectory_clock; // 保留节点时钟以接收 /clock 的仿真时间。
rclcpp::Publisher<quadrotor_msgs::msg::PositionCommand>::SharedPtr pos_cmd_pub;

quadrotor_msgs::msg::PositionCommand cmd;
double pos_gain[3] = {0, 0, 0}; ///< PositionCommand 携带的位置反馈增益，当前均为 0。
double vel_gain[3] = {0, 0, 0}; ///< PositionCommand 携带的速度反馈增益，当前均为 0。

using ego_planner::UniformBspline;

// traj_server 是规划器和控制器之间的“轨迹采样器”：
// 它接收一整条 B 样条，随后以 100 Hz 输出当前时刻的期望位置、速度和加速度。
bool receive_traj_ = false;
// traj_[0]、traj_[1]、traj_[2] 分别是位置、速度和加速度 B 样条。
vector<UniformBspline> traj_;
double traj_duration_;      ///< 当前位置 B 样条有效时长，秒。
rclcpp::Time start_time_;   ///< 当前轨迹在 ROS 时钟上的绝对开始时刻。
int traj_id_;               ///< 当前轨迹版本号，控制端可用于识别轨迹更新。

// yaw control
double last_yaw_;     ///< 上一控制周期滤波后的偏航角，弧度。
double last_yaw_dot_; ///< 上一控制周期滤波后的偏航角速度，rad/s。
double time_forward_; ///< 用未来多少秒处的轨迹点决定朝向，秒。

void bsplineCallback(traj_utils::msg::Bspline::ConstPtr msg)
{
  // 1. 从 ROS 消息还原位置控制点和节点向量。

  Eigen::MatrixXd pos_pts(3, msg->pos_pts.size());

  Eigen::VectorXd knots(msg->knots.size());
  for (size_t i = 0; i < msg->knots.size(); ++i)
  {
    knots(i) = msg->knots[i];
  }

  for (size_t i = 0; i < msg->pos_pts.size(); ++i)
  {
    pos_pts(0, i) = msg->pos_pts[i].x;
    pos_pts(1, i) = msg->pos_pts[i].y;
    pos_pts(2, i) = msg->pos_pts[i].z;
  }

  // 构造函数中的 0.1 只是临时间隔；下一行 setKnot() 会以消息中的完整节点
  // 向量覆盖它，因此最终轨迹时间参数仍由规划器发布的 knots 决定。
  UniformBspline pos_traj(pos_pts, msg->order, 0.1);
  pos_traj.setKnot(knots);

  // parse yaw traj

  // Eigen::MatrixXd yaw_pts(msg->yaw_pts.size(), 1);
  // for (int i = 0; i < msg->yaw_pts.size(); ++i) {
  //   yaw_pts(i, 0) = msg->yaw_pts[i];
  // }

  // UniformBspline yaw_traj(yaw_pts, msg->order, msg->yaw_dt);

  // 2. start_time_ 是轨迹在统一时钟上的绝对起点。cmdCallback() 用
  // now - start_time_ 得到轨迹内部时间，所以规划器和轨迹服务器必须同源计时。
  start_time_ = msg->start_time;
  traj_id_ = msg->traj_id;

  traj_.clear();
  // 3. B 样条的导数仍是 B 样条，可直接得到速度和加速度轨迹。
  traj_.push_back(pos_traj);
  traj_.push_back(traj_[0].getDerivative());
  traj_.push_back(traj_[1].getDerivative());

  traj_duration_ = traj_[0].getTimeSum();

  receive_traj_ = true;
}

std::pair<double, double> calculate_yaw(double t_cur, Eigen::Vector3d &pos, rclcpp::Time &time_now, rclcpp::Time &time_last)
{
  /*
   * 偏航角计算思路：
   * 1. 用“当前点 -> 前视点”的水平投影方向作为期望机头方向 yaw_temp；
   * 2. 角度范围是 [-pi, pi]，跨越边界时不能直接相减。例如从 179° 到 -179°
   *    实际只应旋转 2°，而不是反向旋转 358°；所以下面两大分支专门处理 ±pi 绕回；
   * 3. 每周期最大允许变化量 = 最大角速度 × 周期时长，防止机头瞬间跳变；
   * 4. 最后用新旧值各 0.5 的一阶低通滤波进一步减小抖动。
   * 返回值 first 是偏航角 rad，second 是偏航角速度 rad/s。
   */
  constexpr double PI = 3.1415926;
  constexpr double YAW_DOT_MAX_PER_SEC = PI;
  // constexpr double YAW_DOT_DOT_MAX_PER_SEC = PI;
  std::pair<double, double> yaw_yawdot(0, 0);
  double yaw = 0;
  double yawdot = 0;

  // 偏航朝向轨迹前方 time_forward_ 秒处，而不是直接使用瞬时速度方向。
  // 前视能减少低速或局部曲率变化时的偏航抖动。
  Eigen::Vector3d dir = t_cur + time_forward_ <= traj_duration_ ? traj_[0].evaluateDeBoorT(t_cur + time_forward_) - pos : traj_[0].evaluateDeBoorT(traj_duration_) - pos;
  double yaw_temp = dir.norm() > 0.1 ? atan2(dir(1), dir(0)) : last_yaw_;
  double max_yaw_change = YAW_DOT_MAX_PER_SEC * (time_now - time_last).seconds();
  if (yaw_temp - last_yaw_ > PI)
  {
    // 期望角看似比当前角大于 pi：优先考虑跨过 -pi/pi 边界的较短旋转。
    if (yaw_temp - last_yaw_ - 2 * PI < -max_yaw_change)
    {
      yaw = last_yaw_ - max_yaw_change;
      if (yaw < -PI)
        yaw += 2 * PI;

      yawdot = -YAW_DOT_MAX_PER_SEC;
    }
    else
    {
      yaw = yaw_temp;
      if (yaw - last_yaw_ > PI)
        yawdot = -YAW_DOT_MAX_PER_SEC;
      else
        yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).seconds();
    }
  }
  else if (yaw_temp - last_yaw_ < -PI)
  {
    // 与上一分支对称：期望角看似小于当前角超过 pi。
    if (yaw_temp - last_yaw_ + 2 * PI > max_yaw_change)
    {
      yaw = last_yaw_ + max_yaw_change;
      if (yaw > PI)
        yaw -= 2 * PI;

      yawdot = YAW_DOT_MAX_PER_SEC;
    }
    else
    {
      yaw = yaw_temp;
      if (yaw - last_yaw_ < -PI)
        yawdot = YAW_DOT_MAX_PER_SEC;
      else
        yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).seconds();
    }
  }
  else
  {
    // 没有跨越 ±pi 边界，直接按最大角速度限制本周期转角。
    if (yaw_temp - last_yaw_ < -max_yaw_change)
    {
      yaw = last_yaw_ - max_yaw_change;
      if (yaw < -PI)
        yaw += 2 * PI;

      yawdot = -YAW_DOT_MAX_PER_SEC;
    }
    else if (yaw_temp - last_yaw_ > max_yaw_change)
    {
      yaw = last_yaw_ + max_yaw_change;
      if (yaw > PI)
        yaw -= 2 * PI;

      yawdot = YAW_DOT_MAX_PER_SEC;
    }
    else
    {
      yaw = yaw_temp;
      if (yaw - last_yaw_ > PI)
        yawdot = -YAW_DOT_MAX_PER_SEC;
      else if (yaw - last_yaw_ < -PI)
        yawdot = YAW_DOT_MAX_PER_SEC;
      else
        yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).seconds();
    }
  }

  if (fabs(yaw - last_yaw_) <= max_yaw_change)
    yaw = 0.5 * last_yaw_ + 0.5 * yaw; // 简单一阶低通滤波（原注释中的 nieve 应为 naive）。
  yawdot = 0.5 * last_yaw_dot_ + 0.5 * yawdot;
  last_yaw_ = yaw;
  last_yaw_dot_ = yawdot;

  yaw_yawdot.first = yaw;
  yaw_yawdot.second = yawdot;

  return yaw_yawdot;
}

void cmdCallback()
{
  /* no publishing before receive traj_ */
  if (!receive_traj_)
    return;

  // 统一时间源
  auto &clock = *trajectory_clock; // 独立 ROS Clock 未绑定时间源，必须使用节点 Clock。
  rclcpp::Time time_now = clock.now();
  // 把绝对 ROS 时间转换为 B 样条的局部参数时间。
  double t_cur = (time_now - start_time_).seconds();

  Eigen::Vector3d pos(Eigen::Vector3d::Zero()), vel(Eigen::Vector3d::Zero()), acc(Eigen::Vector3d::Zero()), pos_f;
  std::pair<double, double> yaw_yawdot(0, 0);

  static rclcpp::Time time_last = clock.now();
  if (t_cur < traj_duration_ && t_cur >= 0.0)
  {
    // De Boor 算法分别计算这一时刻的位置及其一、二阶导数。
    pos = traj_[0].evaluateDeBoorT(t_cur);
    vel = traj_[1].evaluateDeBoorT(t_cur);
    acc = traj_[2].evaluateDeBoorT(t_cur);

    /*** calculate yaw ***/
    yaw_yawdot = calculate_yaw(t_cur, pos, time_now, time_last);
    /*** calculate yaw ***/

    double tf = min(traj_duration_, t_cur + 2.0);
    pos_f = traj_[0].evaluateDeBoorT(tf);
  }
  else if (t_cur >= traj_duration_)
  {
    // 轨迹完成后持续发布末端位置和零速度、零加速度，使飞行器悬停。
    pos = traj_[0].evaluateDeBoorT(traj_duration_);
    vel.setZero();
    acc.setZero();

    yaw_yawdot.first = last_yaw_;
    yaw_yawdot.second = 0;

    pos_f = pos;
  }
  else
  {
    return; // 轨迹尚未开始时不发布虚假的零位置目标。
  }
  time_last = time_now;

  cmd.header.stamp = time_now;
  cmd.header.frame_id = "odom"; // 小车轨迹实际在 odom 中生成，反馈控制使用同一坐标系。
  cmd.trajectory_flag = quadrotor_msgs::msg::PositionCommand::TRAJECTORY_STATUS_READY;
  cmd.trajectory_id = traj_id_;

  cmd.position.x = pos(0);
  cmd.position.y = pos(1);
  cmd.position.z = pos(2);

  cmd.velocity.x = vel(0);
  cmd.velocity.y = vel(1);
  cmd.velocity.z = vel(2);

  cmd.acceleration.x = acc(0);
  cmd.acceleration.y = acc(1);
  cmd.acceleration.z = acc(2);

  cmd.yaw = yaw_yawdot.first;
  cmd.yaw_dot = yaw_yawdot.second;

  last_yaw_ = cmd.yaw;

  pos_cmd_pub->publish(cmd);
}

int main(int argc, char **argv)
{
  // traj_server 是与 ego_planner_node 分开的第二个 ROS 2 进程。
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("traj_server"); // 创建轨迹采样节点并接收 use_sim_time 参数。
  trajectory_clock = node->get_clock(); // 回调统一使用节点绑定的仿真时钟。

  // 队列深度 10：接收规划器偶尔更新的整条 B 样条消息。
  auto bspline_sub = node->create_subscription<traj_utils::msg::Bspline>(
      "planning/bspline",
      10,
      bsplineCallback);

  // 发布器队列深度 50；话题会在 launch 中重映射到本架无人机的命名空间。
  pos_cmd_pub = node->create_publisher<quadrotor_msgs::msg::PositionCommand>(
      "/position_cmd",
      50);

  // 每 10 ms 采样一次当前 B 样条，相当于约 100 Hz 的指令更新率。
  auto cmd_timer = node->create_wall_timer(
      std::chrono::milliseconds(10),
      cmdCallback);

  // PositionCommand 消息携带的反馈增益；当前数组初始化为 0，实际 SO3 控制器
  // 也可从自己的 YAML 文件读取控制增益，两者不要混为一谈。
  cmd.kx[0] = pos_gain[0];
  cmd.kx[1] = pos_gain[1];
  cmd.kx[2] = pos_gain[2];

  cmd.kv[0] = vel_gain[0];
  cmd.kv[1] = vel_gain[1];
  cmd.kv[2] = vel_gain[2];

  // 偏航前视时间：值越大，机头更早朝向远处路径；过大会削弱对急弯的响应。
  node->declare_parameter("traj_server/time_forward", -1.0);
  node->get_parameter("traj_server/time_forward", time_forward_);

  last_yaw_ = 0.0;
  last_yaw_dot_ = 0.0;

  // 启动后短暂等待，使其他仿真/控制节点有时间完成初始化。
  rclcpp::sleep_for(std::chrono::seconds(1));

  RCLCPP_WARN(node->get_logger(), "[Traj server]: ready.");

  // 进入回调循环，持续处理新轨迹并执行 10 ms 定时器。
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
