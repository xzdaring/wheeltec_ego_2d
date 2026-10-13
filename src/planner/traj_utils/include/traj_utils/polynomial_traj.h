#ifndef _POLYNOMIAL_TRAJ_H
#define _POLYNOMIAL_TRAJ_H

#include <Eigen/Eigen>
#include <vector>

using std::vector;

/**
 * @brief 分段三维多项式轨迹容器及最小 snap 初值生成器。
 *
 * 每段 x/y/z 使用同阶多项式和同一段时长。系数数组按“最高次项到常数项”
 * 保存；evaluate*() 会先把全局轨迹时间换算成本段局部时间，再计算位置及导数。
 * 在 EGO 规划流程中，该类通常只负责生成 B 样条优化前的连续初值。
 */
class PolynomialTraj
{
private:
  vector<double> times;       // time of each segment；每段持续时间，[s]
  vector<vector<double>> cxs; // coefficient of x of each segment, from n-1 -> 0；x 轴降幂系数
  vector<vector<double>> cys; // coefficient of y of each segment；y 轴降幂系数
  vector<vector<double>> czs; // coefficient of z of each segment；z 轴降幂系数

  double time_sum;            ///< 所有分段时长之和，[s]；调用 init() 后有效。
  int num_seg;                ///< 分段数量，等于 times.size()。

  /* evaluation */
  vector<Eigen::Vector3d> traj_vec3d; ///< getTraj() 以 0.01 s 采样得到的缓存点。
  double length;                       ///< 基于 traj_vec3d 折线累加的近似长度，[m]。

public:
  PolynomialTraj(/* args */)
  {
  }
  ~PolynomialTraj()
  {
  }

  /// 清空所有分段和统计量，使对象可重新填充。
  void reset()
  {
    times.clear(), cxs.clear(), cys.clear(), czs.clear();
    time_sum = 0.0, num_seg = 0;
  }

  /**
   * @brief 追加一段多项式。
   * @param cx/cy/cz 三个轴从最高次到常数项排列的系数；三者长度必须相同。
   * @param t 本段持续时间，[s]，必须为正。
   */
  void addSegment(vector<double> cx, vector<double> cy, vector<double> cz, double t)
  {
    cxs.push_back(cx), cys.push_back(cy), czs.push_back(cz), times.push_back(t);
  }

  /// 在所有 addSegment() 完成后汇总段数和总时间。
  void init()
  {
    num_seg = times.size();
    time_sum = 0.0;
    for (int i = 0; i < times.size(); ++i)
    {
      time_sum += times[i];
    }
  }

  vector<double> getTimes()
  {
    return times;
  }

  /// 返回指定轴的全部分段系数：axis=0/1/2 分别对应 x/y/z。
  vector<vector<double>> getCoef(int axis)
  {
    switch (axis)
    {
    case 0:
      return cxs;
    case 1:
      return cys;
    case 2:
      return czs;
    default:
      std::cout << "\033[31mIllegal axis!\033[0m" << std::endl;
    }

    vector<vector<double>> empty;
    return empty;
  }

  /**
   * @brief 计算整条轨迹在 t 时刻的位置。
   * @param t 相对整条轨迹起点的时间，[s]；调用者应保证 0<=t<=time_sum。
   *
   * idx 是 t 所属的分段编号；while 每越过一段，就从 t 中减去该段时长，
   * 最终留下本段局部时间。tv 是 [t^(n-1), ..., t, 1] 的幂向量。
   */
  Eigen::Vector3d evaluate(double t)
  {
    /* detetrmine segment num */
    int idx = 0;
    while (times[idx] + 1e-4 < t)
    {
      t -= times[idx];
      ++idx;
    }

    /* evaluation */
    int order = cxs[idx].size();
    Eigen::VectorXd cx(order), cy(order), cz(order), tv(order);
    for (int i = 0; i < order; ++i)
    {
      cx(i) = cxs[idx][i], cy(i) = cys[idx][i], cz(i) = czs[idx][i];
      tv(order - 1 - i) = std::pow(t, double(i));
    }

    Eigen::Vector3d pt;
    pt(0) = tv.dot(cx), pt(1) = tv.dot(cy), pt(2) = tv.dot(cz);
    return pt;
  }

  /// 计算 t 时刻速度，[m/s]；分段定位方式与 evaluate() 相同。
  Eigen::Vector3d evaluateVel(double t)
  {
    /* detetrmine segment num */
    int idx = 0;
    while (times[idx] + 1e-4 < t)
    {
      t -= times[idx];
      ++idx;
    }

    /* evaluation */
    int order = cxs[idx].size();
    Eigen::VectorXd vx(order - 1), vy(order - 1), vz(order - 1);

    /* coef of vel */
    for (int i = 0; i < order - 1; ++i)
    {
      vx(i) = double(i + 1) * cxs[idx][order - 2 - i];
      vy(i) = double(i + 1) * cys[idx][order - 2 - i];
      vz(i) = double(i + 1) * czs[idx][order - 2 - i];
    }
    double ts = t;
    Eigen::VectorXd tv(order - 1);
    for (int i = 0; i < order - 1; ++i)
      tv(i) = pow(ts, i);

    Eigen::Vector3d vel;
    vel(0) = tv.dot(vx), vel(1) = tv.dot(vy), vel(2) = tv.dot(vz);
    return vel;
  }

  /// 计算 t 时刻加速度，[m/s^2]；先对每段系数做两次解析求导。
  Eigen::Vector3d evaluateAcc(double t)
  {
    /* detetrmine segment num */
    int idx = 0;
    while (times[idx] + 1e-4 < t)
    {
      t -= times[idx];
      ++idx;
    }

    /* evaluation */
    int order = cxs[idx].size();
    Eigen::VectorXd ax(order - 2), ay(order - 2), az(order - 2);

    /* coef of vel */
    for (int i = 0; i < order - 2; ++i)
    {
      ax(i) = double((i + 2) * (i + 1)) * cxs[idx][order - 3 - i];
      ay(i) = double((i + 2) * (i + 1)) * cys[idx][order - 3 - i];
      az(i) = double((i + 2) * (i + 1)) * czs[idx][order - 3 - i];
    }
    double ts = t;
    Eigen::VectorXd tv(order - 2);
    for (int i = 0; i < order - 2; ++i)
      tv(i) = pow(ts, i);

    Eigen::Vector3d acc;
    acc(0) = tv.dot(ax), acc(1) = tv.dot(ay), acc(2) = tv.dot(az);
    return acc;
  }

  /* for evaluating traj, should be called in sequence!!! */
  double getTimeSum()
  {
    return this->time_sum;
  }

  /// 以固定 0.01 s 时间步长采样整条轨迹，并刷新 traj_vec3d 缓存。
  vector<Eigen::Vector3d> getTraj()
  {
    double eval_t = 0.0;
    traj_vec3d.clear();
    while (eval_t < time_sum)
    {
      Eigen::Vector3d pt = evaluate(eval_t);
      traj_vec3d.push_back(pt);
      eval_t += 0.01;
    }
    return traj_vec3d;
  }

  /// 累加 getTraj() 缓存点之间的距离；使用前应先调用 getTraj()。
  double getLength()
  {
    length = 0.0;

    Eigen::Vector3d p_l = traj_vec3d[0], p_n;
    for (int i = 1; i < traj_vec3d.size(); ++i)
    {
      p_n = traj_vec3d[i];
      length += (p_n - p_l).norm();
      p_l = p_n;
    }
    return length;
  }

  double getMeanVel()
  {
    double mean_vel = length / time_sum;
  }

  /// 计算当前实现采用的近似加速度代价，用于轨迹质量评估。
  double getAccCost()
  {
    double cost = 0.0;
    int order = cxs[0].size();

    for (int s = 0; s < times.size(); ++s)
    {
      Eigen::Vector3d um;
      um(0) = 2 * cxs[s][order - 3], um(1) = 2 * cys[s][order - 3], um(2) = 2 * czs[s][order - 3];
      cost += um.squaredNorm() * times[s];
    }

    return cost;
  }

  /// 解析积分各段 jerk 平方，数值越小通常表示轨迹越平滑。
  double getJerk()
  {
    double jerk = 0.0;

    /* evaluate jerk */
    for (int s = 0; s < times.size(); ++s)
    {
      Eigen::VectorXd cxv(cxs[s].size()), cyv(cys[s].size()), czv(czs[s].size());
      /* convert coefficient */
      int order = cxs[s].size();
      for (int j = 0; j < order; ++j)
      {
        cxv(j) = cxs[s][order - 1 - j], cyv(j) = cys[s][order - 1 - j], czv(j) = czs[s][order - 1 - j];
      }
      double ts = times[s];

      /* jerk matrix */
      Eigen::MatrixXd mat_jerk(order, order);
      mat_jerk.setZero();
      for (int i = 3; i < order; i += 1)
        for (int j = 3; j < order; j += 1)
        {
          mat_jerk(i, j) =
              i * (i - 1) * (i - 2) * j * (j - 1) * (j - 2) * pow(ts, i + j - 5) / (i + j - 5);
        }

      jerk += (cxv.transpose() * mat_jerk * cxv)(0, 0);
      jerk += (cyv.transpose() * mat_jerk * cyv)(0, 0);
      jerk += (czv.transpose() * mat_jerk * czv)(0, 0);
    }

    return jerk;
  }

  void getMeanAndMaxVel(double &mean_v, double &max_v)
  {
    int num = 0;
    mean_v = 0.0, max_v = -1.0;
    for (int s = 0; s < times.size(); ++s)
    {
      int order = cxs[s].size();
      Eigen::VectorXd vx(order - 1), vy(order - 1), vz(order - 1);

      /* coef of vel */
      for (int i = 0; i < order - 1; ++i)
      {
        vx(i) = double(i + 1) * cxs[s][order - 2 - i];
        vy(i) = double(i + 1) * cys[s][order - 2 - i];
        vz(i) = double(i + 1) * czs[s][order - 2 - i];
      }
      double ts = times[s];

      double eval_t = 0.0;
      while (eval_t < ts)
      {
        Eigen::VectorXd tv(order - 1);
        for (int i = 0; i < order - 1; ++i)
          tv(i) = pow(ts, i);
        Eigen::Vector3d vel;
        vel(0) = tv.dot(vx), vel(1) = tv.dot(vy), vel(2) = tv.dot(vz);
        double vn = vel.norm();
        mean_v += vn;
        if (vn > max_v)
          max_v = vn;
        ++num;

        eval_t += 0.01;
      }
    }

    mean_v = mean_v / double(num);
  }

  void getMeanAndMaxAcc(double &mean_a, double &max_a)
  {
    int num = 0;
    mean_a = 0.0, max_a = -1.0;
    for (int s = 0; s < times.size(); ++s)
    {
      int order = cxs[s].size();
      Eigen::VectorXd ax(order - 2), ay(order - 2), az(order - 2);

      /* coef of acc */
      for (int i = 0; i < order - 2; ++i)
      {
        ax(i) = double((i + 2) * (i + 1)) * cxs[s][order - 3 - i];
        ay(i) = double((i + 2) * (i + 1)) * cys[s][order - 3 - i];
        az(i) = double((i + 2) * (i + 1)) * czs[s][order - 3 - i];
      }
      double ts = times[s];

      double eval_t = 0.0;
      while (eval_t < ts)
      {
        Eigen::VectorXd tv(order - 2);
        for (int i = 0; i < order - 2; ++i)
          tv(i) = pow(ts, i);
        Eigen::Vector3d acc;
        acc(0) = tv.dot(ax), acc(1) = tv.dot(ay), acc(2) = tv.dot(az);
        double an = acc.norm();
        mean_a += an;
        if (an > max_a)
          max_a = an;
        ++num;

        eval_t += 0.01;
      }
    }

    mean_a = mean_a / double(num);
  }

  /**
   * @brief 生成经过多个给定位置的分段最小 snap 轨迹。
   * @param Pos 3×(M+1) 航点矩阵，每列一个三维点，共形成 M 段。
   * @param start_vel/end_vel 整条轨迹起末速度，[m/s]。
   * @param start_acc/end_acc 整条轨迹起末加速度，[m/s^2]。
   * @param Time 长度为 M 的各段持续时间，[s]。
   */
  static PolynomialTraj minSnapTraj(const Eigen::MatrixXd &Pos, const Eigen::Vector3d &start_vel,
                                    const Eigen::Vector3d &end_vel, const Eigen::Vector3d &start_acc,
                                    const Eigen::Vector3d &end_acc, const Eigen::VectorXd &Time);

  /// 生成仅含一段、满足完整起末位置/速度/加速度边界的多项式轨迹；t 为总时长 [s]。
  static PolynomialTraj one_segment_traj_gen(const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                             const Eigen::Vector3d &end_pt, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc,
                                             double t);
};

#endif
