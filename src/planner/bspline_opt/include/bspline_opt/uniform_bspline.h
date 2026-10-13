#ifndef _UNIFORM_BSPLINE_H_
#define _UNIFORM_BSPLINE_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>

using namespace std;

namespace ego_planner
{
  /**
   * @brief 任意维度的 B 样条轨迹及其求值、求导和动力学检查工具。
   *
   * 类名沿用原工程；内部 knot vector 可以被 setKnot() 改成非均匀形式。
   * 规划主流程通常传入 3×N 矩阵，每一列是一个三维位置控制点。构造函数内部
   * 会转置为本类使用的 N×3 布局，所以不要仅凭成员矩阵形状判断外部接口布局。
   */
  // An implementation of non-uniform B-spline with different dimensions
  // It also represents uniform B-spline which is a special case of non-uniform
  class UniformBspline
  {
  private:
    // control points for B-spline with different dimensions.
    // Each row represents one single control point
    // The dimension is determined by column number
    // e.g. B-spline with N points in 3D space -> Nx3 matrix
    Eigen::MatrixXd control_points_;

    int p_, n_, m_;     // p：次数；n+1：控制点数；m=n+p+1：最后一个 knot 的索引
    Eigen::VectorXd u_; // knot vector，长度 m+1，单位通常为秒
    double interval_;   // 均匀 knot span，即相邻节点时间间隔 delta t，[s]

    /// 根据控制点差分和 knot 间隔构造导数 B 样条的控制点。
    Eigen::MatrixXd getDerivativeControlPoints();

    // limit_ratio_ 记录可行性检查建议的时间放大倍率；tolerance 为允许超限余量。
    double limit_vel_, limit_acc_, limit_ratio_, feasibility_tolerance_; // physical limits and time adjustment ratio

  public:
    UniformBspline() {}
    /// @param points 外部布局为 dim×N，每列一个控制点；order 实际表示次数 p；interval 单位 s。
    UniformBspline(const Eigen::MatrixXd &points, const int &order, const double &interval);
    ~UniformBspline();

    Eigen::MatrixXd get_control_points(void) { return control_points_; }

    // initialize as an uniform B-spline
    void setUniformBspline(const Eigen::MatrixXd &points, const int &order, const double &interval);

    // get / set basic bspline info

    /// 用调用者给定 knot vector 覆盖默认均匀节点；长度必须与控制点数和次数匹配。
    void setKnot(const Eigen::VectorXd &knot);
    Eigen::VectorXd getKnot();
    Eigen::MatrixXd getControlPoint();
    double getInterval();
    /// 返回曲线有效参数区间 [u_p, u_{m-p}]，而非整个 knot vector 范围。
    bool getTimeSpan(double &um, double &um_p);

    // compute position / derivative

    /// 使用 De Boor 算法在 knot 参数 u 上求值；u 应落在有效参数区间内。
    Eigen::VectorXd evaluateDeBoor(const double &u);                                               // use u \in [up, u_mp]
    /// 使用从零开始的轨迹时间 t 求值，内部自动偏移到 u_p。
    inline Eigen::VectorXd evaluateDeBoorT(const double &t) { return evaluateDeBoor(t + u_(p_)); } // use t \in [0, duration]
    /// 返回当前曲线对时间的一阶导 B 样条；位置轨迹调用一次得到速度，再调用得到加速度。
    UniformBspline getDerivative();

    // 3D B-spline interpolation of points in point_set, with boundary vel&acc
    // constraints
    // input : (K+2) points with boundary vel/acc; ts
    // output: (K+6) control_pts
    /**
     * @brief 将离散路径样本拟合/插值参数化为三次均匀 B 样条控制点。
     * @param ts 相邻离散样本对应的时间间隔，[s]，不是空间点距。
     * @param point_set 希望曲线经过/逼近的三维位置样本，按时间顺序排列。
     * @param start_end_derivative 固定顺序为起点速度、终点速度、起点加速度、终点加速度。
     * @param ctrl_pts 输出 3×N 控制点矩阵，每一列为一个三维控制点。
     */
    static void parameterizeToBspline(const double &ts, const vector<Eigen::Vector3d> &point_set,
                                      const vector<Eigen::Vector3d> &start_end_derivative,
                                      Eigen::MatrixXd &ctrl_pts);

    /* check feasibility, adjust time */

    /// 设置逐轴速度、加速度上限及允许的相对容差；本函数不修改轨迹。
    void setPhysicalLimits(const double &vel, const double &acc, const double &tolerance);
    /// 检查导数控制点是否超限；ratio 输出建议的时间放大倍率，show 控制诊断输出。
    bool checkFeasibility(double &ratio, bool show = false);
    /// 通过放大 knot 时间轴让同一空间曲线走得更慢；ratio 应大于 1。
    void lengthenTime(const double &ratio);

    /* for performance evaluation */

    double getTimeSum();
    /// 以 res 秒为采样步长累加折线长度；res 越小越精确但计算越慢。
    double getLength(const double &res = 0.01);
    double getJerk();
    void getMeanAndMaxVel(double &mean_v, double &max_v);
    void getMeanAndMaxAcc(double &mean_a, double &max_a);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
} // namespace ego_planner
#endif
