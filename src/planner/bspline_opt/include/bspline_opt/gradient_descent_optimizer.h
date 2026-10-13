#ifndef _GRADIENT_DESCENT_OPT_H_
#define _GRADIENT_DESCENT_OPT_H_

#include <iostream>
#include <vector>
#include <Eigen/Eigen>

using namespace std;

/**
 * @brief 一个轻量级的一阶梯度下降封装。
 *
 * 优化变量统一展开为 Eigen::VectorXd。目标函数同时返回目标值和梯度，并可通过
 * force_return 要求优化器提前结束。该类与主规划器使用的 LBFGS 是两套实现，
 * 主要保留给实验性或较简单的优化问题。
 */
class GradientDescentOptimizer
{

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  /**
   * @brief 目标函数回调类型。
   * @param x 当前优化变量，只读。
   * @param grad 输出与 x 同维的目标梯度。
   * @param force_return 回调置 true 时要求 optimize() 立即按指令退出。
   * @param data 调用者自定义上下文，对应构造函数的 f_data。
   * @return 当前 x 对应的标量目标函数值。
   */
  typedef double (*objfunDef)(const Eigen::VectorXd &x, Eigen::VectorXd &grad, bool &force_return, void *data);

  /// optimize() 的终止原因；调用者可据此区分正常收敛、失败和达到迭代上限。
  enum RESULT
  {
    FIND_MIN,
    FAILED,
    RETURN_BY_ORDER,
    REACH_MAX_ITERATION
  };

  /**
   * @param v_num 优化变量维数，必须与传给 optimize() 的向量长度一致。
   * @param objf 计算目标值和梯度的回调函数。
   * @param f_data 原样转交给 objf 的用户上下文指针；本类不拥有其内存。
   */
  GradientDescentOptimizer(int v_num, objfunDef objf, void *f_data)
  {
    variable_num_ = v_num;
    objfun_ = objf;
    f_data_ = f_data;
  };

  void set_maxiter(int limit) { iter_limit_ = limit; }       ///< 最大梯度更新次数。
  void set_maxeval(int limit) { invoke_limit_ = limit; }     ///< 最大目标函数调用次数。
  void set_xtol_rel(double xtol_rel) { xtol_rel_ = xtol_rel; } ///< 变量相对变化收敛阈值。
  void set_xtol_abs(double xtol_abs) { xtol_abs_ = xtol_abs; } ///< 变量绝对变化收敛阈值。
  void set_min_grad(double min_grad) { min_grad_ = min_grad; } ///< 梯度范数收敛阈值。

  /// 从 x_init_optimal 给出的初值开始优化；成功或提前结束后该向量保存最后解，opt_f 保存目标值。
  RESULT optimize(Eigen::VectorXd &x_init_optimal, double &opt_f);

private:
  int variable_num_{0};                         ///< 决策变量维数。
  int iter_limit_{static_cast<int>(1e10)};      ///< 梯度更新次数上限。
  int invoke_limit_{static_cast<int>(1e10)};    ///< objfun_ 调用次数上限。
  double xtol_rel_;                             ///< 相邻解的相对变化阈值。
  double xtol_abs_;                             ///< 相邻解的绝对变化阈值。
  double min_grad_;                             ///< 梯度足够小时的停止阈值。
  double time_limit_;                           ///< 预留的运行时间上限字段。
  void *f_data_;                                ///< 用户上下文的非拥有指针。
  objfunDef objfun_;                            ///< 目标值/梯度回调。
};

#endif
