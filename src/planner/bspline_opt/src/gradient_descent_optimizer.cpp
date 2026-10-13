#include <bspline_opt/gradient_descent_optimizer.h>

#define RESET "\033[0m"
#define RED "\033[31m"

GradientDescentOptimizer::RESULT
GradientDescentOptimizer::optimize(Eigen::VectorXd &x_init_optimal, double &opt_f)
{
    // 入口先检查会直接破坏终止判据的配置。这里不检查目标函数指针和向量维数，
    // 因而调用者必须遵守头文件中的接口约定。
    if (min_grad_ < 1e-10)
    {
        cout << RED << "min_grad_ is invalid:" << min_grad_ << RESET << endl;
        return FAILED;
    }
    if (iter_limit_ <= 2)
    {
        cout << RED << "iter_limit_ is invalid:" << iter_limit_ << RESET << endl;
        return FAILED;
    }

    void *f_data = f_data_; // 目标函数上下文的局部别名，本函数不释放它。
    int iter = 2;           // 前两个试探点在主 while 之前计算，因此从 2 开始计数。
    int invoke_count = 2;   // 与前两次 objfun_ 调用对应的函数评估计数。
    bool force_return;      // 由 objfun_ 写入；true 表示调用者要求有序提前结束。
    Eigen::VectorXd x_k(x_init_optimal), x_kp1(x_init_optimal.rows()); // 相邻两次迭代点 x_k、x_(k+1)。
    double cost_k, cost_kp1, cost_min; // 两个迭代点的代价和运行中见过的最小代价。
    Eigen::VectorXd grad_k(x_init_optimal.rows()), grad_kp1(x_init_optimal.rows()); // 对应梯度。

    cost_k = objfun_(x_k, grad_k, force_return, f_data);
    if (force_return)
        return RETURN_BY_ORDER;
    cost_min = cost_k;
    // max_grad 是梯度各分量绝对值的无穷范数。alpha0 限制第一次更新中任一
    // 决策变量最多移动 0.1；对位置型优化变量而言单位通常是米。
    double max_grad = max(abs(grad_k.maxCoeff()), abs(grad_k.minCoeff()));
    constexpr double MAX_MOVEMENT_AT_FIRST_ITERATION = 0.1; // meter
    double alpha0 = max_grad < MAX_MOVEMENT_AT_FIRST_ITERATION ? 1.0 : (MAX_MOVEMENT_AT_FIRST_ITERATION / max_grad);
    x_kp1 = x_k - alpha0 * grad_k;
    cost_kp1 = objfun_(x_kp1, grad_kp1, force_return, f_data);
    if (force_return)
        return RETURN_BY_ORDER;
    if (cost_min > cost_kp1)
        cost_min = cost_kp1;

    /*** start iteration ***/
    while (++iter <= iter_limit_ && invoke_count <= invoke_limit_)
    {
        // s 是最近一次变量位移，y 是对应梯度变化。s^T y / y^T y 是
        // Barzilai-Borwein 步长估计，用局部曲率信息比固定学习率更快地选步长。
        Eigen::VectorXd s = x_kp1 - x_k;
        Eigen::VectorXd y = grad_kp1 - grad_k;
        double alpha = s.dot(y) / y.dot(y);
        if (isnan(alpha) || isinf(alpha))
        {
            cout << RED << "step size invalid! alpha=" << alpha << RESET << endl;
            return FAILED;
        }

        // 奇偶迭代交替覆盖 x_k 和 x_kp1，避免每轮复制整条变量向量。
        if (iter % 2) // to aviod copying operations
        {
            do
            {
                x_k = x_kp1 - alpha * grad_kp1;
                cost_k = objfun_(x_k, grad_k, force_return, f_data);
                invoke_count++;
                if (force_return)
                    return RETURN_BY_ORDER;
                // 回溯线搜索：若不满足下面的 Armijo 充分下降条件，步长减半重试。
                alpha *= 0.5;
            } while (cost_k > cost_kp1 - 1e-4 * alpha * grad_kp1.transpose() * grad_kp1); // Armijo condition

            if (grad_k.norm() < min_grad_)
            {
                opt_f = cost_k;
                return FIND_MIN;
            }
        }
        else
        {
            do
            {
                x_kp1 = x_k - alpha * grad_k;
                cost_kp1 = objfun_(x_kp1, grad_kp1, force_return, f_data);
                invoke_count++;
                if (force_return)
                    return RETURN_BY_ORDER;
                // 与奇数分支相同，只是当前点和下一点的缓存角色互换。
                alpha *= 0.5;
            } while (cost_kp1 > cost_k - 1e-4 * alpha * grad_k.transpose() * grad_k); // Armijo condition

            if (grad_kp1.norm() < min_grad_)
            {
                opt_f = cost_kp1;
                return FIND_MIN;
            }
        }
    }

    // 到达上限时返回最后一次有效迭代的目标值；奇偶性决定它存在哪个缓存中。
    opt_f = iter_limit_ % 2 ? cost_k : cost_kp1;
    return REACH_MAX_ITERATION;
}
