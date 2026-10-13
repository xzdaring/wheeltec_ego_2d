#ifndef RAYCAST_H_
#define RAYCAST_H_

#include <Eigen/Eigen>
#include <vector>

/**
 * @file raycast.h
 * @brief 三维体素栅格上的射线遍历工具。
 *
 * 输入坐标在调用者的连续栅格坐标系中，输出为射线依次穿过的体素索引。
 * GridMap 用它把相机到深度点之间的体素标为空闲，并把射线末端标为占据。
 */

/// 返回 x 的符号：负数为 -1，正数为 1，零为 0。
double signum(double x);

/// 数学意义上的非负模运算，用于处理负坐标；与 C++ 浮点 fmod 的负数行为不同。
double mod(double value, double modulus);

/// 从坐标 s 沿方向 ds 前进，首次到达整数体素边界所需的参数距离。
double intbound(double s, double ds);

/**
 * @brief 一次性输出 start 到 end 穿过的体素。
 * @param min/max 允许输出的栅格坐标包围盒。
 * @param output_points_cnt 实际写入 output 的点数。
 * @param output 调用者预分配的输出数组，容量必须足够容纳整条射线。
 */
void Raycast(const Eigen::Vector3d& start, const Eigen::Vector3d& end, const Eigen::Vector3d& min,
             const Eigen::Vector3d& max, int& output_points_cnt, Eigen::Vector3d* output);

/// 与上一个重载算法相同，但由 vector 动态保存所有输出体素。
void Raycast(const Eigen::Vector3d& start, const Eigen::Vector3d& end, const Eigen::Vector3d& min,
             const Eigen::Vector3d& max, std::vector<Eigen::Vector3d>* output);

/**
 * @brief 可逐步迭代的 3D DDA/Amanatides-Woo 射线遍历器。
 *
 * setInput() 预计算每个轴的步进方向和下一边界距离；随后反复调用 step()，
 * 每次得到一个体素坐标，返回 false 表示已经越过射线终点。
 */
class RayCaster {
private:
  /* data */
  Eigen::Vector3d start_;     ///< 连续栅格坐标中的射线起点。
  Eigen::Vector3d end_;       ///< 连续栅格坐标中的射线终点。
  Eigen::Vector3d direction_; ///< end-start 的方向向量。
  Eigen::Vector3d min_;       ///< 预留的裁剪下界。
  Eigen::Vector3d max_;       ///< 预留的裁剪上界。
  int x_, y_, z_;             ///< 当前所在体素的整数索引。
  int endX_, endY_, endZ_;    ///< 终点所在体素索引。
  double maxDist_;            ///< 射线允许遍历的最大参数距离。
  double dx_, dy_, dz_;       ///< 方向向量三个轴分量的绝对值/缓存量。
  int stepX_, stepY_, stepZ_; ///< 每跨过一个轴边界时索引增加 -1、0 或 1。
  double tMaxX_, tMaxY_, tMaxZ_;       ///< 从起点到各轴下一个体素边界的参数距离。
  double tDeltaX_, tDeltaY_, tDeltaZ_; ///< 沿各轴再跨一个完整体素所增加的参数距离。
  double dist_;               ///< 当前已经推进到的参数距离。

  int step_num_;              ///< 已输出体素数量，用于首点和终止控制。

public:
  RayCaster(/* args */) {
  }
  ~RayCaster() {
  }

  /// 初始化一条新射线；start/end 使用连续栅格坐标而不是世界坐标米制值。
  bool setInput(const Eigen::Vector3d& start,
                const Eigen::Vector3d& end /* , const Eigen::Vector3d& min,
                const Eigen::Vector3d& max */);

  /// 输出下一个穿过体素的坐标；成功输出返回 true，遍历结束返回 false。
  bool step(Eigen::Vector3d& ray_pt);
};

#endif  // RAYCAST_H_
