#ifndef _DYN_A_STAR_H_
#define _DYN_A_STAR_H_

#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Eigen>
#include <plan_env/grid_map.h>
#include <queue>

/*
 * 本文件定义一个用于三维占据栅格的 A* 搜索器。
 *
 * 建议先记住 A* 的核心公式：
 *
 *   f(n) = g(n) + h(n)
 *
 * g(n)：从起点走到节点 n 已经付出的真实代价；
 * h(n)：从节点 n 到终点还需要多少代价的估计值（启发函数）；
 * f(n)：经过节点 n 到达终点的预计总代价。
 *
 * 算法每次从 open set 中取出 f 最小的节点，并检查它的相邻栅格。
 * 找到终点后，再沿 cameFrom 指针从终点逐步回到起点。
 *
 * 注意：这里搜索得到的是离散栅格折线，主要给 B 样条优化器提供绕障方向，
 * 并不是最终直接发送给无人机控制器的平滑轨迹。
 */

// 用一个足够大的数初始化尚未访问节点的代价。
// 1 << 20 等于 1,048,576。这里保留原代码写法，不表示数学上的真正无穷大。
constexpr double inf = 1 >> 20;

// 先声明 GridNode，随后才能定义“指向 GridNode 的指针类型”。
struct GridNode;
typedef GridNode *GridNodePtr;

/**
 * @brief A* 搜索中的一个三维栅格节点。
 *
 * 每个节点对应搜索池 GridNodeMap_ 中的一个整数索引 (ix, iy, iz)。
 * 节点对象会在多次 A* 调用之间复用，rounds 字段用于判断它在本轮是否有效。
 */
struct GridNode
{
	// 节点在“本轮搜索”中的状态。
	enum enum_state
	{
		OPENSET = 1,   ///< 已经发现，等待或仍可能等待进一步展开。
		CLOSEDSET = 2, ///< 已经从优先队列取出并展开，不再重复处理。
		UNDEFINED = 3  ///< 尚未参与搜索。
	};

	/*
	 * 本节点最近一次被哪一轮搜索访问。
	 *
	 * AStar::rounds_ 每调用一次 AstarSearch() 就加一。若：
	 *     node->rounds == rounds_
	 * 说明节点已经在当前搜索中出现过；否则节点中的旧 gScore、state 等数据
	 * 属于上一次搜索，可以直接用本轮的新数据覆盖。
	 *
	 * 这种做法避免每次搜索前遍历整个三维节点池进行清零。
	 */
	int rounds{0};

	enum enum_state state
	{
		UNDEFINED
	};

	// 节点在 GridNodeMap_ 中的整数栅格索引，不是以米为单位的世界坐标。
	Eigen::Vector3i index;

	// gScore：起点到本节点的累计真实代价。
	// fScore：gScore + 启发式估价，是优先队列排序依据。
	double gScore{inf};
	double fScore{inf};

	// 当前已知最优路径中，本节点的父节点。找到终点后靠它反向恢复路径。
	GridNodePtr cameFrom{NULL};
};

/**
 * @brief priority_queue 的比较器，使 fScore 较小的节点位于队首。
 *
 * C++ 的 std::priority_queue 默认是“大顶堆”，默认会先弹出较大的元素。
 * 这里使用 node1->fScore > node2->fScore，相当于把它变成按 fScore 排序的
 * “小顶堆”，从而让 A* 总是优先展开预计总代价最小的节点。
 */
class NodeComparator
{
public:
	bool operator()(GridNodePtr node1, GridNodePtr node2)
	{
		return node1->fScore > node2->fScore;
	}
};

/**
 * @brief 面向局部规划的三维栅格 A* 搜索器。
 *
 * 使用方法：
 *
 *   AStar::Ptr astar = std::make_shared<AStar>();
 *   astar->initGridMap(grid_map, Eigen::Vector3i(100, 100, 100));
 *   if (astar->AstarSearch(0.1, start, goal))
 *     std::vector<Eigen::Vector3d> path = astar->getPath();
 *
 * 搜索池以 start 和 goal 的中点为中心，POOL_SIZE_ 决定最大可搜索索引范围，
 * step_size 决定相邻搜索点在世界坐标中的距离。
 */
class AStar
{
private:
	// plan_env 提供的占据栅格。A* 不拥有地图，只保存 shared_ptr 并查询它。
	GridMap::Ptr grid_map_;

	// 旧版本遗留的快速坐标转换声明，当前文件中没有实现，也没有被调用。
	inline void coord2gridIndexFast(const double x, const double y, const double z, int &id_x, int &id_y, int &id_z);

	// 三种启发函数。当前 getHeu() 实际选择 getDiagHeu()。
	double getDiagHeu(GridNodePtr node1, GridNodePtr node2);
	double getManhHeu(GridNodePtr node1, GridNodePtr node2);
	double getEuclHeu(GridNodePtr node1, GridNodePtr node2);
	inline double getHeu(GridNodePtr node1, GridNodePtr node2);

	/**
	 * @brief 把起终点世界坐标转成搜索池索引，并处理落在障碍物内的端点。
	 *
	 * 若起点或终点所在栅格已被占据，就沿远离另一端的方向每次移动一个
	 * step_size_，直到找到自由栅格；若移动后超出节点池，则返回 false。
	 */
	bool ConvertToIndexAndAdjustStartEndPoints(const Eigen::Vector3d start_pt, const Eigen::Vector3d end_pt, Eigen::Vector3i &start_idx, Eigen::Vector3i &end_idx);

	// 搜索池整数索引 <-> 世界坐标（米）之间的转换。
	inline Eigen::Vector3d Index2Coord(const Eigen::Vector3i &index) const;
	inline bool Coord2Index(const Eigen::Vector3d &pt, Eigen::Vector3i &idx) const;

	//bool (*checkOccupancyPtr)( const Eigen::Vector3d &pos );

	// 查询“膨胀后的占据地图”。障碍膨胀已经把无人机尺寸和安全余量考虑进去。
	// GridMap::getInflateOccupancy() 返回 1 表示占据、0 表示空闲、-1 表示越界。
	// 强制转换为 bool 后 1 和 -1 都是 true，因此地图外也会被当作不可通行。
	inline bool checkOccupancy(const Eigen::Vector3d &pos) { return (bool)grid_map_->getInflateOccupancy(pos); }

	// 从终点沿 cameFrom 反向回溯到起点。返回顺序暂时是“终点 -> 起点”。
	std::vector<GridNodePtr> retrievePath(GridNodePtr current);

	double step_size_;     ///< 栅格步长，单位为米。
	double inv_step_size_; ///< 1 / step_size_，用于世界坐标快速转换为索引。

	// center_ 是本次起点和终点的几何中点，对应搜索池中的 CENTER_IDX_。
	Eigen::Vector3d center_;
	Eigen::Vector3i CENTER_IDX_;
	Eigen::Vector3i POOL_SIZE_; ///< 三个方向的节点池大小，例如 (100,100,100)。

	/*
	 * 很小的 tie breaker（平局打破系数）。
	 *
	 * 启发值乘以略大于 1 的系数，会让搜索稍微更倾向于靠近终点，
	 * 通常能减少展开节点数量，但严格来说会轻微破坏标准 A* 启发函数的
	 * 可采纳性，因此最优性会让位于搜索速度。
	 */
	const double tie_breaker_ = 1.0 + 1.0 / 10000;

	// 搜索成功后保存“终点 -> 起点”的节点序列，getPath() 再将它反转。
	std::vector<GridNodePtr> gridPath_;

	// 预分配的三维节点池：GridNodeMap_[x][y][z]。
	// 搜索时只重写本轮访问到的节点，避免频繁 new/delete。
	GridNodePtr ***GridNodeMap_;

	// open set：按 fScore 从小到大取节点的优先队列。
	std::priority_queue<GridNodePtr, std::vector<GridNodePtr>, NodeComparator> openSet_;

	// 全局搜索轮次编号。每开始一次新搜索就递增。
	int rounds_{0};

public:
	typedef std::shared_ptr<AStar> Ptr;

	AStar(){};
	~AStar();

	/**
	 * @brief 绑定占据地图并一次性分配三维节点池。
	 * @param occ_map plan_env 维护的膨胀占据地图。
	 * @param pool_size 节点池在 x/y/z 三个方向的栅格数量。
	 *
	 * 该函数应在第一次 AstarSearch() 之前调用，并且正常使用中只调用一次。
	 */
	void initGridMap(GridMap::Ptr occ_map, const Eigen::Vector3i pool_size);

	/**
	 * @brief 在 start_pt 与 end_pt 之间执行一次三维 A* 搜索。
	 * @param step_size 搜索分辨率（相邻轴向节点间距，单位米）。
	 * @param start_pt 起点世界坐标。
	 * @param end_pt 终点世界坐标。
	 * @return 在 0.2 秒限制内找到路径时返回 true。
	 */
	bool AstarSearch(const double step_size, Eigen::Vector3d start_pt, Eigen::Vector3d end_pt);

	/// 将上一次成功搜索的节点路径转成世界坐标，并按“起点 -> 终点”返回。
	std::vector<Eigen::Vector3d> getPath();
};

inline double AStar::getHeu(GridNodePtr node1, GridNodePtr node2)
{
	// 当前使用适合 26 邻域的三维对角距离，并乘一个极小 tie breaker。
	return tie_breaker_ * getDiagHeu(node1, node2);
}

inline Eigen::Vector3d AStar::Index2Coord(const Eigen::Vector3i &index) const
{
	/*
	 * CENTER_IDX_ 对应世界坐标 center_。
	 *
	 * 例如 step_size_=0.1，index 比 CENTER_IDX_ 的 x 大 3，
	 * 则转换后的世界坐标 x 比 center_.x() 大 3×0.1=0.3 米。
	 */
	return ((index - CENTER_IDX_).cast<double>() * step_size_) + center_;
};

inline bool AStar::Coord2Index(const Eigen::Vector3d &pt, Eigen::Vector3i &idx) const
{
	/*
	 * 先计算点相对 center_ 偏移了多少个 step_size_，加 0.5 后转 int，
	 * 相当于把正方向坐标舍入到最近栅格，再平移到 CENTER_IDX_。
	 *
	 * 注意：Eigen 的 cast<int>() 是截断而不是严格的 floor/round；
	 * 这里保留原项目的转换方式。
	 */
	idx = ((pt - center_) * inv_step_size_ + Eigen::Vector3d(0.5, 0.5, 0.5)).cast<int>() + CENTER_IDX_;

	// 节点池只是围绕本次起终点分配的有限局部数组，不等同于整张 GridMap。
	if (idx(0) < 0 || idx(0) >= POOL_SIZE_(0) || idx(1) < 0 || idx(1) >= POOL_SIZE_(1) || idx(2) < 0 || idx(2) >= POOL_SIZE_(2))
	{
		RCLCPP_ERROR(rclcpp::get_logger("Coord2Index"), "Ran out of pool, index=%d %d %d, POOL_SIZE=%d %d %d", idx(0), idx(1), idx(2),POOL_SIZE_(0), POOL_SIZE_(1), POOL_SIZE_(2));
		return false;
	}

	return true;
};

#endif
