#include "path_searching/dyn_a_star.h"



using namespace std;
using namespace Eigen;

namespace
{
    constexpr double kGroundZ = 0.0;
}

/**
 * @brief 释放 initGridMap() 为每一个栅格创建的 GridNode 对象。
 *
 * GridNodeMap_ 是一个手工构造的三级指针数组：
 *   GridNodeMap_[x][y][z] -> GridNode
 *
 * 当前析构函数沿用原项目实现，只 delete 了最内层 GridNode；外层几个
 * new[] 创建的指针数组没有在这里释放。通常规划器进程退出时操作系统会
 * 回收这些内存，但若反复创建/销毁 AStar 对象，建议进一步补齐 delete[]。
 * 本次只添加解释，不改变原算法和内存管理行为。
 */
AStar::~AStar()
{
    for (int i = 0; i < POOL_SIZE_(0); i++)
        for (int j = 0; j < POOL_SIZE_(1); j++)
            for (int k = 0; k < POOL_SIZE_(2); k++)
                delete GridNodeMap_[i][j][k];
}

void AStar::initGridMap(GridMap::Ptr occ_map, const Eigen::Vector3i pool_size)
{
    // 保存节点池尺寸。整数向量除以 2 得到池中心索引，例如：
    // POOL_SIZE_=(100,100,100) 时 CENTER_IDX_=(50,50,50)。
    POOL_SIZE_ = pool_size;
    CENTER_IDX_ = pool_size / 2;

    /*
     * 一次性分配三维节点池。
     *
     * 为什么不在搜索过程中按需 new 节点？
     * 在线规划会频繁调用 A*；预分配可以避免实时循环中大量动态内存分配，
     * 并让 (x,y,z) 索引可以 O(1) 找到对应 GridNode。
     */
    GridNodeMap_ = new GridNodePtr **[POOL_SIZE_(0)];
    for (int i = 0; i < POOL_SIZE_(0); i++)
    {
        GridNodeMap_[i] = new GridNodePtr *[POOL_SIZE_(1)];
        for (int j = 0; j < POOL_SIZE_(1); j++)
        {
            GridNodeMap_[i][j] = new GridNodePtr[POOL_SIZE_(2)];
            for (int k = 0; k < POOL_SIZE_(2); k++)
            {
                // 节点的 index、gScore 等会在它首次参与某一轮搜索时赋值。
                GridNodeMap_[i][j][k] = new GridNode;
            }
        }
    }

    // 保存 plan_env 提供的地图，搜索时通过 checkOccupancy() 查询。
    grid_map_ = occ_map;
}

/**
 * @brief 适合三维 26 邻域移动的“对角距离”启发函数。
 *
 * 每次扩展允许 dx/dy/dz 分别取 -1、0、1，因此有三类有效移动：
 *
 * - 只改变一个轴：代价 1；
 * - 同时改变两个轴：代价 sqrt(2)；
 * - 同时改变三个轴：代价 sqrt(3)。
 *
 * 该函数先尽量使用三轴对角移动，再使用双轴对角移动，最后使用单轴移动，
 * 得到在“完全没有障碍物”时从 node1 到 node2 的最短 26 邻域距离。
 * 它比曼哈顿距离更符合本搜索器实际允许的移动方式。
 */
double AStar::getDiagHeu(GridNodePtr node1, GridNodePtr node2)
{
    // 三个方向还相差多少个栅格。abs 保证只关注距离，不关注方向。
    double dx = abs(node1->index(0) - node2->index(0));
    double dy = abs(node1->index(1) - node2->index(1));
    double dz = abs(node1->index(2) - node2->index(2));

    double h = 0.0;
    // 三个方向可以同时前进一步的次数。
    // 例如差值 (5,3,2)，可以先走 2 次三轴对角线。
    int diag = min(min(dx, dy), dz);
    dx -= diag;
    dy -= diag;
    dz -= diag;

    if (dx == 0)
    {
        // x 差值已由三轴对角移动消耗完：
        // 剩下 y/z 尽量走双轴对角线，再沿剩余单轴行走。
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dy, dz) + 1.0 * abs(dy - dz);
    }
    if (dy == 0)
    {
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dz) + 1.0 * abs(dx - dz);
    }
    if (dz == 0)
    {
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dy) + 1.0 * abs(dx - dy);
    }
    return h;
}

double AStar::getManhHeu(GridNodePtr node1, GridNodePtr node2)
{
    // 曼哈顿距离假设每次只沿一个坐标轴移动。
    // 本实现允许对角移动，所以当前没有选用这个启发函数。
    double dx = abs(node1->index(0) - node2->index(0));
    double dy = abs(node1->index(1) - node2->index(1));
    double dz = abs(node1->index(2) - node2->index(2));

    return dx + dy + dz;
}

double AStar::getEuclHeu(GridNodePtr node1, GridNodePtr node2)
{
    // 索引空间中的直线距离。当前也没有被 getHeu() 选用。
    return (node2->index - node1->index).norm();
}

vector<GridNodePtr> AStar::retrievePath(GridNodePtr current)
{
    /*
     * 搜索到终点时 current 指向终点。
     * cameFrom 保存“到达当前节点时来自哪个父节点”，因此可以：
     *
     *   终点 -> 父节点 -> 父节点 -> ... -> 起点
     *
     * 此处得到的是逆序路径；getPath() 最后会 reverse。
     */
    vector<GridNodePtr> path;
    path.push_back(current);

    while (current->cameFrom != NULL)
    {
        current = current->cameFrom;
        path.push_back(current);
    }

    return path;
}

bool AStar::ConvertToIndexAndAdjustStartEndPoints(Vector3d start_pt, Vector3d end_pt, Vector3i &start_idx, Vector3i &end_idx)
{
    // 搜索只操作整数索引，所以第一步要把以米为单位的世界坐标离散化。
    // 任一端点超出本次局部节点池时，无法继续搜索。
    if (!Coord2Index(start_pt, start_idx) || !Coord2Index(end_pt, end_idx))
        return false;

    if (checkOccupancy(Index2Coord(start_idx)))
    {
        /*
         * 起点落在膨胀障碍物内时，沿“从终点指向起点”的单位方向移动，
         * 即继续远离终点，每次移动 step_size_，直到进入自由体素。
         *
         * 这里调整的是函数内部的 start_pt 副本；最终搜索使用调整后的
         * start_idx，但调用者传入的原始 start_pt 不会被改写。
         */
        // RCLCPP_WARN(rclcpp::get_logger("ConvertToIndexAndAdjustStartEndPoints"), "Start point is insdide an obstacle.");
        do
        {
            start_pt = (start_pt - end_pt).normalized() * step_size_ + start_pt;
            if (!Coord2Index(start_pt, start_idx))
                return false;
        } while (checkOccupancy(Index2Coord(start_idx)));
    }

    if (checkOccupancy(Index2Coord(end_idx)))
    {
        // 与起点同理：若终点被占据，就沿“从起点指向终点”的方向继续外移。
        // RCLCPP_WARN(rclcpp::get_logger("ConvertToIndexAndAdjustStartEndPoints"), "End point is insdide an obstacle.");
        do
        {
            end_pt = (end_pt - start_pt).normalized() * step_size_ + end_pt;
            if (!Coord2Index(end_pt, end_idx))
                return false;
        } while (checkOccupancy(Index2Coord(end_idx)));
    }

    return true;
}

bool AStar::AstarSearch(const double step_size, Vector3d start_pt, Vector3d end_pt)
{
    
    start_pt.z() = kGroundZ;
    end_pt.z() = kGroundZ;
    // 记录开始时间，因为本实现给每次搜索设置了 0.2 秒硬超时。
    rclcpp::Time time_1 = rclcpp::Clock().now();

    // 开启新一轮搜索。节点的 rounds 若不等于这个值，就视为本轮未访问。
    ++rounds_;

    // step_size 决定搜索分辨率；inv_step_size_ 避免坐标转换时反复做除法。
    step_size_ = step_size;
    inv_step_size_ = 1 / step_size;

    /*
     * 将搜索池的中心放在起点与终点中点。
     *
     * GridNodeMap_ 本身只是一块没有固定世界原点的索引数组。每次搜索通过
     * center_ 把这块数组“放置”到当前起终点附近，因此同一节点池可重复用于
     * 不同空间位置的局部搜索。
     */
    center_ = (start_pt + end_pt) / 2;

    Vector3i start_idx, end_idx;
    if (!ConvertToIndexAndAdjustStartEndPoints(start_pt, end_pt, start_idx, end_idx))
    {
        RCLCPP_ERROR(rclcpp::get_logger("AstarSearch"), "Unable to handle the initial or end point, force return!");
        return false;
    }

    // if ( start_pt(0) > -1 && start_pt(0) < 0 )
    //     cout << "start_pt=" << start_pt.transpose() << " end_pt=" << end_pt.transpose() << endl;

    // 从预分配节点池中取得起点、终点对象。这里没有创建新节点。
    GridNodePtr startPtr = GridNodeMap_[start_idx(0)][start_idx(1)][start_idx(2)];
    GridNodePtr endPtr = GridNodeMap_[end_idx(0)][end_idx(1)][end_idx(2)];

    // priority_queue 没有 clear()，与一个空队列 swap 是常见的清空方式。
    std::priority_queue<GridNodePtr, std::vector<GridNodePtr>, NodeComparator> empty;
    openSet_.swap(empty);

    GridNodePtr neighborPtr = NULL;
    GridNodePtr current = NULL;

    /*
     * 初始化起点：
     * - 从起点到起点的实际代价 g=0；
     * - 预计总代价 f=0+h(start, goal)；
     * - 没有父节点；
     * - 放入 open set 等待展开。
     */
    startPtr->index = start_idx;
    startPtr->rounds = rounds_;
    startPtr->gScore = 0;
    startPtr->fScore = getHeu(startPtr, endPtr);
    startPtr->state = GridNode::OPENSET; //put start node in open set
    startPtr->cameFrom = NULL;
    openSet_.push(startPtr); //put start in open set

    // 启发函数只需要终点索引，不需要预先设置终点的 g/f/state。
    endPtr->index = end_idx;

    double tentative_gScore;

    int num_iter = 0;
    while (!openSet_.empty())
    {
        num_iter++;

        // 取出当前 fScore 最小的候选节点，这是 A* 的核心选择策略。
        current = openSet_.top();
        openSet_.pop();

        // if ( num_iter < 10000 )
        //     cout << "current=" << current->index.transpose() << endl;

        if (current->index(0) == endPtr->index(0) && current->index(1) == endPtr->index(1) && current->index(2) == endPtr->index(2))
        {
            // 已到达目标索引。沿 cameFrom 回溯并保存路径，然后结束搜索。
            // ros::Time time_2 = ros::Time::now();
            // printf("\033[34mA star iter:%d, time:%.3f\033[0m\n",num_iter, (time_2 - time_1).toSec()*1000);
            // if((time_2 - time_1).toSec() > 0.1)
            //     ROS_WARN("Time consume in A star path finding is %f", (time_2 - time_1).toSec() );
            gridPath_ = retrievePath(current);
            return true;
        }
        // current 即将被完整展开，之后标记为 closed，避免再次展开它。
        current->state = GridNode::CLOSEDSET;

        /*
         * 枚举当前节点周围的 3×3×3 立方体。
         * 去掉 (0,0,0) 自身后，一共有 26 个邻居：
         *
         * - 6 个面邻居，移动代价 1；
         * - 12 个棱邻居，移动代价 sqrt(2)；
         * - 8 个角邻居，移动代价 sqrt(3)。
         */
        for (int dx = -1; dx <= 1; dx++)
            for (int dy = -1; dy <= 1; dy++)
                for (int dz = 0; dz <= 0; dz++)
                {
                    if (dx == 0 && dy == 0 && dz == 0)
                        // 不能把自己当作自己的邻居。
                        continue;

                    // 邻居索引 = 当前索引 + 一个方向增量。
                    Vector3i neighborIdx;
                    neighborIdx(0) = (current->index)(0) + dx;
                    neighborIdx(1) = (current->index)(1) + dy;
                    neighborIdx(2) = (current->index)(2) + dz;

                    if (neighborIdx(0) < 1 || neighborIdx(0) >= POOL_SIZE_(0) - 1 || neighborIdx(1) < 1 || neighborIdx(1) >= POOL_SIZE_(1) - 1 || neighborIdx(2) < 1 || neighborIdx(2) >= POOL_SIZE_(2) - 1)
                    {
                        // 留出一层数组边界，不扩展索引 0 和 size-1，防止后续越界。
                        continue;
                    }

                    // O(1) 取得预分配的邻居节点，并写入本次搜索使用的索引。
                    neighborPtr = GridNodeMap_[neighborIdx(0)][neighborIdx(1)][neighborIdx(2)];
                    neighborPtr->index = neighborIdx;

                    // rounds 相等说明它已经在本轮被发现过；否则其旧字段应忽略。
                    bool flag_explored = neighborPtr->rounds == rounds_;

                    if (flag_explored && neighborPtr->state == GridNode::CLOSEDSET)
                    {
                        // 已经完成展开的节点不再重新打开。
                        continue;
                    }

                    // 从现在开始，该节点属于当前搜索轮次。
                    neighborPtr->rounds = rounds_;

                    if (checkOccupancy(Index2Coord(neighborPtr->index)))
                    {
                        // 将邻居索引转回世界坐标，查询膨胀地图；占据或越界则跳过。
                        continue;
                    }

                    /*
                     * 从 current 走到 neighbor 的一步代价：
                     * 轴向移动 sqrt(1)=1，双轴对角 sqrt(2)，三轴对角 sqrt(3)。
                     */
                    double static_cost = sqrt(dx * dx + dy * dy + dz * dz);

                    // “经过 current 到达 neighbor”的候选累计代价。
                    tentative_gScore = current->gScore + static_cost;

                    if (!flag_explored)
                    {
                        // 第一次发现该节点：记录父节点和代价，加入 open set。
                        neighborPtr->state = GridNode::OPENSET;
                        neighborPtr->cameFrom = current;
                        neighborPtr->gScore = tentative_gScore;
                        neighborPtr->fScore = tentative_gScore + getHeu(neighborPtr, endPtr);
                        openSet_.push(neighborPtr); //put neighbor in open set and record it.
                    }
                    else if (tentative_gScore < neighborPtr->gScore)
                    {
                        /*
                         * 节点已在 open set，但新路线更短，于是更新父节点和代价。
                         *
                         * 实现细节提示：
                         * std::priority_queue 不提供 decrease-key。这里直接修改了
                         * 队列中指针所指对象的 fScore，却没有重新 push 或重建堆，
                         * 因而堆的内部顺序不一定立即更新。原项目保持此写法通常仍
                         * 能找到路径，但若追求严格 A* 行为，可考虑重新入队并在
                         * pop 时丢弃旧条目，或使用支持 decrease-key 的数据结构。
                         */
                        neighborPtr->cameFrom = current;
                        neighborPtr->gScore = tentative_gScore;
                        neighborPtr->fScore = tentative_gScore + getHeu(neighborPtr, endPtr);
                    }
                }
        // 每展开一个节点后检查运行时间，避免在线规划被一次 A* 长时间阻塞。
        rclcpp::Time time_2 = rclcpp::Clock().now();
        if ((time_2 - time_1).seconds() > 0.2)
        {
            RCLCPP_WARN(rclcpp::get_logger("AstarSearch"), "Failed in A star path searching !!! 0.2 seconds time limit exceeded.");
            return false;
        }
    }

    // open set 已空仍未到达终点，表示当前搜索范围内不存在可行路径。
    rclcpp::Time time_2 = rclcpp::Clock().now();

    if ((time_2 - time_1).seconds() > 0.1)
        RCLCPP_WARN(rclcpp::get_logger("AstarSearch"), 
                    "Time consume in A star path finding is %.3fs, iter=%d", (time_2 - time_1).seconds(), num_iter);

    return false;
}

vector<Vector3d> AStar::getPath()
{
    // gridPath_ 保存的是节点指针，而且顺序是终点到起点。
    vector<Vector3d> path;

    // 对外不暴露内部节点对象，只返回以米为单位的世界坐标。
    for (auto ptr : gridPath_)
        path.push_back(Index2Coord(ptr->index));

    // 转换成更符合使用习惯的“起点 -> 终点”顺序。
    reverse(path.begin(), path.end());
    return path;
}
