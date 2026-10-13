# `path_searching` 初学者导读

## 1. 这个包在整个系统中的位置

`path_searching` 实现三维占据栅格上的 A* 搜索。

它的输入是：

- 一个起点世界坐标；
- 一个终点世界坐标；
- `plan_env::GridMap` 提供的膨胀占据地图；
- 搜索步长。

它的输出是一系列无碰撞世界坐标点：

```text
起点 -> 栅格点 -> 栅格点 -> ... -> 终点
```

这些点构成离散折线，不是无人机最终直接执行的轨迹。`bspline_opt` 主要在
初始控制点穿过障碍物时调用 A*，根据这条折线判断从障碍物哪一侧绕行，
然后继续优化出平滑且满足动力学约束的 B 样条。

## 2. A* 需要记住的三个数

对于每个节点 `n`：

```text
g(n) = 从起点到 n 已经走过的真实代价
h(n) = 从 n 到终点还需要的估计代价
f(n) = g(n) + h(n)
```

代码中的对应关系：

```cpp
node->gScore
getHeu(node, endPtr)
node->fScore
```

A* 总是优先尝试 `fScore` 最小的节点。

如果只使用 `gScore` 而不使用 `h`，算法接近 Dijkstra，会向四周均匀扩散；
加入 `h` 后，搜索会更有方向性地朝终点前进。

## 3. open set 和 closed set

### open set

已经发现、但还没有完全检查其邻居的候选节点。

本项目使用：

```cpp
std::priority_queue<GridNodePtr, ..., NodeComparator> openSet_;
```

它按 `fScore` 排序。

### closed set

已经从 open set 取出，并完成了邻居展开的节点。代码没有单独创建 closed
容器，而是在节点上记录：

```cpp
node->state = GridNode::CLOSEDSET;
```

## 4. 为什么要有 `cameFrom`

搜索过程中，每当发现“经过 current 到达 neighbor 更好”时，就记录：

```cpp
neighbor->cameFrom = current;
```

假设最后找到终点：

```text
起点 A -> B -> C -> 终点 D
```

节点中保存的是：

```text
D.cameFrom = C
C.cameFrom = B
B.cameFrom = A
A.cameFrom = nullptr
```

因此从 D 不断访问 `cameFrom` 就可以回到 A。

## 5. 世界坐标与搜索索引

世界坐标以米为单位，例如：

```text
(2.3, -1.1, 1.0) m
```

搜索索引是三维整数：

```text
(53, 39, 50)
```

`Coord2Index()` 把世界坐标离散到节点池；`Index2Coord()` 把节点索引恢复成
世界坐标。

本项目并不让 A* 直接使用 `GridMap` 的体素索引。它另外维护一个以起终点
中点为中心的局部搜索池：

```text
世界坐标 center_ <-> 搜索索引 CENTER_IDX_
```

若步长是 `0.1 m`，索引在 x 方向增加 3，就表示世界坐标 x 增加 `0.3 m`。

## 6. 26 邻域

三层循环：

```cpp
for (int dx = -1; dx <= 1; dx++)
  for (int dy = -1; dy <= 1; dy++)
    for (int dz = -1; dz <= 1; dz++)
```

会生成 `3 × 3 × 3 = 27` 种组合。去掉 `(0,0,0)` 自己后，还剩 26 个邻居：

| 类型 | 数量 | 一步代价 |
|---|---:|---:|
| 面邻居，只改变一个轴 | 6 | `1` |
| 棱邻居，同时改变两个轴 | 12 | `sqrt(2)` |
| 角邻居，同时改变三个轴 | 8 | `sqrt(3)` |

真实空间中的距离还应乘 `step_size`，但因为整次搜索的 `step_size` 相同，
本代码直接使用索引空间代价，不影响路径之间的大小比较。

## 7. 一次搜索的完整流程

```text
调用 AstarSearch()
  ↓
搜索轮次 rounds_ 加一
  ↓
保存 step_size，并把节点池中心放到起终点中点
  ↓
起终点世界坐标转换为搜索索引
  ↓
若端点落在障碍物中，将它向外移动到自由位置
  ↓
清空 open set
  ↓
设置起点 g=0、f=h，放入 open set
  ↓
循环：取出 f 最小节点 current
  ├─ current 是终点：沿 cameFrom 回溯，成功
  └─ 否则枚举 26 个邻居
       ├─ 超出搜索池：忽略
       ├─ 已在 closed set：忽略
       ├─ 地图中被占据：忽略
       ├─ 第一次发现：记录代价和父节点，加入 open set
       └─ 已发现但新路线更短：更新代价和父节点
  ↓
open set 为空：当前范围内无路可走，失败
```

另外，每次搜索超过 0.2 秒也会直接失败，避免阻塞在线规划。

## 8. `rounds` 为什么存在

节点池可能包含 `100 × 100 × 100 = 1,000,000` 个节点。

如果每次搜索前都把所有节点的状态和代价清空，会产生很大开销。因此每个
节点记录它最后参与的搜索轮次：

```cpp
node->rounds
```

搜索器记录当前轮次：

```cpp
AStar::rounds_
```

只有两者相等，节点里的 `state/gScore/fScore` 才属于本轮搜索。否则可以把
旧值忽略，并在首次访问时覆盖。

## 9. 启发函数

本项目提供：

- `getManhHeu()`：曼哈顿距离；
- `getEuclHeu()`：欧氏直线距离；
- `getDiagHeu()`：三维对角距离。

实际使用 `getDiagHeu()`，因为搜索允许 26 邻域移动，对角距离与实际移动
模型更一致。

启发值还乘了：

```cpp
1.0 + 1.0 / 10000
```

这会让搜索稍微偏向终点，减少一些无意义展开。代价是它不再是完全标准的
最优 A*，但对在线规划通常是可接受的速度取舍。

## 10. 与地图的关系

搜索器通过：

```cpp
grid_map_->getInflateOccupancy(pos)
```

查询膨胀后的地图。

返回值含义：

```text
0  空闲
1  占据
-1 地图范围外
```

代码强制转换成 `bool`，所以 `1` 和 `-1` 都会被当作不可通行。也就是说，
A* 不会主动搜索到地图外。

## 11. 值得注意的原实现细节

以下是阅读代码时应知道的实现特征，本次注释没有改变它们：

1. `priority_queue` 不支持 decrease-key。代码在找到更短路径时直接修改
   队列元素指向节点的 `fScore`，但没有重新整理堆，严格来说可能破坏优先
   队列顺序。更稳妥的实现通常是把更新后的节点再次入队，并在弹出时跳过
   过期条目。
2. 析构函数释放了每个 `GridNode`，但没有释放三级指针中由 `new[]` 创建的
   外层数组。通常进程退出时会由操作系统回收，但反复创建搜索器时可能泄漏。
3. 邻居检查只验证邻居体素是否占据，没有额外检查对角移动经过的相邻面是否
   被障碍封住，因此在某些离散地图上可能出现“贴角穿越”。
4. `Coord2Index()` 使用加 `0.5` 后转整数的方式离散坐标。负方向上的截断
   行为与严格的四舍五入并不完全相同。
5. 0.2 秒是写死在代码中的搜索超时，不是 ROS 参数。

## 12. 推荐阅读顺序

1. 先看 `GridNode`，理解一个节点保存什么；
2. 看 `NodeComparator`，理解 open set 怎样选择节点；
3. 看 `Index2Coord()` 和 `Coord2Index()`；
4. 看 `getDiagHeu()`；
5. 看 `retrievePath()`；
6. 最后完整阅读 `AstarSearch()`。

阅读 `AstarSearch()` 时，可以只抓住下面这段伪代码：

```text
open.push(start)

while open 非空:
    current = open 中 f 最小的节点

    if current 是终点:
        回溯路径并返回

    把 current 标为 closed

    for neighbor in current 的 26 个邻居:
        if neighbor 不可通行:
            continue

        new_g = current.g + 一步代价

        if neighbor 未发现，或者 new_g 更小:
            neighbor.cameFrom = current
            neighbor.g = new_g
            neighbor.f = new_g + h
            把 neighbor 放入 open
```
