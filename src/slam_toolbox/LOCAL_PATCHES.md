# 本地边界修复

来源：https://github.com/SteveMacenski/slam_toolbox ，分支 humble。
固定版本：dccc5d1fd2b5007098f4050e681d20da37ef183f。
保留上游源码和许可证；第三方未修改代码未重复添加教学注释。

修改位置：`lib/karto_sdk/include/karto_sdk/Karto.h`，`OccupancyGrid::ComputeDimensions`。

原实现用 Round(尺寸/分辨率) 作为宽高，而 WorldToGrid 同样用 Round。
最大端点可能映射到等于 width 或 height 的索引，RayTrace 拒绝该命中。
本地改为 ceil(尺寸/分辨率)+1，包含最大端点并留出栅格取整余量。
只是扩展地图边界，不伪造障碍、不调整命中概率，也不改变定位优化。

构建后 source 当前工作区；`ros2 pkg prefix slam_toolbox` 应指向本工作区 install。
仅修改 /opt/ros 的头文件不会改变已编译的系统库，本项目使用源码 overlay 实现修复。
