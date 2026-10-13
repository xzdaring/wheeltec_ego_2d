#ifndef _PLANNING_VISUALIZATION_H_
#define _PLANNING_VISUALIZATION_H_

#include <Eigen/Eigen>
#include <algorithm>
// #include <bspline_opt/uniform_bspline.h>
#include <iostream>
// #include <bspline_opt/polynomial_traj.h>
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <stdlib.h>

using std::vector;
namespace ego_planner
{
  /**
   * @brief 把规划过程中的点、控制多边形和搜索路径转换成 RViz Marker。
   *
   * 该类只做可视化发布，不参与规划决策。所有 Eigen::Vector3d 均按 world 坐标系
   * 解释；scale 单位为米，color 顺序为 RGBA，id 用于覆盖或区分同话题 Marker。
   */
  class PlanningVisualization
  {
  private:
    rclcpp::Node::SharedPtr node_; ///< 提供时钟、frame 配置和发布者生命周期的 ROS 2 节点。
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr goal_point_pub;   ///< 局部/最终目标点。
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr global_list_pub; ///< 全局参考轨迹采样。
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr init_list_pub;   ///< 优化前初始路径/候选。
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr optimal_list_pub;///< 优化后 B 样条控制点。
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr a_star_list_pub; ///< 碰撞段对应的 A* 绕障路径。
    // ros::Publisher guide_vector_pub;
    // ros::Publisher intermediate_state_pub;

  public:
    ~PlanningVisualization() {}
    PlanningVisualization(const rclcpp::Node::SharedPtr &node)
      : node_(node)
    {
      // 初始化发布者，调整消息类型和队列大小
      goal_point_pub = node_->create_publisher<visualization_msgs::msg::Marker>("goal_point", 2);
      global_list_pub = node_->create_publisher<visualization_msgs::msg::Marker>("global_list", 2);
      init_list_pub = node_->create_publisher<visualization_msgs::msg::Marker>("init_list", 2);
      optimal_list_pub = node_->create_publisher<visualization_msgs::msg::Marker>("optimal_list", 2);
      a_star_list_pub = node_->create_publisher<visualization_msgs::msg::Marker>("a_star_list", 20);
    }

    typedef std::shared_ptr<PlanningVisualization> Ptr;

    /// 使用指定发布者显示一列点；show_sphere=false 时通常改用线段表现。
    void displayMarkerList(rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr &pub, const std::vector<Eigen::Vector3d> &list, double scale,
                           Eigen::Vector4d color, int id, bool show_sphere = true);
    /// 向 MarkerArray 追加路径显示 Marker，不负责发布 array。
    void generatePathDisplayArray(visualization_msgs::msg::MarkerArray &array,
                                  const std::vector<Eigen::Vector3d> &list, double scale, Eigen::Vector4d color, int id);
    /// 把 list 中每两个相邻点解释为箭头起终点并追加到 array。
    void generateArrowDisplayArray(visualization_msgs::msg::MarkerArray &array,
                                   const std::vector<Eigen::Vector3d> &list, double scale, Eigen::Vector4d color, int id);
    /// 显示单个目标球体；goal_point 单位 m，scale 为直径/线宽尺度。
    void displayGoalPoint(Eigen::Vector3d goal_point, Eigen::Vector4d color, const double scale, int id);
    /// 显示全局参考轨迹的离散采样点。
    void displayGlobalPathList(std::vector<Eigen::Vector3d> global_pts, const double scale, int id);
    /// 显示一次优化开始前的 point_set 或控制多边形。
    void displayInitPathList(std::vector<Eigen::Vector3d> init_pts, const double scale, int id);
    /// 同时显示障碍物不同侧的多条拓扑初值。
    void displayMultiInitPathList(std::vector<std::vector<Eigen::Vector3d>> init_trajs, const double scale);
    /// 显示 3×N B 样条控制点矩阵。
    void displayOptimalList(Eigen::MatrixXd optimal_pts, int id);
    /// 显示每个碰撞区段生成的 A* 折线路径。
    void displayAStarList(std::vector<std::vector<Eigen::Vector3d>> a_star_paths, int id);
    void displayArrowList(rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr &pub, const std::vector<Eigen::Vector3d> &list, double scale, Eigen::Vector4d color, int id);
    // void displayIntermediateState(ros::Publisher& intermediate_pub, ego_planner::BsplineOptimizer::Ptr optimizer, double sleep_time, const int start_iteration);
    // void displayNewArrow(ros::Publisher& guide_vector_pub, ego_planner::BsplineOptimizer::Ptr optimizer);
  };
} // namespace ego_planner
#endif
