#!/usr/bin/env python3
import math  # 三角函数用于传感器坐标转换和全向速度旋转。
import numpy as np  # 批量提取在线地图障碍，避免逐格 Python 循环。
import rclpy  # ROS 2 Python 客户端负责消息和仿真时钟。
from rclpy.node import Node  # 节点对象保存订阅器、发布器和定时器。
from rclpy.time import Time  # 零时刻 TF 查询表示最新变换。
from rclpy.qos import QoSProfile  # 地图订阅深度。
from rclpy.qos import DurabilityPolicy, ReliabilityPolicy  # 地图持久化 QoS。
from tf2_ros import Buffer, TransformListener, TransformException  # 真实变换坐标，不能只改 frame_id。
from geometry_msgs.msg import PoseStamped  # RViz 目标接口。
from nav_msgs.msg import OccupancyGrid, Odometry  # 在线 SLAM 地图与底盘反馈。
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32  # 将二维障碍平铺到 EGO 的 z=0 平面。
from sensor_msgs.msg import PointCloud2  # EGO 的独立点云输入消息。
from std_msgs.msg import Header  # 给点云标明坐标系与采样时刻。


def yaw_of(q):  # 仿真底盘按平面运动处理四元数。
    sine = 2.0 * (q.w * q.z + q.x * q.y)  # 偏航正弦分量。
    cosine = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)  # 偏航余弦分量。
    return math.atan2(sine, cosine)  # 提取绕 Z 轴角度。


class NavigationBridge(Node):  # 仅管理地图与目标转换，速度控制交给 C++。
    def __init__(self):  # 构造时创建接口，所有状态由回调更新。
        super().__init__('ego_navigation_bridge')  # 与 launch 中的节点名称保持一致。
        self.planning_frame = self.declare_parameter('planning_frame', 'odom').value
        self.buffer = Buffer()  # 保存 map、odom、laser 和 base_link 的变换历史。
        self.listener = TransformListener(self.buffer, self)  # 保留监听器，防止离开作用域后停止接收 TF。
        self.map = self.odom = None  # 地图和里程计未就绪时不生成点云。
        self.pending_goal = None  # 缓存等待地图与 TF 的点击。
        self.map_time = -100.0  # 数据看门狗初始为过期。
        self.cloud = self.create_publisher(  # EGO 的障碍输入。
            PointCloud2, '/ego/cloud', 10)  # 已真实转换到 odom。
        self.goal_pub = self.create_publisher(  # 同时供 EGO 和小车服务器订阅。
            PoseStamped, '/move_base_simple/goal', 10)  # 转换后的目标。
        map_qos = QoSProfile(  # 后启动也能收到最近地图。
            depth=1,  # 只保留最新地图。
            durability=DurabilityPolicy.TRANSIENT_LOCAL,  # 持久化。
            reliability=ReliabilityPolicy.RELIABLE)  # 可靠传输。
        self.create_subscription(OccupancyGrid, '/map', self.on_map, map_qos)  # 输入来自在线 SLAM，无需预存地图。
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)  # 获取位置和车体朝向用于反馈。
        self.create_subscription(  # RViz2 的标准 Goal Pose 工具输出。
            PoseStamped, '/goal_pose', self.on_goal, 10)  # 保留接口回调。
        self.create_timer(0.2, self.publish_cloud)  # 5 Hz 更新局部障碍并重试待处理目标。

    def seconds(self):  # 全部超时都跟随 Gazebo 仿真时间。
        return self.get_clock().now().nanoseconds * 1e-9  # 暂停仿真不会误判轨迹时间。

    def on_map(self, msg):  # SLAM 地图回调只缓存，批处理放到定时器。
        self.map = msg  # 保留最新完整在线地图。
        self.map_time = self.seconds()  # 地图停止更新时控制器会停车。

    def on_odom(self, msg):  # 底盘位姿始终以 odom 为规划参考。
        if msg.header.frame_id != self.planning_frame:
            self.get_logger().error('里程计 frame 与 planning_frame 不一致，拒绝输入。')
            self.odom = None
            return
        self.odom = msg  # 保留 yaw 和 x/y 的原始反馈。


    def on_goal(self, msg):  # 缓存点击；C++ 同时监听该话题并停止旧目标。
        self.pending_goal = msg  # 等地图、TF 和里程计就绪后再下发。
        self.get_logger().info('收到目标，等待在线地图及 TF 后交给 EGO。')  # 明确等待原因。

    def publish_cloud(self):  # 将累计 SLAM 地图转为 EGO 可读的平面障碍。
        if self.seconds() - self.map_time >= 3.0:  # 旧地图停止输出，C++ 点云看门狗会停车。
            return  # 不用旧点云伪装感知正常。
        if self.map is None or self.odom is None:  # 未知环境从第一帧雷达地图开始。
            return  # 未就绪时不发布误导性的空地图。
        try:  # map 会随 SLAM 优化变化，因此每次重新查询。
            tr = self.buffer.lookup_transform(  # 地图修正可能持续变化。
                self.planning_frame, self.map.header.frame_id, Time())  # 查询最新变换。
        except TransformException:  # 初始 SLAM 尚未发布 map 到 odom 时等待。
            return  # 不使用伪造的单位变换。
        cells = np.asarray(self.map.data).reshape(  # 恢复二维栅格。
            self.map.info.height, self.map.info.width)  # 行和列顺序。
        rows, cols = np.nonzero(cells >= 50)  # 只将已观测占据区作为障碍，未知区允许局部探索。
        origin = self.map.info.origin  # 地图原点也可能带旋转。
        a = yaw_of(origin.orientation)  # 处理 OccupancyGrid 原点姿态。
        x = (cols + 0.5) * self.map.info.resolution  # 使用每个障碍体素中心。
        y = (rows + 0.5) * self.map.info.resolution  # 行索引对应地图局部 Y。
        mx = origin.position.x + np.cos(a) * x - np.sin(a) * y  # 地图局部栅格转 map 坐标。
        my = origin.position.y + np.sin(a) * x + np.cos(a) * y  # 同时处理原点平移。
        a = yaw_of(tr.transform.rotation)  # map 到 odom 的实际旋转。
        ox = tr.transform.translation.x + np.cos(a) * mx - np.sin(a) * my  # 障碍点转换到 odom。
        oy = tr.transform.translation.y + np.sin(a) * mx + np.cos(a) * my  # 与规划器里程计一致。
        p = self.odom.pose.pose.position  # 围绕小车裁剪局部点云以限制负载。
        keep = (np.abs(ox - p.x) < 5.0) & (np.abs(oy - p.y) < 5.0)  # 与 EGO 局部更新范围配合。
        height = np.zeros(np.count_nonzero(keep))  # 障碍平铺到 z=0。
        points = np.column_stack((ox[keep], oy[keep], height))  # 三维点云接口。
        header = Header(  # 点云已经完成真实坐标转换。
            frame_id=self.planning_frame,  # 与小车规划坐标一致。
            stamp=self.get_clock().now().to_msg())  # 仿真时间。
        self.cloud.publish(create_cloud_xyz32(header, points.tolist()))  # 即使无障碍也刷新局部占据地图。
        if self.pending_goal is not None and self.seconds() - self.map_time < 3.0:  # 地图有效后才允许目标下发。
            self.send_goal()  # 对暂缺 TF 的目标会在下一周期重试。

    def send_goal(self):  # 将 RViz map 目标转换为 EGO odom 目标。
        msg = self.pending_goal  # 新目标覆盖旧待处理目标。
        if msg.header.frame_id not in ('map', self.planning_frame):  # 本入口仅接受已定义的导航坐标系。
            self.get_logger().error('目标坐标系必须为 map 或 odom。')  # 防止静默使用错误坐标。
            self.pending_goal = None  # 明确拒绝，等待用户重新点击。
            return  # 保持停车。
        try:  # 使用最新定位修正解释当前点击的目标。
            tr = self.buffer.lookup_transform(self.planning_frame, msg.header.frame_id, Time())  # odom 目标自然得到单位变换。
        except TransformException:  # 地图与底盘尚未连通时保留目标。
            return  # 后续地图定时器继续重试。
        a = yaw_of(tr.transform.rotation)  # 平面目标只需要 yaw 旋转。
        x, y = msg.pose.position.x, msg.pose.position.y  # 读取点击位置，不采用三维高度。
        goal = PoseStamped()  # 为 EGO 创建独立的目标消息。
        goal.header.frame_id = self.planning_frame  # 输入已经完成真实坐标变换。
        goal.header.stamp = self.get_clock().now().to_msg()  # 与仿真轨迹开始时刻统一。
        tx = tr.transform.translation.x  # 变换的 X 平移。
        ty = tr.transform.translation.y  # 变换的 Y 平移。
        goal.pose.position.x = tx + math.cos(a) * x - math.sin(a) * y  # 目标 X。
        goal.pose.position.y = ty + math.sin(a) * x + math.cos(a) * y  # 目标 Y。
        target_yaw = a + yaw_of(msg.pose.orientation)  # 目标朝向也从 map 转到 odom。
        goal.pose.orientation.z = math.sin(target_yaw / 2.0)  # 平面四元数 Z。
        goal.pose.orientation.w = math.cos(target_yaw / 2.0)  # 平面四元数 W。
        self.pending_goal = None  # 避免周期性重发导致规划器不断重置。
        self.goal_pub.publish(goal)  # 调用原 EGO 的手动目标和滚动状态机。
        self.get_logger().info('目标已转换到 odom 并交给 EGO。')  # 便于排查目标是否进入规划。



def main():  # ROS 可执行入口。
    rclpy.init()  # 解析 ROS 参数，包括 use_sim_time。
    node = NavigationBridge()  # 持有全部接口与状态。
    try:  # Ctrl+C 时释放地图与目标接口。
        rclpy.spin(node)  # 单线程执行回调，避免地图和目标状态竞争。
    except KeyboardInterrupt:  # 正常终端退出不输出异常栈。
        pass  # 后面的 finally 负责清理。
    finally:  # 退出时释放节点资源；停车由 C++ 服务器负责。
        if rclpy.ok():  # ROS 上下文有效时才发布。
            node.destroy_node()  # 释放订阅和定时器。
            rclpy.shutdown()  # 关闭 ROS 上下文。


if __name__ == '__main__':  # 允许被安装成 ROS 可执行脚本。
    main()  # 启动在线导航适配节点。
