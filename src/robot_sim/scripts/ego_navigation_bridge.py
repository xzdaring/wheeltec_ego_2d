#!/usr/bin/env python3
import math  # 三角函数用于传感器坐标转换和全向速度旋转。
import numpy as np  # 批量提取在线地图障碍，避免逐格 Python 循环。
import rclpy  # ROS 2 Python 客户端负责消息和仿真时钟。
from rclpy.node import Node  # 节点对象保存订阅器、发布器和定时器。
from rclpy.time import Time  # 零时刻 TF 查询表示最新变换。
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data  # 匹配地图与雷达的不同 QoS。
from tf2_ros import Buffer, TransformListener, TransformException  # 真实变换坐标，不能只改 frame_id。
from geometry_msgs.msg import PoseStamped, Twist  # RViz 目标和底盘速度接口。
from nav_msgs.msg import OccupancyGrid, Odometry  # 在线 SLAM 地图与底盘反馈。
from sensor_msgs.msg import LaserScan  # 雷达还承担近距离独立停车检查。
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32  # 将二维障碍平铺到 EGO 的 z=0 平面。
from sensor_msgs.msg import PointCloud2  # EGO 的独立点云输入消息。
from std_msgs.msg import Header  # 给点云标明坐标系与采样时刻。
from quadrotor_msgs.msg import PositionCommand  # 复用原 EGO 轨迹服务器的期望状态。
from traj_utils.msg import Bspline  # 新目标必须等新轨迹，避免继续执行旧目标。


def yaw_of(q):  # 仿真底盘按平面运动处理四元数。
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))  # 提取绕 Z 轴角度。


class NavigationBridge(Node):  # 集中管理地图适配、目标变换和小车反馈控制。
    def __init__(self):  # 构造时创建接口，所有状态由回调更新。
        super().__init__('ego_navigation_bridge')  # 与 launch 中的节点名称保持一致。
        self.buffer = Buffer()  # 保存 map、odom、laser 和 base_link 的变换历史。
        self.listener = TransformListener(self.buffer, self)  # 保留监听器，防止离开作用域后停止接收 TF。
        self.map = self.odom = self.command = self.scan = None  # 未收到必要数据时必须停车。
        self.goal = self.pending_goal = None  # 分别保存已下发目标和等待地图的点击。
        self.enabled = False  # 新目标发布后等新的 B 样条再允许运动。
        self.last_traj_id = self.required_traj_id = -1  # 用轨迹编号区分新目标与旧目标，避免节点时钟传播的微小延迟。
        self.goal_time = 0.0  # 用轨迹开始时间拒绝上一目标残留轨迹。
        self.odom_time = self.command_time = self.scan_time = self.map_time = -100.0  # 数据看门狗初始为过期。
        self.status_time = -100.0  # 等待日志每五秒一次，避免看门狗原因被隐藏。
        self.scan_points = []  # 保存车体坐标下的当前雷达命中点。
        self.cloud = self.create_publisher(PointCloud2, '/ego/cloud', 10)  # 发布到规划 launch 显式映射的点云接口。
        self.goal_pub = self.create_publisher(PoseStamped, '/move_base_simple/goal', 10)  # 复用 EGO 手动目标回调。
        self.velocity = self.create_publisher(Twist, '/cmd_vel', 10)  # 仿真底盘唯一的导航速度来源。
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)  # 后启动节点也能收到最近地图。
        self.create_subscription(OccupancyGrid, '/map', self.on_map, map_qos)  # 输入来自在线 SLAM，无需预存地图。
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)  # 获取位置和车体朝向用于反馈。
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)  # 使用 Best Effort 兼容 Gazebo 雷达。
        self.create_subscription(PoseStamped, '/goal_pose', self.on_goal, 10)  # RViz2 的标准 Goal Pose 工具输出。
        self.create_subscription(PositionCommand, '/ego/position_cmd', self.on_command, 10)  # 获取轨迹采样器输出。
        self.create_subscription(Bspline, '/drone_0_planning/bspline', self.on_bspline, 10)  # 等待与最新目标对应的新规划。
        self.create_timer(0.2, self.publish_cloud)  # 5 Hz 更新局部障碍并重试待处理目标。
        self.create_timer(0.05, self.control)  # 20 Hz 发布带反馈的全向速度。

    def seconds(self):  # 全部超时都跟随 Gazebo 仿真时间。
        return self.get_clock().now().nanoseconds * 1e-9  # 暂停仿真不会误判轨迹时间。

    def on_map(self, msg):  # SLAM 地图回调只缓存，批处理放到定时器。
        self.map = msg  # 保留最新完整在线地图。
        self.map_time = self.seconds()  # 地图停止更新时控制器会停车。

    def on_odom(self, msg):  # 底盘位姿始终以 odom 为规划参考。
        self.odom = msg  # 保留 yaw 和 x/y 的原始反馈。
        self.odom_time = self.seconds()  # 反馈中断时禁止继续盲走。

    def on_command(self, msg):  # 接收连续轨迹期望状态。
        if msg.header.frame_id == 'odom':  # 防止把其他坐标系速度当成 odom。
            self.command = msg  # 缓存最新参考位置与速度。
            self.command_time = self.seconds()  # 检测轨迹服务器掉线。

    def on_bspline(self, msg):  # 新目标后只有新轨迹能解除等待。
        stamp = msg.start_time.sec + msg.start_time.nanosec * 1e-9  # 与目标发布的仿真时刻比较。
        self.last_traj_id = max(self.last_traj_id, msg.traj_id)  # 保存已接收轨迹的最大编号。
        if self.goal is not None and self.pending_goal is None and msg.traj_id > self.required_traj_id and stamp >= self.goal_time - 0.2:  # 新编号加时间容差，兼容 /clock 在不同节点的传播延迟。
            self.enabled = True  # 指令时刻仍会在 control 中检查。

    def on_goal(self, msg):  # 接到新点击立即停止旧目标。
        self.required_traj_id = self.last_traj_id  # 新点击要求收到更大的轨迹编号。
        self.pending_goal = msg  # 等地图、TF 和里程计就绪后再下发。
        self.enabled = False  # 等待阶段不得执行旧轨迹。
        self.command = None  # 清空上一目标的期望状态。
        self.velocity.publish(Twist())  # 主动发送零速度。
        self.get_logger().info('收到目标，等待在线地图及 TF 后交给 EGO。')  # 明确等待原因。

    def on_scan(self, msg):  # 将当前雷达命中转到车体，用于近障停车。
        try:  # TF 尚未建立时不得假定雷达就在车体中心。
            tr = self.buffer.lookup_transform('base_link', msg.header.frame_id, Time.from_msg(msg.header.stamp))  # 按雷达采样时间获取安装变换。
        except TransformException:  # 丢弃无法定位的一帧，不刷新有效时间。
            return  # 看门狗会保持停车。
        a = yaw_of(tr.transform.rotation)  # 当前模型雷达无俯仰，使用平面变换。
        t = tr.transform.translation  # 雷达相对底盘的真实安装偏移。
        self.scan_points = [(t.x + r * math.cos(msg.angle_min + i * msg.angle_increment + a), t.y + r * math.sin(msg.angle_min + i * msg.angle_increment + a)) for i, r in enumerate(msg.ranges) if math.isfinite(r) and msg.range_min <= r <= msg.range_max]  # 无穷远不是障碍命中。
        self.scan = msg  # 记录已成功变换的雷达帧。
        self.scan_time = self.seconds()  # 仅有效帧刷新传感器看门狗。

    def publish_cloud(self):  # 将累计 SLAM 地图转为 EGO 可读的平面障碍。
        if self.map is None or self.odom is None:  # 未知环境从第一帧雷达地图开始。
            return  # 未就绪时不发布误导性的空地图。
        try:  # map 会随 SLAM 优化变化，因此每次重新查询。
            tr = self.buffer.lookup_transform('odom', self.map.header.frame_id, Time())  # 把 SLAM 地图转换到连续的里程计规划坐标。
        except TransformException:  # 初始 SLAM 尚未发布 map 到 odom 时等待。
            return  # 不使用伪造的单位变换。
        cells = np.asarray(self.map.data).reshape(self.map.info.height, self.map.info.width)  # 恢复二维栅格布局。
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
        points = np.column_stack((ox[keep], oy[keep], np.zeros(np.count_nonzero(keep))))  # 障碍投影到固定规划高度 z=0。
        header = Header(frame_id='odom', stamp=self.get_clock().now().to_msg())  # 标记转换后的真实坐标系。
        self.cloud.publish(create_cloud_xyz32(header, points.tolist()))  # 即使无障碍也刷新局部占据地图。
        if self.pending_goal is not None and self.seconds() - self.map_time < 3.0:  # 地图有效后才允许目标下发。
            self.send_goal()  # 对暂缺 TF 的目标会在下一周期重试。

    def send_goal(self):  # 将 RViz map 目标转换为 EGO odom 目标。
        msg = self.pending_goal  # 新目标覆盖旧待处理目标。
        if msg.header.frame_id not in ('map', 'odom'):  # 本入口仅接受已定义的导航坐标系。
            self.get_logger().error('目标坐标系必须为 map 或 odom。')  # 防止静默使用错误坐标。
            self.pending_goal = None  # 明确拒绝，等待用户重新点击。
            return  # 保持停车。
        try:  # 使用最新定位修正解释当前点击的目标。
            tr = self.buffer.lookup_transform('odom', msg.header.frame_id, Time())  # odom 目标自然得到单位变换。
        except TransformException:  # 地图与底盘尚未连通时保留目标。
            return  # 后续地图定时器继续重试。
        a = yaw_of(tr.transform.rotation)  # 平面目标只需要 yaw 旋转。
        x, y = msg.pose.position.x, msg.pose.position.y  # 读取点击位置，不采用三维高度。
        goal = PoseStamped()  # 为 EGO 创建独立的目标消息。
        goal.header.frame_id = 'odom'  # 输入已经完成真实坐标变换。
        goal.header.stamp = self.get_clock().now().to_msg()  # 与仿真轨迹开始时刻统一。
        goal.pose.position.x = tr.transform.translation.x + math.cos(a) * x - math.sin(a) * y  # 旋转并平移目标 X。
        goal.pose.position.y = tr.transform.translation.y + math.sin(a) * x + math.cos(a) * y  # 旋转并平移目标 Y。
        goal.pose.orientation.w = 1.0  # 当前只验收到达位置，保持车头角度不作为目标约束。
        self.goal = goal  # 跟踪器用最终目标判断到点。
        self.goal_time = self.seconds()  # 新轨迹必须不早于本次目标。
        self.pending_goal = None  # 避免周期性重发导致规划器不断重置。
        self.goal_pub.publish(goal)  # 调用原 EGO 的手动目标和滚动状态机。
        self.get_logger().info('目标已转换到 odom 并交给 EGO。')  # 便于排查目标是否进入规划。

    def control(self):  # 位置误差反馈与轨迹速度前馈生成麦轮车可执行速度。
        cmd = Twist()  # 默认停车，所有失败分支都保留零速度。
        now = self.seconds()  # 当前仿真时间用于数据有效性检查。
        ready = self.enabled and self.command is not None and self.odom is not None and self.goal is not None  # 等待完整闭环数据。
        fresh = now - self.odom_time < 0.5 and now - self.command_time < 0.5 and now - self.scan_time < 0.5 and now - self.map_time < 3.0  # 任一关键输入中断就停车。
        if self.goal is not None and (not ready or not fresh) and now - self.status_time > 5.0:  # 等待状态也需可诊断。
            self.get_logger().warn(f'等待导航数据: 新轨迹={self.enabled}, 指令={self.command is not None}, 数据年龄 odom/cmd/scan/map={[round(now-t, 2) for t in (self.odom_time, self.command_time, self.scan_time, self.map_time)]}')  # 区分缺轨迹和反馈超时。
            self.status_time = now  # 限制日志频率。
        if ready and fresh:  # 仅在反馈、感知、地图和新轨迹全部有效时运动。
            p = self.odom.pose.pose.position  # 当前底盘位置以 odom 表示。
            ref = self.command  # 当前轨迹采样状态。
            stamp = ref.header.stamp.sec + ref.header.stamp.nanosec * 1e-9  # 拒绝目标发布前的残留指令。
            distance = math.hypot(self.goal.pose.position.x - p.x, self.goal.pose.position.y - p.y)  # 最终位置误差。
            if distance < 0.12:  # 到点后不再跟随持续发布的末端保持指令。
                self.enabled = False  # 下一次点击才重新启动。
                self.goal = None  # 完成后不再输出等待新轨迹的告警。
                self.get_logger().info('到达目标，已停车。')  # 明确到点状态。
            elif stamp >= self.goal_time:  # 防止旧指令与新轨迹回调交错。
                vx = ref.velocity.x + 0.8 * (ref.position.x - p.x)  # 轨迹速度前馈叠加 X 方向位置反馈。
                vy = ref.velocity.y + 0.8 * (ref.position.y - p.y)  # 同样保留全向 Y 方向能力。
                scale = min(1.0, 0.3 / max(math.hypot(vx, vy), 1e-9))  # 限制水平合速度，避免对角运动超速。
                a = yaw_of(self.odom.pose.pose.orientation)  # 将世界速度转换为车体命令。
                cmd.linear.x = scale * (math.cos(a) * vx + math.sin(a) * vy)  # 车头方向速度。
                cmd.linear.y = scale * (-math.sin(a) * vx + math.cos(a) * vy)  # 车体左侧方向速度。
                blocked = any(math.hypot(x - cmd.linear.x * t, y - cmd.linear.y * t) < 0.22 for x, y in self.scan_points for t in (0.0, 0.2, 0.4, 0.6))  # 沿短期平移扫掠圆形包络，独立于地图延迟停车。
                if blocked:  # 近障停车等待 EGO 重规划，不能继续执行危险速度。
                    cmd = Twist()  # 全向两个平移方向同时清零。
        self.velocity.publish(cmd)  # 所有分支都定期输出，避免底盘保留旧速度。


def main():  # ROS 可执行入口。
    rclpy.init()  # 解析 ROS 参数，包括 use_sim_time。
    node = NavigationBridge()  # 持有全部接口与状态。
    try:  # Ctrl+C 时也执行停车清理。
        rclpy.spin(node)  # 单线程执行回调，避免地图和目标状态竞争。
    except KeyboardInterrupt:  # 正常终端退出不输出异常栈。
        pass  # 后面的 finally 负责清理。
    finally:  # 无论如何退出都尽力发布零速度。
        if rclpy.ok():  # ROS 上下文有效时才发布。
            node.velocity.publish(Twist())  # 结束导航时停车。
            node.destroy_node()  # 释放订阅和定时器。
            rclpy.shutdown()  # 关闭 ROS 上下文。


if __name__ == '__main__':  # 允许被安装成 ROS 可执行脚本。
    main()  # 启动在线导航适配节点。
