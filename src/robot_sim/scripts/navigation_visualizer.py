#!/usr/bin/env python3
import math  # 计算速度大小、路径长度和参数离散尺度。
import numpy as np  # 按 EGO 节点向量执行 De Boor 求值。
import rclpy  # 接收轨迹和里程计，不参与控制。
from rclpy.node import Node  # 持有显示状态与发布器。
from rcl_interfaces.srv import GetParameters  # 读取实际运行参数。
from geometry_msgs.msg import Point, PoseStamped  # 标记点和最终目标。
from nav_msgs.msg import Odometry, Path  # 保存真实行驶路径。
from quadrotor_msgs.msg import PositionCommand  # 显示当前参考位置。
from traj_utils.msg import Bspline  # 使用规划器真正输出的曲线。
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import read_points
from visualization_msgs.msg import Marker, MarkerArray  # 汇总可视化图层。


def evaluate(points, knots, degree, time):  # 与 EGO 相同的 B 样条求值。
    index = int(np.searchsorted(knots, time, side='right') - 1)  # 当前节点区间。
    index = min(max(index, degree), len(points) - 1)  # 端点归入有效区间。
    work = points[index - degree:index + 1].copy()  # 当前曲线段所需控制点。
    for level in range(1, degree + 1):  # 逐级线性插值，不画控制多边形充当曲线。
        for j in range(degree, level - 1, -1):  # 逆序避免覆盖本轮仍需的数据。
            i = index - degree + j  # 对应原节点索引。
            width = knots[i + degree - level + 1] - knots[i]  # 当前插值跨度。
            alpha = (time - knots[i]) / width  # EGO 节点严格递增。
            work[j] = (1.0 - alpha) * work[j - 1] + alpha * work[j]
    return work[degree]  # 返回真实曲线上的位置。


class NavigationVisualizer(Node):  # 独立可视化节点，不发布速度或导航目标。
    def __init__(self):  # 订阅全部显示所需信息。
        super().__init__('navigation_visualizer')  # 便于单独诊断。
        self.planning_frame = self.declare_parameter('planning_frame', 'odom').value
        self.grid_resolution = 0.1
        self.inflated_cells = []
        self.items = {}  # 每个命名空间保存最新标记，定期重发兼容晚开的 RViz。
        self.path = Path()  # 实际执行路径与期望轨迹分开显示。
        self.path.header.frame_id = self.planning_frame  # 跟随真实里程计坐标。
        self.pose = Point()  # 信息文字放在小车旁边。
        self.details = '等待局部轨迹'  # 尚未规划时也显示状态。
        self.parameters = '等待规划器参数'  # 不用硬编码值冒充运行参数。
        self.future = None  # 防止参数请求重复堆积。
        self.pub = self.create_publisher(MarkerArray, '/ego/visualization', 10)
        self.path_pub = self.create_publisher(Path, '/ego/executed_path', 10)
        self.client = self.create_client(  # Humble 的标准参数服务。
            GetParameters, '/drone_0_ego_planner_node/get_parameters')
        self.create_subscription(  # 全局参考由 EGO 自己生成。
            Marker, '/drone_0_plan_vis/global_list', self.on_global, 10)
        self.create_subscription(  # 局部优化结果按完整节点向量重新采样。
            Bspline, '/drone_0_planning/bspline', self.on_spline, 10)
        self.create_subscription(  # 最终目标已经转换到 odom。
            PoseStamped, '/move_base_simple/goal', self.on_goal, 10)
        self.create_subscription(  # 当前采样参考与整条曲线分开显示。
            PositionCommand, '/ego/position_cmd', self.on_reference, 10)
        self.create_subscription(PointCloud2, '/drone_0_grid/grid_map/occupancy_inflate', self.on_inflated, 1)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_timer(0.5, self.publish)  # 低频重发显示，不拖慢控制节点。

    def marker(self, name, kind, color, size):  # 统一坐标、颜色和标记尺度。
        marker = Marker()  # 独立消息避免修改已经缓存的图层。
        marker.header.frame_id = self.planning_frame  # 所有几何已经位于真实规划坐标系。
        marker.ns = name  # 分开控制点、曲线、目标和文字。
        marker.type = kind  # LINE_STRIP、POINTS、ARROW 或文字。
        marker.pose.orientation.w = 1.0  # 使用有效单位四元数。
        marker.scale.x = marker.scale.y = marker.scale.z = size
        marker.color.r, marker.color.g, marker.color.b = color  # 不同图层不同颜色。
        marker.color.a = 1.0  # 默认可见。
        return marker  # 调用方补充位置与点列。

    def on_global(self, msg):  # 全局参考线不代表全局无碰撞路线。
        if msg.type != Marker.LINE_STRIP:  # 忽略重复的点列表。
            return  # 只缓存连续线。
        msg.ns = 'global_reference'  # 单独命名空间方便辨认。
        msg.scale.x = 0.035  # 全局参考使用较细的青色线。
        msg.color.r, msg.color.g, msg.color.b = 0.0, 0.8, 1.0
        self.items['global'] = msg  # 持续重发，RViz 晚启动也能看到。

    def on_goal(self, msg):  # 显示最终目标，不和局部末端混淆。
        marker = self.marker('final_goal', Marker.SPHERE, (1.0, 0.2, 0.8), 0.18)
        marker.pose.position = msg.pose.position  # odom 中的实际目标位置。
        self.items['goal'] = marker  # 紫色球表示最终目标。
        arrow = self.marker(  # 紫色箭头显示用户指定的最终朝向。
            'final_heading', Marker.ARROW, (1.0, 0.2, 0.8), 0.1)
        arrow.pose = msg.pose  # 使用已转换到 odom 的完整目标位姿。
        arrow.scale.x = 0.4  # 朝向箭头长度 m。
        arrow.scale.y = arrow.scale.z = 0.08  # 头部与杆的显示宽度。
        self.items['heading'] = arrow  # 与最终位置球同时显示。

    def on_reference(self, msg):  # 黄色小球表示当前跟踪参考。
        marker = self.marker('current_reference', Marker.SPHERE, (1.0, 0.9, 0.0), 0.09)
        marker.pose.position = msg.position  # 不把参考点误认为车体实际位置。
        self.items['reference'] = marker  # 真实位置由 RobotModel 显示。

    def on_spline(self, msg):  # 提取连续曲线和速度方向。
        degree = msg.order  # 当前 EGO 使用三次 B 样条。
        points = np.array([(p.x, p.y, p.z) for p in msg.pos_pts])
        knots = np.array(msg.knots)  # 必须使用优化后的时间分配。
        if degree != 3 or len(points) < 4 or len(knots) != len(points) + 4:
            return  # 非法消息不进入求值器。
        if not np.all(np.diff(knots) > 0):  # 当前均匀 B 样条要求严格递增。
            return  # 避免除以零。
        times = np.linspace(knots[degree], knots[len(points)], 101)
        samples = np.array([evaluate(points, knots, degree, t) for t in times])
        curve = self.marker('local_curve', Marker.LINE_STRIP, (1.0, 0.3, 0.05), 0.055)
        curve.points = [Point(x=float(p[0]), y=float(p[1]), z=0.04) for p in samples]
        self.items['curve'] = curve  # 橙色连续线是真正执行的局部曲线。
        controls = self.marker('control_points', Marker.POINTS, (1.0, 1.0, 1.0), 0.06)
        controls.points = list(msg.pos_pts)  # 白色点为控制点，通常不在曲线上。
        self.items['controls'] = controls  # 可与曲线比较优化效果。
        end = self.marker('local_endpoint', Marker.SPHERE, (0.2, 1.0, 0.1), 0.13)
        end.pose.position = curve.points[-1]  # 绿色球表示当前局部轨迹末端。
        self.items['end'] = end  # 末端随重规划推进，不等同于最终目标。
        speeds = np.linalg.norm(np.diff(samples, axis=0), axis=1) / np.diff(times)
        length = np.linalg.norm(np.diff(samples, axis=0), axis=1).sum()
        self.details = (  # 数值仅用于显示，速度由有限差分估计。
            f'轨迹 #{msg.traj_id} | 控制点 {len(points)}\n'
            f'局部长 {length:.2f} m | 时长 {times[-1]-times[0]:.2f} s\n'
            f'采样峰值速度 {speeds.max():.2f} m/s')
        for i in range(0, 100, 10):  # 固定十支箭头，避免重规划残留旧标记。
            arrow = self.marker('velocity', Marker.ARROW, (1.0, 0.8, 0.1), 0.02)
            arrow.id = i  # 相同编号覆盖上一条轨迹的箭头。
            delta = (samples[i + 1] - samples[i]) / (times[i + 1] - times[i])
            tip = samples[i] + delta * 0.5  # 箭头长度表示未来半秒的速度尺度。
            arrow.points = [curve.points[i], Point(x=float(tip[0]), y=float(tip[1]), z=0.04)]
            arrow.scale.y, arrow.scale.z = 0.05, 0.08  # 箭头头部尺寸。
            self.items[f'arrow{i}'] = arrow  # 黄色箭头显示行进方向与速度大小。

    def on_inflated(self, msg):
        # PointCloud2 不携带体素边长，RViz 固定 Size 会在修改分辨率后显示错误。
        # 用实际栅格中心和参数化尺寸显示同一份占据层，不额外画近似安全圆。
        if msg.header.frame_id != self.planning_frame:
            return
        self.inflated_cells = list(read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True))

    def update_inflated_marker(self):
        marker = self.marker('inflated_cells', Marker.CUBE_LIST, (0.7, 0.3, 1.0), self.grid_resolution)
        marker.scale.z = 0.01
        marker.color.a = 0.25
        cells = {(float(p[0]), float(p[1])) for p in self.inflated_cells
                 if abs(float(p[2])) <= self.grid_resolution * 0.51}
        marker.points = [Point(x=x, y=y, z=0.0) for x, y in cells]
        self.items['inflated_cells'] = marker

    def on_odom(self, msg):  # 记录实际走过的路径。
        self.pose = msg.pose.pose.position  # 文字跟随车体移动。
        if self.path.poses:  # 仅在移动足够距离后新增点。
            last = self.path.poses[-1].pose.position
            if math.hypot(last.x - self.pose.x, last.y - self.pose.y) < 0.03:
                return  # 静止时不重复积累点。
        pose = PoseStamped(header=msg.header, pose=msg.pose.pose)
        self.path.poses.append(pose)  # 轨迹来自实际反馈，不是规划结果。
        self.path.poses = self.path.poses[-2000:]  # 限制长时间运行的显示负载。

    def read_parameters(self):  # 读取当前规划节点实际参数。
        if self.future is None and self.client.service_is_ready():
            names = ['grid_map/obstacles_inflation', 'grid_map/resolution',
                     'manager/planning_horizon', 'manager/max_vel']
            self.future = self.client.call_async(GetParameters.Request(names=names))  # 异步查询，不阻塞消息回调。
        elif self.future is not None and self.future.done():
            try:  # 规划器重启时允许再次查询。
                values = [p.double_value for p in self.future.result().values]
                radius, resolution, horizon, speed = values  # 与请求顺序一致。
                self.grid_resolution = resolution
                self.parameters = (  # 显示配置值与栅格离散尺度。
                    f'膨胀半径 {radius:.2f} m | 分辨率 {resolution:.2f} m\n'
                    f'点云圆形膨胀；格边长 {resolution:.2f} m（含离散余量）\n'
                    f'规划视野 {horizon:.2f} m | 规划限速 {speed:.2f} m/s')
            except (AttributeError, ZeroDivisionError, RuntimeError):
                self.parameters = '规划器参数暂不可用'  # 不显示过期的伪造配置。
            self.future = None  # 下个周期重新读取，参数变化能反映到界面。

    def publish(self):  # 重发所有图层和信息文字。
        self.update_inflated_marker()
        self.read_parameters()  # 从运行节点读取膨胀参数。
        text = self.marker('information', Marker.TEXT_VIEW_FACING, (1.0, 1.0, 1.0), 0.16)
        text.pose.position = Point(x=self.pose.x, y=self.pose.y + 0.7, z=0.6)
        text.text = self.parameters + '\n' + self.details  # 文字随车显示。
        self.items['text'] = text  # 固定命名空间覆盖旧文字。
        stamp = self.get_clock().now().to_msg()  # 全部图层使用仿真时间。
        for marker in self.items.values():  # 缓存全局参考支持晚打开 RViz。
            marker.header.stamp = stamp  # 保持可查询的最新 TF 时间。
        self.pub.publish(MarkerArray(markers=list(self.items.values())))
        self.path.header.stamp = stamp  # 行驶轨迹也在 odom 中显示。
        self.path_pub.publish(self.path)  # RViz 按 TF 变换到 map。


def main():  # 独立显示节点入口。
    rclpy.init()  # 使用 launch 传入的仿真时间。
    node = NavigationVisualizer()  # 保存显示缓存与参数客户端。
    try:  # 支持终端正常退出。
        rclpy.spin(node)  # 可视化不干涉导航控制回调。
    except KeyboardInterrupt:
        pass  # 继续执行资源清理。
    finally:
        node.destroy_node()  # 释放接口。
        if rclpy.ok():
            rclpy.shutdown()  # 关闭 ROS 上下文。


if __name__ == '__main__':  # ROS 安装脚本入口。
    main()  # 启动显示。
