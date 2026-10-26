#!/usr/bin/env python3
"""结合车体轮廓与厂家遮挡角度，剔除 V550 自身回波。"""
import copy
from collections import deque
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener


FACTORY_SECTORS = (70.0, 100.0, 120.0, 240.0, 260.0, 290.0)


def factory_hit_mask(scan, mode, near_range, sectors):
    """厂家区间使用雷达坐标的角度；near 模式只丢弃已知遮挡区内的近点。"""
    ranges = np.asarray(scan.ranges, dtype=float)
    mask = np.zeros(len(ranges), dtype=bool)
    if mode == 'off':
        return mask
    angles = np.degrees(scan.angle_min + np.arange(len(ranges)) * scan.angle_increment) % 360.0
    for start, end in zip(sectors[::2], sectors[1::2]):
        # 允许配置跨 0° 的区间，例如 [350,10]；厂家三组默认区间不跨零。
        if start <= end:
            mask |= (angles >= start) & (angles <= end)
        else:
            mask |= (angles >= start) | (angles <= end)
    if mode == 'full':
        # 全角度屏蔽包括无回波的 Inf；遮挡方向是未知，不能当作有效空旷观测。
        return mask
    valid = np.isfinite(ranges) & (ranges >= scan.range_min) & (ranges <= scan.range_max)
    if mode == 'near':
        valid &= ranges <= near_range
    return mask & valid


def normalize_scan(scan, count=1800):
    """固定 0.2 度角度网格，避免串口每圈采样数变化使 SLAM 拒收。"""
    output = copy.deepcopy(scan)
    values = np.asarray(scan.ranges, dtype=float)
    angles = scan.angle_min + np.arange(len(values)) * scan.angle_increment
    indices = np.floor((angles % (2 * math.pi)) * count / (2 * math.pi) + 0.5).astype(int) % count
    # 同一角度格保留最近有效回波；不插值跨越遮挡区，也不把缺测当成自由空间。
    ranges = np.full(count, np.inf)
    valid = np.isfinite(values) & (values >= scan.range_min) & (values <= scan.range_max)
    np.minimum.at(ranges, indices[valid], values[valid])
    observed = np.zeros(count, dtype=bool)
    observed[indices[valid | np.isposinf(values)]] = True
    ranges[~observed] = np.nan
    output.angle_min = 0.0
    output.angle_increment = 2 * math.pi / count
    output.angle_max = (count - 1) * output.angle_increment
    output.time_increment = scan.scan_time / count if scan.scan_time > 0 else 0.0
    output.ranges = ranges.tolist()
    output.intensities = []  # 不将原始强度数组错误地配到新的角度网格。
    return output


def body_hit_mask(scan, transform, bounds):
    """先按 TF 转到车体坐标，再判断点是否位于实物占用的矩形内。"""
    ranges = np.asarray(scan.ranges, dtype=float)
    angles = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment
    valid = np.isfinite(ranges) & (ranges >= scan.range_min) & (ranges <= scan.range_max)
    # 无效距离暂用零计算，最后由 valid 排除，避免 inf * 0 产生运算警告。
    distance = np.where(valid, ranges, 0.0)
    points = np.column_stack((distance * np.cos(angles), distance * np.sin(angles),
                              np.zeros(len(ranges))))
    q = transform.rotation
    vector = np.array([q.x, q.y, q.z])
    norm = math.sqrt(float(vector @ vector) + q.w * q.w)
    if norm < 1e-12:
        raise ValueError('雷达 TF 的四元数无效')
    vector, w = vector / norm, q.w / norm
    # 四元数旋转支持实际安装的 roll/pitch/yaw，不能只改消息 frame_id。
    cross = 2.0 * np.cross(vector, points)
    points += w * cross + np.cross(vector, cross)
    points += np.array([transform.translation.x, transform.translation.y,
                        transform.translation.z])
    xmin, xmax, ymin, ymax = bounds
    return valid & (points[:, 0] >= xmin) & (points[:, 0] <= xmax) & \
        (points[:, 1] >= ymin) & (points[:, 1] <= ymax)


class SettlingGate:
    """仿真专用：在扫描时刻检查落地高度、倾角及垂直速度，并连续稳定一段时间。"""
    def __init__(self, max_z=0.04, max_tilt=math.radians(2.0), max_vz=0.003, hold=1.0):
        self.max_z, self.max_tilt, self.max_vz, self.hold = max_z, max_tilt, max_vz, hold
        self.previous = None
        self.since = None

    def update(self, stamp, z, roll, pitch):
        previous = self.previous
        self.previous = (stamp, z)
        if previous is None or not 0 < stamp - previous[0] <= 0.5:
            self.since = None
            return False
        vz = (z - previous[1]) / (stamp - previous[0])
        safe = all(math.isfinite(v) for v in (z, roll, pitch, vz)) and (
            0.0 <= z <= self.max_z and max(abs(roll), abs(pitch)) <= self.max_tilt
            and abs(vz) <= self.max_vz)
        if not safe:
            self.since = None
            return False
        if self.since is None:
            self.since = stamp
        return stamp - self.since >= self.hold


class ScanSelfFilter(Node):
    def __init__(self):
        super().__init__('scan_self_filter')
        self.sim_gate_enabled = self.declare_parameter('sim_settling_gate', False).value
        self.sim_gate_frame = self.declare_parameter('sim_gate_frame', 'odom').value
        self.sim_gate = SettlingGate()
        self.pending = deque(maxlen=12)
        self.gate_ready = False
        self.base_frame = self.declare_parameter('base_frame', 'base_footprint').value
        # 用户实测含轮长 26 cm、宽 23 cm，与 CAD 包络基本一致。
        # 预留 1 cm 尺寸测量和模型误差；轮廓外近点保留，不改近障停车半径。
        self.bounds = tuple(self.declare_parameter(name, value).value for name, value in (
            ('min_x', -0.140), ('max_x', 0.140), ('min_y', -0.125), ('max_y', 0.125)))
        if not (self.bounds[0] < self.bounds[1] and self.bounds[2] < self.bounds[3]):
            raise ValueError('车体轮廓的最小边界必须小于最大边界')
        self.factory_mode = self.declare_parameter('factory_mask_mode', 'full').value
        # 本车后角距前置雷达最远约 24.3 cm，近距门槛留约 2 cm 测量余量。
        # 这是雷达测距，不是车体中心的安全停车半径。
        self.factory_near_range = self.declare_parameter('factory_near_range', 0.26).value
        self.factory_sectors = self.declare_parameter(
            'factory_mask_angles', list(FACTORY_SECTORS)).value
        if self.factory_mode not in ('near', 'full', 'off'):
            raise ValueError('factory_mask_mode 必须为 near、full 或 off')
        if not math.isfinite(self.factory_near_range) or self.factory_near_range <= 0:
            raise ValueError('factory_near_range 必须为正数，单位米')
        if not self.factory_sectors or len(self.factory_sectors) % 2 or any(
                not math.isfinite(v) or not 0 <= v <= 360 for v in self.factory_sectors):
            raise ValueError('factory_mask_angles 必须为 0～360° 的起止角度对')
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.publisher = self.create_publisher(LaserScan, '/scan', 10)
        self.create_subscription(LaserScan, '/scan_raw', self.on_scan, qos_profile_sensor_data)
        if self.sim_gate_enabled:
            self.get_logger().info('仿真启动保护：等待高度不超过4 cm、倾角不超过2度、垂直速度不超过3 mm/s，并连续稳定1秒。')
            self.create_timer(0.02, self.drain_pending)
        self.get_logger().info(f'自身回波过滤轮廓 {self.bounds}，坐标系 {self.base_frame}')
        self.get_logger().info(
            f'厂家角度过滤模式 {self.factory_mode}，近距门槛 {self.factory_near_range} m，'
            f'雷达角度区间 {list(self.factory_sectors)}')

    def on_scan(self, scan):
        if self.sim_gate_enabled:
            self.pending.append(scan) # 等扫描时刻的odom TF就绪，避免使用错时姿态。
            return
        self.filter_scan(scan)

    def drain_pending(self):
        while self.pending:
            scan = self.pending[0]
            stamp = Time.from_msg(scan.header.stamp)
            age = self.get_clock().now().nanoseconds * 1e-9 - stamp.nanoseconds * 1e-9
            if age > 0.5 or age < -0.1:
                self.pending.popleft()
                self.sim_gate.since = None
                self.gate_ready = False
                continue
            try:
                pose = self.buffer.lookup_transform(self.sim_gate_frame, self.base_frame, stamp).transform
            except TransformException:
                return # 等待下一次TF到达，超过0.5秒则由上方丢弃。
            self.pending.popleft()
            q = pose.rotation
            roll = math.atan2(2 * (q.w*q.x + q.y*q.z), 1 - 2 * (q.x*q.x + q.y*q.y))
            pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w*q.y - q.z*q.x))))
            ready = self.sim_gate.update(stamp.nanoseconds * 1e-9, pose.translation.z, roll, pitch)
            if not ready:
                if self.gate_ready:
                    self.get_logger().warning('仿真车体姿态异常，暂停扫描；下游超时停车。')
                self.gate_ready = False
                continue # 不发布空扫描，避免把暂时不可用的感知当作空旷环境。
            if not self.gate_ready:
                self.get_logger().info('仿真车体已落地并稳定，开始放行扫描。')
            self.gate_ready = True
            self.filter_scan(scan)

    def filter_scan(self, scan):
        try:
            tr = self.buffer.lookup_transform(
                self.base_frame, scan.header.frame_id, Time.from_msg(scan.header.stamp))
        except TransformException:
            # 没有可靠安装 TF 时不发布；下游感知超时会停止规划或运动。
            self.get_logger().warning('等待雷达到车体的 TF，暂不发布过滤扫描。',
                                      throttle_duration_sec=5.0)
            return
        scan = normalize_scan(scan)
        mask = body_hit_mask(scan, tr.transform, self.bounds)
        # 矩形覆盖不到的车体结构可能在已知遮挡方向上产生少量近点。
        # 默认按厂家全距离屏蔽；near 作为可选模式保留较远测量。
        mask |= factory_hit_mask(
            scan, self.factory_mode, self.factory_near_range, self.factory_sectors)
        output = copy.deepcopy(scan)
        # NaN 表示被车体遮挡、无有效测量，不把该方向伪装成远距离自由空间。
        for i in np.flatnonzero(mask):
            output.ranges[int(i)] = float('nan')
        self.publisher.publish(output)


def main():
    rclpy.init()
    node = ScanSelfFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
