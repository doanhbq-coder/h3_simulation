import math

import numpy as np
import rclpy
import rclpy.duration
from nav_msgs.msg import OccupancyGrid
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32, String
from tf2_ros import ConnectivityException, ExtrapolationException, LookupException
import tf2_ros


class MapDriftMonitorNode(Node):
    def __init__(self):
        super().__init__('map_drift_monitor_node')

        self.declare_parameter('scan_topic', '/scan_top')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('robot_frame', 'base_link')
        self.declare_parameter('check_rate', 2.0)
        self.declare_parameter('min_scan_points', 50)
        self.declare_parameter('occupancy_threshold', 50)
        self.declare_parameter('neighbor_check', True)

        # --- Calibration ---
        # Số samples đầu tiên dùng để đo baseline drift tại pose đúng
        self.declare_parameter('calibration_samples', 10)

        # --- Ngưỡng tăng so với baseline (adaptive) ---
        # Ưu tiên sử dụng. VD: baseline=13% → warn tại 13+15=28%, alert tại 13+22=35%
        self.declare_parameter('drift_increase_warn', 0.15)   # tăng 15% so với baseline
        self.declare_parameter('drift_increase_alert', 0.22)  # tăng 22% so với baseline

        # --- Ngưỡng tuyệt đối (fallback khi chưa calibrate xong) ---
        self.declare_parameter('drift_warn_threshold', 0.40)
        self.declare_parameter('drift_alert_threshold', 0.55)

        self.declare_parameter('consecutive_alerts_required', 3)

        scan_topic = self.get_parameter('scan_topic').value
        map_topic = self.get_parameter('map_topic').value
        self.map_frame = self.get_parameter('map_frame').value
        self.robot_frame = self.get_parameter('robot_frame').value
        self.check_rate = self.get_parameter('check_rate').value
        self.min_scan_points = self.get_parameter('min_scan_points').value
        self.occupancy_threshold = self.get_parameter('occupancy_threshold').value
        self.neighbor_check = self.get_parameter('neighbor_check').value
        self.calibration_samples = self.get_parameter('calibration_samples').value
        self.drift_increase_warn = self.get_parameter('drift_increase_warn').value
        self.drift_increase_alert = self.get_parameter('drift_increase_alert').value
        self.drift_warn_threshold = self.get_parameter('drift_warn_threshold').value
        self.drift_alert_threshold = self.get_parameter('drift_alert_threshold').value
        self.consecutive_required = self.get_parameter('consecutive_alerts_required').value

        # Runtime state
        self.map_data: np.ndarray | None = None
        self.map_info = None
        self.latest_scan: LaserScan | None = None
        self.consecutive_alerts = 0
        self.is_drifted = False

        # Calibration state
        self._calib_buffer: list[float] = []
        self.baseline_drift: float | None = None  # None = chưa calibrate xong

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # nav2_map_server publish /map với TRANSIENT_LOCAL
        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.map_sub = self.create_subscription(OccupancyGrid, map_topic, self._map_cb, map_qos)
        self.scan_sub = self.create_subscription(LaserScan, scan_topic, self._scan_cb, 10)

        self.drift_ratio_pub = self.create_publisher(Float32, '/map_drift/drift_ratio', 10)
        self.drift_increase_pub = self.create_publisher(Float32, '/map_drift/drift_increase', 10)
        self.baseline_pub = self.create_publisher(Float32, '/map_drift/baseline', 10)
        self.alert_pub = self.create_publisher(Bool, '/map_drift/alert', 10)
        self.status_pub = self.create_publisher(String, '/map_drift/status', 10)
        self.reset_request_pub = self.create_publisher(Bool, '/map_drift/reset_pose_request', 10)

        _wall_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(1.0 / self.check_rate, self._check_drift, clock=_wall_clock)

        self.get_logger().info(
            f'MapDriftMonitor started | calibration_samples={self.calibration_samples}'
            f' | increase_warn=+{self.drift_increase_warn:.0%}'
            f' increase_alert=+{self.drift_increase_alert:.0%}'
        )

    # ------------------------------------------------------------------

    def _map_cb(self, msg: OccupancyGrid):
        raw = np.array(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
        self.map_data = raw
        self.map_info = msg.info

        n_occupied = int(np.sum(raw >= self.occupancy_threshold))
        n_free = int(np.sum(raw == 0))
        n_unknown = int(np.sum(raw == -1))
        self.get_logger().info(
            f'Map received: {msg.info.width}x{msg.info.height} cells'
            f' res={msg.info.resolution}m'
            f' | occupied={n_occupied} free={n_free} unknown={n_unknown}'
            f' | origin=({msg.info.origin.position.x:.2f}, {msg.info.origin.position.y:.2f})'
        )

    def _scan_cb(self, msg: LaserScan):
        self.latest_scan = msg

    # ------------------------------------------------------------------

    def _check_drift(self):
        if self.map_data is None:
            self.get_logger().warn('Waiting for /map...', throttle_duration_sec=5.0)
            return
        if self.latest_scan is None:
            self.get_logger().warn(
                f'Waiting for scan on {self.get_parameter("scan_topic").value}...',
                throttle_duration_sec=5.0,
            )
            return

        scan = self.latest_scan
        try:
            tf_stamped = self.tf_buffer.lookup_transform(
                self.map_frame,
                scan.header.frame_id,
                Time(),
                timeout=rclpy.duration.Duration(seconds=0.1),
            )
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f'TF lookup failed: {e}', throttle_duration_sec=5.0)
            return

        drift_ratio = self._compute_drift(scan, tf_stamped)
        if drift_ratio is None:
            self.get_logger().warn('Not enough valid scan points', throttle_duration_sec=5.0)
            return

        self.drift_ratio_pub.publish(Float32(data=float(drift_ratio)))

        # --- Calibration phase ---
        if self.baseline_drift is None:
            self._calib_buffer.append(drift_ratio)
            remaining = self.calibration_samples - len(self._calib_buffer)
            self.get_logger().info(
                f'[CALIBRATING {len(self._calib_buffer)}/{self.calibration_samples}]'
                f' drift={drift_ratio:.1%} — Giữ robot tại vị trí khởi động...'
            )
            if len(self._calib_buffer) >= self.calibration_samples:
                self.baseline_drift = float(np.mean(self._calib_buffer))
                self.baseline_pub.publish(Float32(data=self.baseline_drift))
                self.get_logger().info(
                    f'Calibration done! baseline_drift={self.baseline_drift:.1%}'
                    f' | warn >{self.baseline_drift + self.drift_increase_warn:.1%}'
                    f' | alert >{self.baseline_drift + self.drift_increase_alert:.1%}'
                )
            return

        # --- Publish baseline mỗi chu kỳ ---
        self.baseline_pub.publish(Float32(data=self.baseline_drift))

        # --- Tính drift tăng so với baseline ---
        drift_increase = drift_ratio - self.baseline_drift
        self.drift_increase_pub.publish(Float32(data=float(drift_increase)))

        warn_threshold = self.baseline_drift + self.drift_increase_warn
        alert_threshold = self.baseline_drift + self.drift_increase_alert

        # Fallback: dùng ngưỡng tuyệt đối nếu drift_increase âm hoặc baseline bất thường
        is_above_warn = drift_ratio >= warn_threshold or drift_ratio >= self.drift_warn_threshold
        is_above_alert = drift_ratio >= alert_threshold or drift_ratio >= self.drift_alert_threshold

        # --- Consecutive counter ---
        if is_above_alert:
            self.consecutive_alerts += 1
        else:
            self.consecutive_alerts = max(0, self.consecutive_alerts - 1)

        was_drifted = self.is_drifted
        self.is_drifted = self.consecutive_alerts >= self.consecutive_required

        self.alert_pub.publish(Bool(data=self.is_drifted))

        if self.is_drifted and not was_drifted:
            self.reset_request_pub.publish(Bool(data=True))
            self.get_logger().error(
                f'MAP DRIFT DETECTED!'
                f' drift={drift_ratio:.1%} baseline={self.baseline_drift:.1%}'
                f' increase=+{drift_increase:.1%} — Please reset pose!'
            )

        # --- Status log ---
        if self.is_drifted:
            level = 'DRIFT_ALERT'
        elif is_above_warn:
            level = 'DRIFT_WARN'
        else:
            level = 'OK'

        status = (
            f'{level} | drift={drift_ratio:.1%}'
            f' baseline={self.baseline_drift:.1%}'
            f' increase=+{drift_increase:.1%}'
            f' | consecutive={self.consecutive_alerts}/{self.consecutive_required}'
        )
        self.status_pub.publish(String(data=status))

        if level == 'DRIFT_ALERT':
            self.get_logger().error(status)
        elif level == 'DRIFT_WARN':
            self.get_logger().warn(status)
        else:
            self.get_logger().info(status, throttle_duration_sec=3.0)

    # ------------------------------------------------------------------

    def _compute_drift(self, scan: LaserScan, tf_stamped) -> float | None:
        n = len(scan.ranges)
        angles = scan.angle_min + np.arange(n) * scan.angle_increment
        ranges = np.array(scan.ranges, dtype=np.float32)

        valid = np.isfinite(ranges) & (ranges >= scan.range_min) & (ranges < scan.range_max)
        total_valid = int(np.sum(valid))
        if total_valid < self.min_scan_points:
            return None

        angles = angles[valid]
        ranges = ranges[valid]

        t = tf_stamped.transform.translation
        q = tf_stamped.transform.rotation
        laser_x, laser_y = t.x, t.y
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y**2 + q.z**2))

        global_angles = yaw + angles
        px = laser_x + ranges * np.cos(global_angles)
        py = laser_y + ranges * np.sin(global_angles)

        res = self.map_info.resolution
        ox = self.map_info.origin.position.x
        oy = self.map_info.origin.position.y
        h, w = self.map_data.shape

        mx = np.floor((px - ox) / res).astype(int)
        my = np.floor((py - oy) / res).astype(int)

        in_bounds = (mx >= 0) & (mx < w) & (my >= 0) & (my < h)
        n_out_of_bounds = int(np.sum(~in_bounds))

        occupied_hits = 0
        n_in_bounds = 0
        direct_hits = 0
        if np.any(in_bounds):
            mx_in = mx[in_bounds]
            my_in = my[in_bounds]
            n_in_bounds = len(mx_in)

            # Direct hits (không cần neighbor)
            direct_vals = self.map_data[my_in, mx_in]
            direct_hits = int(np.sum(direct_vals >= self.occupancy_threshold))

            if self.neighbor_check:
                hits = self._check_neighbors(mx_in, my_in, h, w)
            else:
                hits = direct_vals >= self.occupancy_threshold
            occupied_hits = int(np.sum(hits))

        # Log diagnostic một lần mỗi 5 giây để debug
        self.get_logger().info(
            f'[DIAG] robot=({laser_x:.2f},{laser_y:.2f}) yaw={math.degrees(yaw):.1f}°'
            f' | points: total={total_valid} in_bounds={n_in_bounds} out_of_bounds={n_out_of_bounds}'
            f' | hits: direct={direct_hits} with_neighbor={occupied_hits}'
            f' | match={occupied_hits}/{total_valid}={occupied_hits/total_valid:.1%}',
            throttle_duration_sec=5.0,
        )

        match_ratio = occupied_hits / total_valid
        return float(1.0 - match_ratio)

    def _check_neighbors(self, mx: np.ndarray, my: np.ndarray, h: int, w: int) -> np.ndarray:
        hits = np.zeros(len(mx), dtype=bool)
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                nx = mx + dx
                ny = my + dy
                in_map = (nx >= 0) & (nx < w) & (ny >= 0) & (ny < h)
                occupied = np.zeros(len(mx), dtype=bool)
                occupied[in_map] = self.map_data[ny[in_map], nx[in_map]] >= self.occupancy_threshold
                hits |= occupied
        return hits


def main(args=None):
    rclpy.init(args=args)
    node = MapDriftMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
