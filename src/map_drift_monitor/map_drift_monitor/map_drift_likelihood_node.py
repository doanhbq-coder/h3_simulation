"""
Map Drift Monitor — Likelihood Field approach.

Thay vì binary hit/miss, dùng AMCL's Likelihood Field Model:
  1. Precompute distance transform khi nhận map → D[y,x] = khoảng cách tới tường gần nhất (m)
  2. Mỗi scan endpoint tra D → Gaussian score = exp(-d²/2σ²)
  3. match_score = mean(scores),  drift_ratio = 1 - match_score

Ưu điểm so với binary:
  - Graceful falloff: lệch ít → giảm score từ từ, không nhảy 0/1
  - Không cần neighbor_check (distance field đã handle sai số discretization)
  - Chính xác trong môi trường thưa tường (box room, hành lang)
  - Đây là cùng thuật toán AMCL dùng để score từng particle
"""

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

try:
    from scipy.ndimage import distance_transform_edt
    SCIPY_AVAILABLE = True
except ImportError:
    SCIPY_AVAILABLE = False


class MapDriftLikelihoodNode(Node):
    def __init__(self):
        super().__init__('map_drift_likelihood_node')

        self.declare_parameter('scan_topic', '/scan_top')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('robot_frame', 'base_link')
        self.declare_parameter('check_rate', 2.0)
        self.declare_parameter('min_scan_points', 50)
        self.declare_parameter('occupancy_threshold', 50)

        # Gaussian sigma: độ rộng vùng "gần tường" tính là match tốt (mét)
        # Nhỏ hơn → chặt hơn, nhạy hơn với lệch nhỏ
        # Lớn hơn → khoan dung hơn, ít false alarm hơn
        self.declare_parameter('sigma', 0.20)

        # Khoảng cách tối đa để tính (điểm xa hơn có score ≈ 0, tiết kiệm tính toán)
        self.declare_parameter('max_obstacle_distance', 2.0)

        # Calibration
        self.declare_parameter('calibration_samples', 10)

        # Ngưỡng tăng so với baseline
        self.declare_parameter('drift_increase_warn', 0.15)
        self.declare_parameter('drift_increase_alert', 0.22)

        # Ngưỡng tuyệt đối fallback
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
        self.sigma = self.get_parameter('sigma').value
        self.max_obstacle_distance = self.get_parameter('max_obstacle_distance').value
        self.calibration_samples = self.get_parameter('calibration_samples').value
        self.drift_increase_warn = self.get_parameter('drift_increase_warn').value
        self.drift_increase_alert = self.get_parameter('drift_increase_alert').value
        self.drift_warn_threshold = self.get_parameter('drift_warn_threshold').value
        self.drift_alert_threshold = self.get_parameter('drift_alert_threshold').value
        self.consecutive_required = self.get_parameter('consecutive_alerts_required').value

        self.map_data: np.ndarray | None = None
        self.map_info = None
        self.distance_field: np.ndarray | None = None  # D[y,x] = khoảng cách tới tường (m)
        self.latest_scan: LaserScan | None = None
        self.consecutive_alerts = 0
        self.is_drifted = False
        self._calib_buffer: list[float] = []
        self.baseline_drift: float | None = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

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
        self.match_score_pub = self.create_publisher(Float32, '/map_drift/match_score', 10)
        self.alert_pub = self.create_publisher(Bool, '/map_drift/alert', 10)
        self.status_pub = self.create_publisher(String, '/map_drift/status', 10)
        self.reset_request_pub = self.create_publisher(Bool, '/map_drift/reset_pose_request', 10)

        _wall_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(1.0 / self.check_rate, self._check_drift, clock=_wall_clock)

        if not SCIPY_AVAILABLE:
            self.get_logger().error(
                'scipy not found! Install: pip install scipy  '
                'Falling back to binary hit/miss.'
            )

        self.get_logger().info(
            f'MapDriftLikelihood started | sigma={self.sigma}m'
            f' calibration_samples={self.calibration_samples}'
            f' | increase_warn=+{self.drift_increase_warn:.0%}'
            f' increase_alert=+{self.drift_increase_alert:.0%}'
        )

    # ------------------------------------------------------------------

    def _map_cb(self, msg: OccupancyGrid):
        raw = np.array(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
        self.map_data = raw
        self.map_info = msg.info

        n_occupied = int(np.sum(raw >= self.occupancy_threshold))
        self.get_logger().info(
            f'Map received: {msg.info.width}x{msg.info.height} res={msg.info.resolution}m'
            f' occupied={n_occupied}'
            f' origin=({msg.info.origin.position.x:.2f},{msg.info.origin.position.y:.2f})'
            f' | Computing distance field...'
        )

        self._build_distance_field(raw, msg.info.resolution)

    def _build_distance_field(self, map_data: np.ndarray, resolution: float):
        """
        Precompute Euclidean Distance Transform (EDT).
        D[y,x] = khoảng cách (mét) tới ô occupied gần nhất.

        Dùng scipy.ndimage.distance_transform_edt — O(N) rất nhanh.
        Nếu không có scipy, fallback về binary hit/miss.
        """
        if not SCIPY_AVAILABLE:
            self.distance_field = None
            return

        obstacle_mask = map_data >= self.occupancy_threshold

        # distance_transform_edt trả về khoảng cách theo pixel đến cell background=False
        # obstacle_mask=True → obstacle; ~obstacle_mask=True → background (free/unknown)
        # Ta muốn: khoảng cách từ mỗi cell đến obstacle gần nhất
        # → EDT trên (không phải obstacle) = background của occupied
        dist_px = distance_transform_edt(~obstacle_mask)
        self.distance_field = dist_px * resolution  # pixel → mét

        # Clip để tiết kiệm memory và tránh outlier
        self.distance_field = np.minimum(
            self.distance_field, self.max_obstacle_distance
        ).astype(np.float32)

        max_d = float(self.distance_field.max())
        mean_d = float(self.distance_field.mean())
        self.get_logger().info(
            f'Distance field built | max_dist={max_d:.2f}m mean_dist={mean_d:.2f}m'
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

        if self.distance_field is not None:
            drift_ratio, match_score = self._compute_likelihood(scan, tf_stamped)
        else:
            drift_ratio, match_score = self._compute_binary(scan, tf_stamped)

        if drift_ratio is None:
            self.get_logger().warn('Not enough valid scan points', throttle_duration_sec=5.0)
            return

        self.drift_ratio_pub.publish(Float32(data=float(drift_ratio)))
        self.match_score_pub.publish(Float32(data=float(match_score)))

        # --- Calibration phase ---
        if self.baseline_drift is None:
            self._calib_buffer.append(drift_ratio)
            self.get_logger().info(
                f'[CALIBRATING {len(self._calib_buffer)}/{self.calibration_samples}]'
                f' drift={drift_ratio:.1%} match={match_score:.3f}'
                f' — Giữ robot tại vị trí khởi động...'
            )
            if len(self._calib_buffer) >= self.calibration_samples:
                self.baseline_drift = float(np.mean(self._calib_buffer))
                self.baseline_pub.publish(Float32(data=self.baseline_drift))
                self.get_logger().info(
                    f'Calibration done! baseline={self.baseline_drift:.1%}'
                    f' | warn>{self.baseline_drift + self.drift_increase_warn:.1%}'
                    f' | alert>{self.baseline_drift + self.drift_increase_alert:.1%}'
                )
            return

        self.baseline_pub.publish(Float32(data=self.baseline_drift))
        drift_increase = drift_ratio - self.baseline_drift
        self.drift_increase_pub.publish(Float32(data=float(drift_increase)))

        warn_threshold = self.baseline_drift + self.drift_increase_warn
        alert_threshold = self.baseline_drift + self.drift_increase_alert
        is_above_warn = drift_ratio >= warn_threshold or drift_ratio >= self.drift_warn_threshold
        is_above_alert = drift_ratio >= alert_threshold or drift_ratio >= self.drift_alert_threshold

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

        if self.is_drifted:
            level = 'DRIFT_ALERT'
        elif is_above_warn:
            level = 'DRIFT_WARN'
        else:
            level = 'OK'

        status = (
            f'{level} | drift={drift_ratio:.1%}'
            f' match={match_score:.3f}'
            f' base={self.baseline_drift:.1%}'
            f' +{drift_increase:.1%}'
            f' | con={self.consecutive_alerts}/{self.consecutive_required}'
        )
        self.status_pub.publish(String(data=status))

        if level == 'DRIFT_ALERT':
            self.get_logger().error(status)
        elif level == 'DRIFT_WARN':
            self.get_logger().warn(status)
        else:
            self.get_logger().info(status, throttle_duration_sec=3.0)

    # ------------------------------------------------------------------

    def _compute_likelihood(self, scan: LaserScan, tf_stamped) -> tuple[float | None, float]:
        """
        AMCL Likelihood Field Model:
          score_i = exp(-d_i² / 2σ²)   với d_i = khoảng cách endpoint tới tường gần nhất
          match_score = mean(scores)
          drift_ratio  = 1 - match_score
        """
        endpoints = self._project_scan(scan, tf_stamped)
        if endpoints is None:
            return None, 0.0

        px, py, total_valid = endpoints
        res = self.map_info.resolution
        ox = self.map_info.origin.position.x
        oy = self.map_info.origin.position.y
        h, w = self.distance_field.shape

        mx = np.floor((px - ox) / res).astype(int)
        my = np.floor((py - oy) / res).astype(int)

        in_bounds = (mx >= 0) & (mx < w) & (my >= 0) & (my < h)

        # Điểm ngoài bounds: xa map → khoảng cách max
        distances = np.full(len(px), self.max_obstacle_distance, dtype=np.float32)
        if np.any(in_bounds):
            distances[in_bounds] = self.distance_field[my[in_bounds], mx[in_bounds]]

        # Gaussian score: 1.0 khi trúng tường, giảm dần theo khoảng cách
        scores = np.exp(-distances ** 2 / (2.0 * self.sigma ** 2))
        match_score = float(np.mean(scores))
        drift_ratio = 1.0 - match_score

        return drift_ratio, match_score

    def _compute_binary(self, scan: LaserScan, tf_stamped) -> tuple[float | None, float]:
        """Fallback khi không có scipy: binary hit/miss với neighbor 3x3."""
        endpoints = self._project_scan(scan, tf_stamped)
        if endpoints is None:
            return None, 0.0

        px, py, total_valid = endpoints
        res = self.map_info.resolution
        ox = self.map_info.origin.position.x
        oy = self.map_info.origin.position.y
        h, w = self.map_data.shape

        mx = np.floor((px - ox) / res).astype(int)
        my = np.floor((py - oy) / res).astype(int)
        in_bounds = (mx >= 0) & (mx < w) & (my >= 0) & (my < h)

        occupied_hits = 0
        if np.any(in_bounds):
            mx_in, my_in = mx[in_bounds], my[in_bounds]
            hits = np.zeros(len(mx_in), dtype=bool)
            for dy in (-1, 0, 1):
                for dx in (-1, 0, 1):
                    nx, ny = mx_in + dx, my_in + dy
                    valid = (nx >= 0) & (nx < w) & (ny >= 0) & (ny < h)
                    occ = np.zeros(len(mx_in), dtype=bool)
                    occ[valid] = self.map_data[ny[valid], nx[valid]] >= self.occupancy_threshold
                    hits |= occ
            occupied_hits = int(np.sum(hits))

        match_score = occupied_hits / total_valid
        return 1.0 - match_score, match_score

    # ------------------------------------------------------------------

    def _project_scan(self, scan: LaserScan, tf_stamped):
        """Project scan endpoints vào map frame. Trả về (px, py, total_valid) hoặc None."""
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
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y ** 2 + q.z ** 2))

        global_angles = yaw + angles
        px = laser_x + ranges * np.cos(global_angles)
        py = laser_y + ranges * np.sin(global_angles)

        return px, py, total_valid


def main(args=None):
    rclpy.init(args=args)
    node = MapDriftLikelihoodNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
