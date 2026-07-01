"""
Map Drift Monitor — Ray Casting Comparison approach.

Thuật toán:
  1. Với mỗi tia laser: cast ray qua map để tìm d_map (khoảng cách kỳ vọng tới tường)
  2. So sánh d_actual (LiDAR đo được) vs d_map:
       |d_actual - d_map| ≤ tol  → MATCH  (tia khớp tường)
       d_actual > d_map + tol    → DRIFT  (tia đi xuyên qua tường kỳ vọng!)
       d_actual < d_map - tol    → DYNAMIC (vật thể chắn trước tường → bỏ qua)

  drift_ratio = n_drift / (n_match + n_drift)

Ưu điểm:
  - Vật thể động (ghế, người) chắn trước tường → DYNAMIC → KHÔNG tính là drift
  - Chỉ thực sự báo drift khi robot đứng SAI VỊ TRÍ (tia đi xa hơn tường)
  - Không bị ảnh hưởng bởi môi trường thay đổi
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


class MapDriftRaycastNode(Node):
    def __init__(self):
        super().__init__('map_drift_raycast_node')

        self.declare_parameter('scan_topic', '/scan_top')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('robot_frame', 'base_link')
        self.declare_parameter('check_rate', 2.0)
        self.declare_parameter('min_evaluable_rays', 30)
        self.declare_parameter('occupancy_threshold', 50)

        # Khoảng cách chênh lệch cho phép giữa d_actual và d_map (mét)
        # Nhỏ → chặt hơn, nhạy hơn    Lớn → bỏ qua lệch nhỏ hơn
        self.declare_parameter('range_tolerance', 0.20)

        # Khoảng cast tối đa (mét) — giới hạn để tiết kiệm CPU
        self.declare_parameter('max_cast_range', 10.0)

        # Lấy mẫu mỗi N tia — giảm tính toán mà không mất nhiều độ chính xác
        # 1 = tất cả tia, 3 = mỗi 3 tia (360 tia / 1440-ray lidar)
        self.declare_parameter('ray_skip', 3)

        # Calibration và ngưỡng
        self.declare_parameter('calibration_samples', 10)
        self.declare_parameter('drift_increase_warn', 0.15)
        self.declare_parameter('drift_increase_alert', 0.22)
        self.declare_parameter('drift_warn_threshold', 0.35)
        self.declare_parameter('drift_alert_threshold', 0.50)
        self.declare_parameter('consecutive_alerts_required', 3)

        scan_topic = self.get_parameter('scan_topic').value
        map_topic = self.get_parameter('map_topic').value
        self.map_frame = self.get_parameter('map_frame').value
        self.check_rate = self.get_parameter('check_rate').value
        self.min_evaluable_rays = self.get_parameter('min_evaluable_rays').value
        self.occupancy_threshold = self.get_parameter('occupancy_threshold').value
        self.range_tolerance = self.get_parameter('range_tolerance').value
        self.max_cast_range = self.get_parameter('max_cast_range').value
        self.ray_skip = max(1, self.get_parameter('ray_skip').value)
        self.calibration_samples = self.get_parameter('calibration_samples').value
        self.drift_increase_warn = self.get_parameter('drift_increase_warn').value
        self.drift_increase_alert = self.get_parameter('drift_increase_alert').value
        self.drift_warn_threshold = self.get_parameter('drift_warn_threshold').value
        self.drift_alert_threshold = self.get_parameter('drift_alert_threshold').value
        self.consecutive_required = self.get_parameter('consecutive_alerts_required').value

        self.map_data: np.ndarray | None = None
        self.map_info = None
        self.latest_scan: LaserScan | None = None

        # Cache cos/sin của angles để tránh tính lại mỗi cycle
        self._cached_scan_frame: str | None = None
        self._cached_cos: np.ndarray | None = None
        self._cached_sin: np.ndarray | None = None

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
        self.dynamic_ratio_pub = self.create_publisher(Float32, '/map_drift/dynamic_ratio', 10)
        self.alert_pub = self.create_publisher(Bool, '/map_drift/alert', 10)
        self.status_pub = self.create_publisher(String, '/map_drift/status', 10)
        self.reset_request_pub = self.create_publisher(Bool, '/map_drift/reset_pose_request', 10)

        _wall_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(1.0 / self.check_rate, self._check_drift, clock=_wall_clock)

        self.get_logger().info(
            f'MapDriftRaycast started'
            f' | tol={self.range_tolerance}m ray_skip={self.ray_skip}'
            f' | increase_warn=+{self.drift_increase_warn:.0%}'
            f' increase_alert=+{self.drift_increase_alert:.0%}'
        )

    # ------------------------------------------------------------------

    def _map_cb(self, msg: OccupancyGrid):
        self.map_data = np.array(msg.data, dtype=np.int8).reshape(
            msg.info.height, msg.info.width
        )
        self.map_info = msg.info
        n_occ = int(np.sum(self.map_data >= self.occupancy_threshold))
        self.get_logger().info(
            f'Map: {msg.info.width}x{msg.info.height} res={msg.info.resolution}m'
            f' occupied={n_occ}'
            f' origin=({msg.info.origin.position.x:.2f},{msg.info.origin.position.y:.2f})'
        )

    def _scan_cb(self, msg: LaserScan):
        self.latest_scan = msg
        # Cache cos/sin nếu scan frame thay đổi (hoặc lần đầu)
        if msg.header.frame_id != self._cached_scan_frame:
            n = len(msg.ranges)
            skip = self.ray_skip
            indices = np.arange(0, n, skip)
            angles = msg.angle_min + indices * msg.angle_increment
            self._cached_cos = np.cos(angles).astype(np.float32)
            self._cached_sin = np.sin(angles).astype(np.float32)
            self._cached_scan_frame = msg.header.frame_id
            self.get_logger().info(
                f'Scan cached: {len(indices)} rays (skip={skip}, total={n})'
                f' frame={msg.header.frame_id}'
            )

    # ------------------------------------------------------------------

    def _check_drift(self):
        if self.map_data is None:
            self.get_logger().warn('Waiting for /map...', throttle_duration_sec=5.0)
            return
        if self.latest_scan is None:
            self.get_logger().warn(
                f'Waiting for {self.get_parameter("scan_topic").value}...',
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
            self.get_logger().warn(f'TF: {e}', throttle_duration_sec=5.0)
            return

        result = self._compute_raycast_drift(scan, tf_stamped)
        if result is None:
            self.get_logger().warn('Not enough evaluable rays', throttle_duration_sec=5.0)
            return

        drift_ratio, match_ratio, dynamic_ratio, n_eval, n_drift, n_dynamic = result

        self.drift_ratio_pub.publish(Float32(data=float(drift_ratio)))
        self.dynamic_ratio_pub.publish(Float32(data=float(dynamic_ratio)))

        # --- Calibration ---
        if self.baseline_drift is None:
            self._calib_buffer.append(drift_ratio)
            self.get_logger().info(
                f'[CALIBRATING {len(self._calib_buffer)}/{self.calibration_samples}]'
                f' drift={drift_ratio:.1%} dynamic={dynamic_ratio:.1%} eval={n_eval}'
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

        warn_thr = self.baseline_drift + self.drift_increase_warn
        alert_thr = self.baseline_drift + self.drift_increase_alert
        is_above_warn = drift_ratio >= warn_thr or drift_ratio >= self.drift_warn_threshold
        is_above_alert = drift_ratio >= alert_thr or drift_ratio >= self.drift_alert_threshold

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
                f' drift={drift_ratio:.1%} base={self.baseline_drift:.1%}'
                f' +{drift_increase:.1%} dynamic={dynamic_ratio:.1%}'
                f' — Please reset pose!'
            )

        if self.is_drifted:
            level = 'DRIFT_ALERT'
        elif is_above_warn:
            level = 'DRIFT_WARN'
        else:
            level = 'OK'

        status = (
            f'{level}'
            f' | drift={drift_ratio:.1%}'
            f' base={self.baseline_drift:.1%}'
            f' +{drift_increase:.1%}'
            f' | dynamic={dynamic_ratio:.1%}'
            f' eval={n_eval}'
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

    def _compute_raycast_drift(self, scan: LaserScan, tf_stamped):
        """
        So sánh range thực tế vs range kỳ vọng từ map bằng ray casting vectorized.

        Trả về:
          (drift_ratio, match_ratio, dynamic_ratio, n_eval, n_drift, n_dynamic)
          hoặc None nếu không đủ tia để đánh giá.
        """
        # --- Lấy dữ liệu scan (subsampled) ---
        n_full = len(scan.ranges)
        skip = self.ray_skip
        indices = np.arange(0, n_full, skip)

        d_actual = np.array(scan.ranges, dtype=np.float32)[indices]

        # Chỉ lấy tia có return hợp lệ (không phải max_range / inf)
        valid_hit = (
            np.isfinite(d_actual)
            & (d_actual >= scan.range_min)
            & (d_actual < scan.range_max * 0.98)
        )
        if valid_hit.sum() < self.min_evaluable_rays:
            return None

        # --- Pose của laser trong map frame ---
        t = tf_stamped.transform.translation
        q = tf_stamped.transform.rotation
        laser_x = float(t.x)
        laser_y = float(t.y)
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y ** 2 + q.z ** 2),
        )

        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        # Rotate cached cos/sin bằng yaw của robot
        # global_angle = yaw + local_angle
        # cos(yaw+a) = cos(yaw)cos(a) - sin(yaw)sin(a)
        cos_global = cos_yaw * self._cached_cos - sin_yaw * self._cached_sin
        sin_global = sin_yaw * self._cached_cos + cos_yaw * self._cached_sin

        # --- Ray casting vectorized ---
        res = self.map_info.resolution
        ox = self.map_info.origin.position.x
        oy = self.map_info.origin.position.y
        h, w = self.map_data.shape

        cast_range = min(float(scan.range_max), self.max_cast_range)
        n_steps = int(cast_range / res) + 1
        t_arr = (np.arange(1, n_steps + 1, dtype=np.float32) * res)  # (M,)

        N = len(indices)
        # N×M tọa độ endpoints của từng bước trên từng tia
        x_all = laser_x + np.outer(cos_global, t_arr)  # N×M
        y_all = laser_y + np.outer(sin_global, t_arr)  # N×M

        mx_all = np.floor((x_all - ox) / res).astype(np.int32)  # N×M
        my_all = np.floor((y_all - oy) / res).astype(np.int32)  # N×M

        in_bounds = (
            (mx_all >= 0) & (mx_all < w)
            & (my_all >= 0) & (my_all < h)
        )

        mx_safe = np.clip(mx_all, 0, w - 1)
        my_safe = np.clip(my_all, 0, h - 1)

        # Tra cứu giá trị map dọc theo từng tia
        map_vals = self.map_data[my_safe, mx_safe].astype(np.int16)  # N×M
        map_vals[~in_bounds] = -1  # ngoài map = unknown

        # Tìm bước đầu tiên chạm ô occupied
        occ = map_vals >= self.occupancy_threshold  # N×M bool
        ray_hits_wall = occ.any(axis=1)  # (N,) — tia có gặp tường không?
        first_idx = np.argmax(occ, axis=1)  # (N,) index bước đầu tiên
        d_map = (first_idx + 1).astype(np.float32) * res  # khoảng cách kỳ vọng (m)
        d_map[~ray_hits_wall] = cast_range + 1.0  # không có tường → "vô cực"

        # --- Phân loại từng tia ---
        tol = self.range_tolerance

        # Tia được đánh giá: có hit thực tế VÀ map kỳ vọng có tường
        evaluable = valid_hit & ray_hits_wall

        n_eval = int(evaluable.sum())
        if n_eval < self.min_evaluable_rays:
            return None

        # MATCH: tia thực tế gặp tường đúng vị trí kỳ vọng
        match_mask = evaluable & (np.abs(d_actual - d_map) <= tol)

        # DRIFT: tia đi xa hơn tường kỳ vọng → robot ở sai vị trí!
        drift_mask = evaluable & (d_actual > d_map + tol)

        # DYNAMIC: tia gặp vật thể chắn trước tường → bỏ qua (không phải drift)
        dynamic_mask = valid_hit & ray_hits_wall & (d_actual < d_map - tol)

        # Dynamic không thuộc evaluable vẫn cần log
        n_dynamic_all = int((valid_hit & (d_actual < d_map - tol)).sum())

        n_match = int(match_mask.sum())
        n_drift = int(drift_mask.sum())
        n_dynamic = int(dynamic_mask.sum())

        drift_ratio = n_drift / n_eval
        match_ratio = n_match / n_eval

        # Dynamic ratio: tỉ lệ tia bị chặn so với tổng tia valid
        n_valid = int(valid_hit.sum())
        dynamic_ratio = n_dynamic_all / max(n_valid, 1)

        return drift_ratio, match_ratio, dynamic_ratio, n_eval, n_drift, n_dynamic


def main(args=None):
    rclpy.init(args=args)
    node = MapDriftRaycastNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
