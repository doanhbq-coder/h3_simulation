"""
Map Drift Monitor — Pose-Correction approach.

Thay vì đo `drift_ratio` vô hướng tại pose hiện tại (cách cũ, dễ sai), node này
ước lượng ĐỘ LỆCH ĐỊNH VỊ theo đơn vị vật lý (mét / radian) bằng cách trả lời câu
hỏi: "pose nào khớp map tốt nhất, và nó cách pose hiện tại bao xa?".

Luồng 5 tầng (xem ARCHITECTURE.md):
  [1] Distance field (EDT) từ /map  → likelihood field
  [2] Lọc tia "ổn định"             → loại nhiễu động / vùng chưa map
  [3] Local pose search (dx,dy,dθ)  → correction c* = argmax điểm khớp
  [4] Confidence (curvature đỉnh)   → hạ tin cậy ở hành lang đối xứng / không gian mở
  [5] State machine + lọc thời gian → OK / WARN / ALERT (hysteresis)

Ưu điểm:
  - Output theo mét/độ → ngưỡng chuyển được giữa các map, không cần calibration.
  - Vật thể động làm giảm điểm ở MỌI offset như nhau → không dịch đỉnh → miễn nhiễm.
  - Phát hiện cả lệch xoay (yaw).
  - Trả về correction vector dùng được (có thể tự reset /initialpose).
"""

import math

import numpy as np
import rclpy
import rclpy.duration
from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32, String
from tf2_ros import ConnectivityException, ExtrapolationException, LookupException
import tf2_ros
from visualization_msgs.msg import Marker

try:
    from scipy.ndimage import distance_transform_edt
    SCIPY_AVAILABLE = True
except ImportError:
    SCIPY_AVAILABLE = False


def yaw_from_quaternion(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y ** 2 + q.z ** 2),
    )


class MapDriftCorrectorNode(Node):
    def __init__(self):
        super().__init__('map_drift_corrector_node')

        # --- I/O ---
        self.declare_parameter('scan_topic', '/scan_top')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('check_rate', 2.0)

        # --- Likelihood field ---
        self.declare_parameter('sigma', 0.20)
        self.declare_parameter('occupancy_threshold', 50)
        self.declare_parameter('max_obstacle_distance', 2.0)

        # --- Lọc tia ổn định ---
        self.declare_parameter('stable_gate', 0.50)
        self.declare_parameter('min_stable_rays', 40)
        self.declare_parameter('ray_skip', 2)

        # --- Local pose search ---
        self.declare_parameter('search_xy', 0.50)
        self.declare_parameter('search_yaw', 0.175)       # ~10°
        self.declare_parameter('coarse_step_xy', 0.10)
        self.declare_parameter('coarse_step_yaw', 0.0524)  # ~3°
        self.declare_parameter('fine_refine', True)
        # Số lần tái định tâm khi đỉnh ghim ở biên (mở rộng tầm đo mà không cần
        # lưới khổng lồ). Tầm đo hiệu dụng ≈ max_search_iters * search_xy.
        self.declare_parameter('max_search_iters', 4)

        # --- Ngưỡng vật lý (mét / radian) ---
        self.declare_parameter('warn_drift_m', 0.15)
        self.declare_parameter('alert_drift_m', 0.30)
        self.declare_parameter('warn_drift_yaw', 0.087)   # ~5°
        self.declare_parameter('alert_drift_yaw', 0.175)  # ~10°
        self.declare_parameter('exit_drift_m', 0.10)
        self.declare_parameter('min_confidence', 0.5)

        # --- Lọc thời gian ---
        self.declare_parameter('warn_consecutive', 3)
        self.declare_parameter('alert_consecutive', 5)
        self.declare_parameter('exit_consecutive', 5)

        # --- Auto-correct ---
        self.declare_parameter('auto_correct', False)
        self.declare_parameter('auto_correct_min_confidence', 0.8)

        # --- Debug ---
        self.declare_parameter('publish_score_grid', False)
        # Publish laser giả định (xanh: theo pose khớp-nhất) + laser hiện tại (đỏ)
        # để so sánh trực quan trên RViz.
        self.declare_parameter('publish_debug_markers', True)
        self.declare_parameter('marker_point_size', 0.04)

        gp = self.get_parameter
        scan_topic = gp('scan_topic').value
        map_topic = gp('map_topic').value
        self.map_frame = gp('map_frame').value
        self.base_frame = gp('base_frame').value
        self.odom_frame = gp('odom_frame').value
        self.check_rate = gp('check_rate').value
        self.sigma = gp('sigma').value
        self.occupancy_threshold = gp('occupancy_threshold').value
        self.max_obstacle_distance = gp('max_obstacle_distance').value
        self.stable_gate = gp('stable_gate').value
        self.min_stable_rays = gp('min_stable_rays').value
        self.ray_skip = max(1, gp('ray_skip').value)
        self.search_xy = gp('search_xy').value
        self.search_yaw = gp('search_yaw').value
        self.coarse_step_xy = gp('coarse_step_xy').value
        self.coarse_step_yaw = gp('coarse_step_yaw').value
        self.fine_refine = gp('fine_refine').value
        self.max_search_iters = max(1, gp('max_search_iters').value)
        self.warn_drift_m = gp('warn_drift_m').value
        self.alert_drift_m = gp('alert_drift_m').value
        self.warn_drift_yaw = gp('warn_drift_yaw').value
        self.alert_drift_yaw = gp('alert_drift_yaw').value
        self.exit_drift_m = gp('exit_drift_m').value
        self.min_confidence = gp('min_confidence').value
        self.warn_consecutive = gp('warn_consecutive').value
        self.alert_consecutive = gp('alert_consecutive').value
        self.exit_consecutive = gp('exit_consecutive').value
        self.auto_correct = gp('auto_correct').value
        self.auto_correct_min_confidence = gp('auto_correct_min_confidence').value
        self.publish_score_grid = gp('publish_score_grid').value
        self.publish_debug_markers = gp('publish_debug_markers').value
        self.marker_point_size = gp('marker_point_size').value

        # --- State ---
        self.map_data: np.ndarray | None = None
        self.map_info = None
        self.distance_field: np.ndarray | None = None  # D[y,x] (m)
        self.latest_scan: LaserScan | None = None
        self._last_corrected_pose: PoseStamped | None = None

        self.state = 'OK'
        self.warn_count = 0
        self.alert_count = 0
        self.exit_count = 0

        # Cache góc tia (theo scan frame)
        self._cached_scan_frame: str | None = None
        self._cached_angles: np.ndarray | None = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(OccupancyGrid, map_topic, self._map_cb, map_qos)
        self.create_subscription(LaserScan, scan_topic, self._scan_cb, 10)

        self.drift_xy_pub = self.create_publisher(Float32, '/map_drift/drift_xy', 10)
        self.drift_yaw_pub = self.create_publisher(Float32, '/map_drift/drift_yaw', 10)
        # Tỉ lệ lệch chuẩn hoá (0..1+): 1.0 = đúng mức ALERT cần setpose. Dễ debug.
        self.severity_pub = self.create_publisher(Float32, '/map_drift/drift_severity', 10)
        # Tỉ lệ tia khớp map (0..1): thấp → dữ liệu nghèo / robot lệch nặng.
        self.stable_ratio_pub = self.create_publisher(Float32, '/map_drift/stable_ratio', 10)
        # Mức khớp scan-map tại pose tốt nhất (0..1): chất lượng khớp sau khi sửa.
        self.match_quality_pub = self.create_publisher(Float32, '/map_drift/match_quality', 10)
        self.confidence_pub = self.create_publisher(Float32, '/map_drift/confidence', 10)
        self.correction_pub = self.create_publisher(PoseStamped, '/map_drift/correction', 10)
        self.state_pub = self.create_publisher(String, '/map_drift/state', 10)
        self.alert_pub = self.create_publisher(Bool, '/map_drift/alert', 10)
        self.reset_request_pub = self.create_publisher(Bool, '/map_drift/reset_pose_request', 10)
        self.initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10
        )
        if self.publish_score_grid:
            self.score_grid_pub = self.create_publisher(
                OccupancyGrid, '/map_drift/score_grid', 1
            )
        if self.publish_debug_markers:
            # Laser giả định theo pose khớp-nhất (xanh) — nên dán vào tường map
            self.scan_corrected_pub = self.create_publisher(
                Marker, '/map_drift/scan_corrected', 1
            )
            # Laser theo pose hiện tại (đỏ) — lệch khỏi tường nếu robot bị drift
            self.scan_current_pub = self.create_publisher(
                Marker, '/map_drift/scan_current', 1
            )

        _wall_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(1.0 / self.check_rate, self._check, clock=_wall_clock)

        if not SCIPY_AVAILABLE:
            self.get_logger().error('scipy không có! Cài: pip install scipy — node sẽ không chạy.')

        self.get_logger().info(
            f'MapDriftCorrector started | sigma={self.sigma}m'
            f' search=±{self.search_xy}m/±{math.degrees(self.search_yaw):.0f}°'
            f' | warn>{self.warn_drift_m}m alert>{self.alert_drift_m}m'
            f' | auto_correct={self.auto_correct}'
        )

    # ================================================================
    # [1] Map + distance field
    # ================================================================

    def _map_cb(self, msg: OccupancyGrid):
        self.map_data = np.array(msg.data, dtype=np.int8).reshape(
            msg.info.height, msg.info.width
        )
        self.map_info = msg.info
        n_occ = int(np.sum(self.map_data >= self.occupancy_threshold))
        self.get_logger().info(
            f'Map: {msg.info.width}x{msg.info.height} res={msg.info.resolution}m'
            f' occupied={n_occ} | building distance field...'
        )
        self._build_distance_field(self.map_data, msg.info.resolution)

    def _build_distance_field(self, map_data: np.ndarray, resolution: float):
        if not SCIPY_AVAILABLE:
            self.distance_field = None
            return
        obstacle_mask = map_data >= self.occupancy_threshold
        if not obstacle_mask.any():
            self.get_logger().warn('Map không có ô occupied nào — bỏ qua.')
            self.distance_field = None
            return
        dist_px = distance_transform_edt(~obstacle_mask)
        df = dist_px * resolution
        self.distance_field = np.minimum(df, self.max_obstacle_distance).astype(np.float32)
        self.get_logger().info(
            f'Distance field built | max={float(self.distance_field.max()):.2f}m'
            f' mean={float(self.distance_field.mean()):.2f}m'
        )

    def _scan_cb(self, msg: LaserScan):
        self.latest_scan = msg
        if msg.header.frame_id != self._cached_scan_frame:
            n = len(msg.ranges)
            idx = np.arange(0, n, self.ray_skip)
            self._cached_angles = (msg.angle_min + idx * msg.angle_increment).astype(np.float32)
            self._cached_scan_frame = msg.header.frame_id
            self.get_logger().info(
                f'Scan cached: {len(idx)} rays (skip={self.ray_skip}, total={n})'
                f' frame={msg.header.frame_id}'
            )

    # ================================================================
    # Main loop
    # ================================================================

    def _check(self):
        if self.distance_field is None:
            self.get_logger().warn('Chờ /map (distance field)...', throttle_duration_sec=5.0)
            return
        if self.latest_scan is None:
            self.get_logger().warn('Chờ scan...', throttle_duration_sec=5.0)
            return

        scan = self.latest_scan
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, scan.header.frame_id, Time(),
                timeout=rclpy.duration.Duration(seconds=0.1),
            )
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f'TF: {e}', throttle_duration_sec=5.0)
            return

        # Pose của scan frame trong map
        t = tf.transform.translation
        x0, y0 = float(t.x), float(t.y)
        yaw0 = yaw_from_quaternion(tf.transform.rotation)

        # [2] Lọc tia ổn định → toạ độ local (trong scan frame)
        local = self._select_stable_rays(scan, x0, y0, yaw0)
        if local is None:
            self.get_logger().warn('Không đủ tia ổn định', throttle_duration_sec=5.0)
            self._publish_low_confidence()
            return
        lx, ly, n_stable, total_valid = local

        # [3] Local pose search (có tái định tâm)
        (dx, dy, dyaw), score_peak, conf, hit_boundary = self._search_correction(
            lx, ly, x0, y0, yaw0
        )

        drift_m = math.hypot(dx, dy)
        drift_yaw = abs(dyaw)

        # --- Tỉ lệ lệch chuẩn hoá (0..1+): 1.0 = đúng mức ALERT ---
        # Lấy max giữa lệch tịnh tiến và lệch xoay (cái nào nặng hơn quyết định).
        severity = max(
            drift_m / max(self.alert_drift_m, 1e-6),
            drift_yaw / max(self.alert_drift_yaw, 1e-6),
        )
        stable_ratio = n_stable / max(total_valid, 1)

        # [4]+[5] state machine
        self._update_state(drift_m, drift_yaw, conf, hit_boundary)

        # Publish
        self.drift_xy_pub.publish(Float32(data=float(drift_m)))
        self.drift_yaw_pub.publish(Float32(data=float(drift_yaw)))
        self.severity_pub.publish(Float32(data=float(severity)))
        self.stable_ratio_pub.publish(Float32(data=float(stable_ratio)))
        self.match_quality_pub.publish(Float32(data=float(score_peak)))
        self.confidence_pub.publish(Float32(data=float(conf)))
        self.state_pub.publish(String(data=self.state))
        self.alert_pub.publish(Bool(data=self.state == 'ALERT'))

        corrected = self._make_correction_pose(x0, y0, yaw0, dx, dy, dyaw)
        self.correction_pub.publish(corrected)

        if self.publish_debug_markers:
            self._publish_debug_markers(lx, ly, x0, y0, yaw0, dx, dy, dyaw)

        status = (
            f'{self.state} | drift={drift_m:.3f}m yaw={math.degrees(drift_yaw):.1f}°'
            f' severity={severity:.0%}'
            f' | conf={conf:.2f} match={score_peak:.2f}'
            f' stable={n_stable}/{total_valid} ({stable_ratio:.0%})'
            f' | c=({dx:+.2f},{dy:+.2f},{math.degrees(dyaw):+.1f}°)'
            f'{" [BOUNDARY]" if hit_boundary else ""}'
        )
        if self.state == 'ALERT':
            self.get_logger().error(status)
        elif self.state == 'WARN':
            self.get_logger().warn(status)
        else:
            self.get_logger().info(status, throttle_duration_sec=3.0)

    # ================================================================
    # [2] Lọc tia ổn định
    # ================================================================

    def _select_stable_rays(self, scan: LaserScan, x0, y0, yaw0):
        """Trả về (lx, ly, n_stable, total_valid) — toạ độ tia ổn định (local), hoặc None."""
        ranges = np.array(scan.ranges, dtype=np.float32)[::self.ray_skip]
        angles = self._cached_angles
        if angles is None or len(angles) != len(ranges):
            n = len(scan.ranges)
            idx = np.arange(0, n, self.ray_skip)
            angles = (scan.angle_min + idx * scan.angle_increment).astype(np.float32)

        valid = (
            np.isfinite(ranges)
            & (ranges >= scan.range_min)
            & (ranges < scan.range_max * 0.98)
        )
        total_valid = int(valid.sum())
        if total_valid < self.min_stable_rays:
            return None

        ranges = ranges[valid]
        angles = angles[valid]

        # Toạ độ local trong scan frame
        lx = ranges * np.cos(angles)
        ly = ranges * np.sin(angles)

        # Project sang map tại pose hiện tại để tra distance field
        c, s = math.cos(yaw0), math.sin(yaw0)
        px = x0 + c * lx - s * ly
        py = y0 + s * lx + c * ly
        d = self._lookup_distance(px, py)

        # Giữ tia có endpoint sát tường mapped (loại unknown / vùng thay đổi)
        stable = d <= self.stable_gate
        if stable.sum() < self.min_stable_rays:
            return None
        return lx[stable], ly[stable], int(stable.sum()), total_valid

    # ================================================================
    # [3] Local pose search
    # ================================================================

    def _search_correction(self, lx, ly, x0, y0, yaw0):
        """
        Quét lưới (dx,dy,dyaw) tìm correction tối ưu, có tái định tâm khi đỉnh ghim
        ở biên (mở rộng tầm đo). Trả về ((dx,dy,dyaw), score_peak, confidence, boundary).
        """
        nx = max(1, int(round(self.search_xy / self.coarse_step_xy)))
        offs_xy = np.linspace(-self.search_xy, self.search_xy, 2 * nx + 1)
        nyaw = max(1, int(round(self.search_yaw / self.coarse_step_yaw)))
        offs_yaw = np.linspace(-self.search_yaw, self.search_yaw, 2 * nyaw + 1)
        last_idx = len(offs_xy) - 1

        # Tâm tích luỹ — dời theo mỗi lần đụng biên
        cx_acc, cy_acc = 0.0, 0.0
        boundary = False
        best_grid = None
        best_iy = best_ix = 0
        best_score = -1.0
        best_dyaw = 0.0

        for it in range(self.max_search_iters):
            g_grid, g_iy, g_ix, g_score, g_dyaw = self._grid_pass(
                lx, ly, x0 + cx_acc, y0 + cy_acc, yaw0, offs_xy, offs_yaw
            )
            best_grid, best_iy, best_ix = g_grid, g_iy, g_ix
            best_score, best_dyaw = g_score, g_dyaw

            on_edge = g_ix in (0, last_idx) or g_iy in (0, last_idx)
            if on_edge and it < self.max_search_iters - 1:
                # Dời tâm tới đỉnh biên rồi quét lại (không dời yaw)
                cx_acc += float(offs_xy[g_ix])
                cy_acc += float(offs_xy[g_iy])
                continue
            boundary = on_edge
            break

        dx = cx_acc + float(offs_xy[best_ix])
        dy = cy_acc + float(offs_xy[best_iy])
        dyaw = best_dyaw

        # Nội suy parabol sub-cell (chỉ khi đỉnh nằm trong, không ở biên)
        if self.fine_refine and best_grid is not None and not boundary:
            dx = cx_acc + self._parabolic_refine(
                best_grid[best_iy, :], best_ix, offs_xy, float(offs_xy[best_ix])
            )
            dy = cy_acc + self._parabolic_refine(
                best_grid[:, best_ix], best_iy, offs_xy, float(offs_xy[best_iy])
            )

        conf = self._estimate_confidence(best_grid, best_iy, best_ix, best_score, boundary)
        return (dx, dy, dyaw), best_score, conf, boundary

    def _grid_pass(self, lx, ly, cx, cy, yaw0, offs_xy, offs_yaw):
        """Một lượt quét lưới (dx,dy,dyaw) quanh tâm (cx,cy). Trả về đỉnh tốt nhất."""
        inv2s2 = 1.0 / (2.0 * self.sigma ** 2)
        best_grid = None
        best_iy = best_ix = 0
        best_score = -1.0
        best_dyaw = 0.0
        for dyaw in offs_yaw:
            yaw = yaw0 + dyaw
            c, s = math.cos(yaw), math.sin(yaw)
            rx = c * lx - s * ly
            ry = s * lx + c * ly
            grid = np.empty((len(offs_xy), len(offs_xy)), dtype=np.float32)
            for iy, dy in enumerate(offs_xy):
                py = (cy + dy) + ry
                for ix, dx in enumerate(offs_xy):
                    px = (cx + dx) + rx
                    d = self._lookup_distance(px, py)
                    grid[iy, ix] = np.mean(np.exp(-d * d * inv2s2))
            iy, ix = np.unravel_index(int(np.argmax(grid)), grid.shape)
            if grid[iy, ix] > best_score:
                best_score = float(grid[iy, ix])
                best_grid, best_iy, best_ix = grid, iy, ix
                best_dyaw = float(dyaw)
        return best_grid, best_iy, best_ix, best_score, best_dyaw

    @staticmethod
    def _parabolic_refine(line: np.ndarray, i: int, offs: np.ndarray, fallback: float) -> float:
        """Nội suy đỉnh parabol từ 3 điểm quanh index i. Trả về offset tinh chỉnh."""
        if i <= 0 or i >= len(line) - 1:
            return fallback
        y0, y1, y2 = float(line[i - 1]), float(line[i]), float(line[i + 1])
        denom = (y0 - 2 * y1 + y2)
        if abs(denom) < 1e-9:
            return fallback
        delta = 0.5 * (y0 - y2) / denom  # ∈ (-1,1) cell
        step = offs[1] - offs[0]
        return float(offs[i] + delta * step)

    # ================================================================
    # [4] Confidence (curvature đỉnh)
    # ================================================================

    def _estimate_confidence(self, grid, iy, ix, peak, boundary) -> float:
        """
        Confidence ∈ [0,1] tổng hợp từ:
          - độ cao đỉnh (peak score): khớp tốt thật sự
          - độ nhọn đỉnh theo 2 trục (curvature): hướng phẳng → không quan sát được

        Trường hợp `boundary` (đỉnh vẫn ghim ở biên sau khi đã tái định tâm hết số
        lần cho phép → drift rất lớn): không tính được curvature, nhưng peak cao +
        nhiều tia khớp đã là bằng chứng mạnh → chỉ dựa vào peak_conf để VẪN báo được.
        """
        if grid is None or peak <= 1e-6:
            return 0.0

        # Đỉnh phải đủ cao mới đáng tin (khớp tốt)
        peak_conf = float(np.clip((peak - 0.3) / 0.4, 0.0, 1.0))

        if boundary:
            return peak_conf

        # Độ nhọn theo từng trục (curvature ≈ peak - hàng xóm)
        def curvature(line, i):
            if i <= 0 or i >= len(line) - 1:
                return 0.0
            return max(0.0, 2 * line[i] - line[i - 1] - line[i + 1])

        cx = curvature(grid[iy, :], ix)
        cy = curvature(grid[:, ix], iy)
        # Chuẩn hoá theo peak; lấy trục yếu nhất (hướng kém quan sát nhất quyết định)
        sharp = min(cx, cy) / max(peak, 1e-6)
        # sharp ~0 (phẳng) → conf thấp; ~0.1+ (nhọn) → conf cao
        sharp_conf = float(np.clip(sharp / 0.08, 0.0, 1.0))

        return float(sharp_conf * peak_conf)

    # ================================================================
    # [5] State machine + lọc thời gian
    # ================================================================

    def _update_state(self, drift_m, drift_yaw, conf, hit_boundary):
        confident = conf >= self.min_confidence
        over_warn = drift_m >= self.warn_drift_m or drift_yaw >= self.warn_drift_yaw
        over_alert = (
            drift_m >= self.alert_drift_m
            or drift_yaw >= self.alert_drift_yaw
            or hit_boundary  # đụng biên lưới → drift lớn hơn vùng quét
        )

        if confident and over_alert:
            self.alert_count += 1
            self.warn_count += 1
            self.exit_count = 0
        elif confident and over_warn:
            self.warn_count += 1
            self.alert_count = max(0, self.alert_count - 1)
            self.exit_count += 1
        else:
            self.warn_count = max(0, self.warn_count - 1)
            self.alert_count = max(0, self.alert_count - 1)
            self.exit_count += 1

        prev_state = self.state
        if self.state == 'ALERT':
            # thoát ALERT khi drift nhỏ liên tục (hysteresis)
            if drift_m < self.exit_drift_m and self.exit_count >= self.exit_consecutive:
                self.state = 'OK'
                self.warn_count = self.alert_count = 0
        else:
            if self.alert_count >= self.alert_consecutive:
                self.state = 'ALERT'
                self.exit_count = 0
            elif self.warn_count >= self.warn_consecutive:
                self.state = 'WARN'
            elif self.warn_count == 0:
                self.state = 'OK'

        # Cạnh lên ALERT → reset request + (tuỳ chọn) auto-correct
        if self.state == 'ALERT' and prev_state != 'ALERT':
            self.reset_request_pub.publish(Bool(data=True))
            self.get_logger().error(
                f'MAP DRIFT DETECTED! drift={drift_m:.3f}m'
                f' yaw={math.degrees(drift_yaw):.1f}° conf={conf:.2f}'
            )
            self._maybe_auto_correct(conf)

    def _maybe_auto_correct(self, conf):
        if not self.auto_correct:
            return
        if conf < self.auto_correct_min_confidence:
            self.get_logger().warn(
                f'auto_correct bỏ qua: confidence {conf:.2f} <'
                f' {self.auto_correct_min_confidence}'
            )
            return
        if self._last_corrected_pose is None:
            return
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose = self._last_corrected_pose.pose
        # Covariance vừa phải để slam_toolbox tin nhưng vẫn lọc
        cov = [0.0] * 36
        cov[0] = cov[7] = 0.25 ** 2   # x, y
        cov[35] = (math.radians(10)) ** 2  # yaw
        msg.pose.covariance = cov
        self.initialpose_pub.publish(msg)
        self.get_logger().error('AUTO-CORRECT: đã publish /initialpose = pose + correction')

    # ================================================================
    # Helpers
    # ================================================================

    def _lookup_distance(self, px: np.ndarray, py: np.ndarray) -> np.ndarray:
        """Tra distance field tại các điểm map (m). Ngoài bounds → max_obstacle_distance."""
        res = self.map_info.resolution
        ox = self.map_info.origin.position.x
        oy = self.map_info.origin.position.y
        h, w = self.distance_field.shape
        mx = np.floor((px - ox) / res).astype(np.int32)
        my = np.floor((py - oy) / res).astype(np.int32)
        out = np.full(px.shape, self.max_obstacle_distance, dtype=np.float32)
        ok = (mx >= 0) & (mx < w) & (my >= 0) & (my < h)
        if ok.any():
            out[ok] = self.distance_field[my[ok], mx[ok]]
        return out

    def _make_correction_pose(self, x0, y0, yaw0, dx, dy, dyaw) -> PoseStamped:
        msg = PoseStamped()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        cx = x0 + dx
        cy = y0 + dy
        cyaw = yaw0 + dyaw
        msg.pose.position.x = cx
        msg.pose.position.y = cy
        msg.pose.orientation.z = math.sin(cyaw / 2.0)
        msg.pose.orientation.w = math.cos(cyaw / 2.0)
        self._last_corrected_pose = msg
        return msg

    def _publish_debug_markers(self, lx, ly, x0, y0, yaw0, dx, dy, dyaw):
        """
        Publish 2 đám điểm laser (POINTS marker) trong map frame:
          - scan_current  (ĐỎ):  tia ổn định chiếu theo pose HIỆN TẠI
          - scan_corrected (XANH): cùng tia đó chiếu theo pose KHỚP-NHẤT (đã sửa)
        Khi lệch: đỏ nằm lệch khỏi tường, xanh dán vào tường → thấy hướng & độ lệch.
        """
        # Endpoints tại pose hiện tại
        c0, s0 = math.cos(yaw0), math.sin(yaw0)
        cur_x = x0 + c0 * lx - s0 * ly
        cur_y = y0 + s0 * lx + c0 * ly

        # Endpoints tại pose khớp-nhất (yaw0+dyaw, tịnh tiến x0+dx, y0+dy)
        yb = yaw0 + dyaw
        cb, sb = math.cos(yb), math.sin(yb)
        cor_x = (x0 + dx) + cb * lx - sb * ly
        cor_y = (y0 + dy) + sb * lx + cb * ly

        self.scan_current_pub.publish(
            self._points_marker(cur_x, cur_y, 'scan_current', 0, (1.0, 0.1, 0.1))
        )
        self.scan_corrected_pub.publish(
            self._points_marker(cor_x, cor_y, 'scan_corrected', 1, (0.1, 1.0, 0.2))
        )

    def _points_marker(self, px, py, ns, mid, rgb) -> Marker:
        m = Marker()
        m.header.frame_id = self.map_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = ns
        m.id = mid
        m.type = Marker.POINTS
        m.action = Marker.ADD
        m.scale.x = m.scale.y = float(self.marker_point_size)
        m.color.r, m.color.g, m.color.b = float(rgb[0]), float(rgb[1]), float(rgb[2])
        m.color.a = 1.0
        m.pose.orientation.w = 1.0
        m.points = [Point(x=float(x), y=float(y), z=0.0) for x, y in zip(px, py)]
        return m

    def _publish_low_confidence(self):
        self.confidence_pub.publish(Float32(data=0.0))
        self.stable_ratio_pub.publish(Float32(data=0.0))
        self.match_quality_pub.publish(Float32(data=0.0))
        self.state_pub.publish(String(data=self.state))


def main(args=None):
    rclpy.init(args=args)
    node = MapDriftCorrectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
