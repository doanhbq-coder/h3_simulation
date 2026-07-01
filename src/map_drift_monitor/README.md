# map_drift_monitor

Package ROS2 giám sát độ lệch map (map drift) của robot trong quá trình định vị. Khi robot bị mất định vị hoặc pose ước lượng lệch khỏi vị trí thực trên map, node sẽ phát cảnh báo và yêu cầu reset pose.

---

## ⭐ Node khuyến nghị: `map_drift_corrector_node` (Pose-Correction)

3 node gốc bên dưới (`monitor` / `likelihood` / `raycast`) đo một `drift_ratio` vô
hướng tại pose hiện tại — dễ báo nhầm/bỏ sót vì trộn lẫn nhiễu động, vùng chưa map
và lệch định vị thật, lại cần calibration baseline mong manh.

**`map_drift_corrector_node`** dùng hướng tiếp cận mới: ước lượng độ lệch theo
**mét/độ** bằng cách tìm pose khớp map tốt nhất quanh pose hiện tại (local pose
search trên likelihood field). Ưu điểm: không cần calibration, miễn nhiễm vật thể
động, phát hiện cả lệch xoay, và trả về correction vector dùng được.

```bash
ros2 launch map_drift_monitor map_drift_corrector.launch.py use_sim_time:=true
```

Chi tiết thiết kế & luồng 5 tầng: xem [ARCHITECTURE.md](ARCHITECTURE.md).
Tham số: [config/map_drift_corrector.yaml](config/map_drift_corrector.yaml).

---

## Các node gốc (legacy)

> Phần dưới mô tả 3 node gốc theo phương pháp `drift_ratio`. Giữ lại để đối chiếu.

---

## Nguyên lý hoạt động

### 1. Scan-to-Map Matching

Mỗi chu kỳ, node thực hiện:

```
LiDAR scan  ──TF──►  map frame  ──►  chiếu lên OccupancyGrid  ──►  đếm hits
```

1. Lấy transform `laser_frame → map` từ TF tree (do SLAM/AMCL cung cấp)
2. Project từng tia laser thành tọa độ (x, y) trong map frame
3. Chuyển tọa độ thành chỉ số cell trong OccupancyGrid
4. Kiểm tra cell đó (và 8 cells lân cận 3×3) có phải **occupied** không
5. Tính tỉ lệ:

```
match_ratio  = occupied_hits / total_valid_rays
drift_ratio  = 1 − match_ratio
```

**Ý nghĩa:** Khi robot định vị đúng, tia laser kết thúc tại tường → cell occupied. Khi pose lệch, tia laser rơi vào vùng free/unknown → drift_ratio tăng.

```
Pose đúng:                      Pose lệch:
  map       scan                  map       scan
  ████ ←●  (hit)                  ████      ●→  (miss)
  ████ ←●  (hit)                       ←●      (miss)
  ████ ←●  (hit)                  ████ ←●  (hit)
  drift ≈ 13%                     drift ≈ 45%
```

### 2. Adaptive Baseline

Môi trường thực luôn có noise nhất định (không gian mở, vật thể động, lỗi discretization). Vì vậy node **không dùng ngưỡng tuyệt đối** mà tự đo baseline khi khởi động:

```
Giai đoạn calibration (10 samples đầu ≈ 5 giây):
  → Đo drift_ratio tại vị trí đúng
  → baseline = trung bình 10 samples
  → warn_threshold  = baseline + 15%
  → alert_threshold = baseline + 22%
```

Ví dụ nếu baseline = 13.2%:
| Mức | Ngưỡng thực tế | Ý nghĩa |
|-----|----------------|---------|
| OK | < 28.2% | Định vị bình thường |
| WARN | ≥ 28.2% | Bắt đầu lệch |
| ALERT | ≥ 35.2% liên tiếp 3 lần | Xác nhận lệch map, cần reset |

### 3. Consecutive Filter

Để tránh false positive từ spike nhiễu ngắn hạn (người đi qua, cửa mở):

```
consecutive_alerts_required = 3

check 1: drift=36% ≥ threshold → consecutive=1
check 2: drift=37% ≥ threshold → consecutive=2
check 3: drift=38% ≥ threshold → consecutive=3  ← ALERT triggered
```

Nếu giữa chừng có 1 check dưới ngưỡng, counter giảm 1 (không reset về 0).

---

## Cài đặt & Build

```bash
cd ~/Documents/h3_simulation
colcon build --packages-select map_drift_monitor --symlink-install
source install/setup.bash
```

---

## Khởi chạy

### Chạy đơn lẻ

```bash
ros2 launch map_drift_monitor map_drift_monitor.launch.py
```

### Với simulation (có /clock)

```bash
ros2 launch map_drift_monitor map_drift_monitor.launch.py use_sim_time:=true
```

### Tích hợp vào navigation launch

Thêm vào file launch của navigation:

```python
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

drift_monitor = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('map_drift_monitor'),
                     'launch', 'map_drift_monitor.launch.py')
    ),
    launch_arguments={'use_sim_time': use_sim_time}.items(),
)
```

---

## Topics

### Subscribe

| Topic | Type | Mô tả |
|-------|------|-------|
| `/map` | `nav_msgs/OccupancyGrid` | Map tĩnh (QoS: TRANSIENT_LOCAL) |
| `/scan_top` | `sensor_msgs/LaserScan` | LiDAR scan (configurable) |
| TF: `map → laser_frame` | — | Pose robot trong map |

### Publish

| Topic | Type | Mô tả |
|-------|------|-------|
| `/map_drift/drift_ratio` | `std_msgs/Float32` | Tỉ lệ mismatch hiện tại (0.0–1.0) |
| `/map_drift/baseline` | `std_msgs/Float32` | Baseline đã calibrate |
| `/map_drift/drift_increase` | `std_msgs/Float32` | Mức tăng so với baseline |
| `/map_drift/alert` | `std_msgs/Bool` | `true` khi xác nhận lệch map |
| `/map_drift/status` | `std_msgs/String` | Chuỗi trạng thái chi tiết |
| `/map_drift/reset_pose_request` | `std_msgs/Bool` | Phát `true` khi cần reset pose |

### Theo dõi realtime

```bash
# Xem trạng thái tổng quát
ros2 topic echo /map_drift/status

# Xem drift ratio dạng số
ros2 topic echo /map_drift/drift_ratio

# Xem khi nào có alert
ros2 topic echo /map_drift/alert

# Plot drift_ratio và baseline trên rqt
rqt
# Thêm: Plugins → Visualization → Plot → /map_drift/drift_ratio, /map_drift/baseline
```

---

## Cấu hình

File: `config/map_drift_monitor.yaml`

```yaml
map_drift_monitor_node:
  ros__parameters:
    scan_topic: "/scan_top"       # Topic LiDAR
    map_topic: "/map"             # Topic map
    map_frame: "map"
    robot_frame: "base_link"

    # Calibration
    calibration_samples: 10       # Số samples để đo baseline (~5 giây ở 2Hz)

    # Ngưỡng adaptive (so với baseline)
    drift_increase_warn: 0.15     # +15% → WARN
    drift_increase_alert: 0.22    # +22% → ALERT

    # Ngưỡng tuyệt đối (fallback)
    drift_warn_threshold: 0.40
    drift_alert_threshold: 0.55

    consecutive_alerts_required: 3  # Lần liên tiếp vượt ngưỡng để xác nhận
    check_rate: 2.0                 # Hz
    min_scan_points: 50
    occupancy_threshold: 50
    neighbor_check: true            # Kiểm tra 3×3 cells quanh endpoint
```

### Điều chỉnh ngưỡng

| Trường hợp | Điều chỉnh |
|-----------|-----------|
| Quá nhiều false positive | Tăng `drift_increase_alert` (vd: 0.30) hoặc tăng `consecutive_alerts_required` |
| Phát hiện drift quá chậm | Giảm `drift_increase_alert` (vd: 0.18) hoặc giảm `consecutive_alerts_required` |
| Môi trường động (nhiều người) | Tăng `drift_increase_alert`, tăng `consecutive_alerts_required` |
| Hành lang hẹp, nhiều tường | Có thể giảm `drift_increase_warn` |

---

## Quy trình sử dụng điển hình

```
1. Khởi động navigation stack (nav2 + localization)
        ↓
2. Khởi động map_drift_monitor
        ↓
3. Đặt robot tại vị trí khởi động ĐÚNG (set initial pose)
        ↓
4. Giai đoạn calibration tự động (~5 giây)
   [CALIBRATING 1/10] drift=13.1% ...
   [CALIBRATING 10/10] drift=13.4% ...
   Calibration done! baseline=13.2% | warn>28.2% | alert>35.2%
        ↓
5. Robot hoạt động bình thường
   [INFO] OK | drift=13.5% baseline=13.2% increase=+0.3% | consecutive=0/3
        ↓
6. Nếu robot bị lệch map
   [WARN]  DRIFT_WARN  | drift=30.1% increase=+16.9% | consecutive=1/3
   [ERROR] DRIFT_ALERT | drift=38.4% increase=+25.2% | consecutive=3/3
   [ERROR] MAP DRIFT DETECTED! — Please reset pose!
        ↓
7. Nhận /map_drift/reset_pose_request=true
   → Operator hoặc node khác publish /initialpose để reset
```

---

## Xử lý reset pose tự động

Để tự động mở rviz panel khi nhận reset request, có thể viết node lắng nghe:

```python
# Ví dụ: tự publish initial pose về vị trí dock
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseWithCovarianceStamped

def reset_cb(msg):
    if msg.data:
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = 'map'
        # ... set x, y, theta của vị trí dock
        initial_pose_pub.publish(pose)

node.create_subscription(Bool, '/map_drift/reset_pose_request', reset_cb, 10)
```

---

## Giới hạn & Lưu ý

- **Calibration phải đúng:** Nếu robot đứng sai vị trí trong 10 giây đầu, baseline sẽ sai và ngưỡng bị lệch theo.
- **Map tĩnh:** Node so sánh với map đã lưu. Vật thể động (người, xe) làm tăng drift_ratio tạm thời — đây là noise bình thường, được lọc bởi `consecutive_alerts_required`.
- **Phụ thuộc TF:** Node cần TF `map → laser_frame` từ SLAM/AMCL. Nếu localization chưa khởi động, node sẽ báo `TF lookup failed`.
- **Không thay thế AMCL:** Đây là monitor cảnh báo, không phải thuật toán định vị. SLAM/AMCL vẫn cần chạy.
