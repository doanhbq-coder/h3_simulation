# Map Drift Raycast — Hướng dẫn chi tiết

> **Dành cho ai chưa biết gì về robotics.** Tài liệu này giải thích từ khái niệm cơ bản nhất đến cách hoạt động nội tại của node.

---

## Mục lục

1. [Bài toán cần giải quyết](#1-bài-toán-cần-giải-quyết)
2. [Các khái niệm cần biết](#2-các-khái-niệm-cần-biết)
3. [Tại sao các cách đơn giản không đủ tốt](#3-tại-sao-các-cách-đơn-giản-không-đủ-tốt)
4. [Nguyên lý Ray Casting Comparison](#4-nguyên-lý-ray-casting-comparison)
5. [Chi tiết thuật toán từng bước](#5-chi-tiết-thuật-toán-từng-bước)
6. [Adaptive Baseline — Tại sao cần calibration](#6-adaptive-baseline--tại-sao-cần-calibration)
7. [Consecutive Filter — Tránh báo giả](#7-consecutive-filter--tránh-báo-giả)
8. [Cài đặt và chạy](#8-cài-đặt-và-chạy)
9. [Topics ROS2](#9-topics-ros2)
10. [Cấu hình tham số](#10-cấu-hình-tham-số)
11. [Đọc hiểu log output](#11-đọc-hiểu-log-output)
12. [Xử lý sự cố](#12-xử-lý-sự-cố)

---

## 1. Bài toán cần giải quyết

Robot tự hành điều hướng bằng cách so sánh bản đồ đã lưu (map) với cảm biến thực tế. Vị trí của robot trên bản đồ gọi là **pose** (tọa độ x, y và góc xoay).

**Vấn đề: Robot có thể bị mất định vị (localization drift).**

Điều này xảy ra khi hệ thống định vị (AMCL, slam_toolbox) ước lượng sai vị trí robot. Robot nghĩ nó đang ở điểm A nhưng thực tế đang ở điểm B.

```
Bản đồ robot nghĩ:          Thực tế:
  ┌──────────┐                ┌──────────┐
  │          │                │          │
  │    R     │                │      R   │
  │          │                │          │
  └──────────┘                └──────────┘
  Robot nghĩ ở trung tâm      Robot thực sự lệch sang phải
```

Khi drift xảy ra, robot sẽ điều hướng sai, có thể đâm vào tường hoặc đi lạc. **Map Drift Raycast Node** phát hiện điều này và yêu cầu người vận hành reset lại vị trí.

---

## 2. Các khái niệm cần biết

### 2.1 LiDAR là gì?

LiDAR (Light Detection And Ranging) là cảm biến laser quét 360° xung quanh robot. Nó phát ra hàng trăm/nghìn tia laser đồng thời, mỗi tia đo khoảng cách tới vật cản gần nhất.

```
         tia laser
    ╲  │  ╱
     ╲ │ ╱
      ╲│╱
  ────[R]────     R = robot với LiDAR ở trên
      ╱│╲
     ╱ │ ╲
    ╱  │  ╲

Mỗi tia trả về: khoảng cách d (mét) tới vật gần nhất
```

Kết quả là một mảng `ranges[]` trong ROS2 (`sensor_msgs/LaserScan`):
- `ranges[0]` = khoảng cách tia 0° 
- `ranges[90]` = khoảng cách tia 90°
- `ranges[i] = inf` hoặc `range_max` nếu tia không chạm gì

### 2.2 OccupancyGrid (Bản đồ lưới ô)

Map trong ROS2 được lưu dưới dạng lưới ô vuông (`nav_msgs/OccupancyGrid`):

```
Mỗi ô = 5cm × 5cm (resolution = 0.05m)

Giá trị của ô:
  100 = occupied  (tường, vật cản) ← màu đen trên Rviz
    0 = free      (không gian trống) ← màu trắng
   -1 = unknown   (chưa quét tới) ← màu xám
```

Ví dụ một căn phòng đơn giản:

```
100 100 100 100 100 100 100
100   0   0   0   0   0 100
100   0   0   0   0   0 100
100   0   0   R   0   0 100   ← R = vị trí robot
100   0   0   0   0   0 100
100   0   0   0   0   0 100
100 100 100 100 100 100 100
```

### 2.3 TF (Transform)

TF là hệ thống quản lý tọa độ trong ROS2. Nó trả lời câu hỏi: *"Laser đang ở đâu trong hệ tọa độ của bản đồ?"*

```
map frame (hệ tọa độ bản đồ)
   └── odom frame (hệ tọa độ odometry)
        └── base_link (thân robot)
             └── laser_frame (vị trí LiDAR)
```

Khi SLAM/AMCL cập nhật, nó điều chỉnh transform `map → odom`. Node này dùng TF để biết vị trí laser trong map frame, từ đó chiếu tia laser lên bản đồ.

---

## 3. Tại sao các cách đơn giản không đủ tốt

### Cách đơn giản nhất: Binary Hit/Miss

Ý tưởng: chiếu endpoint của mỗi tia laser lên map, xem có trúng ô occupied không.

```
Tia laser → endpoint (x, y) → ô map tại (x,y) → occupied? → hit/miss
drift = số miss / tổng số tia
```

**Vấn đề:** Trong môi trường thực, 40-60% tia laser chiếu vào bàn ghế, thiết bị — những thứ KHÔNG có trên map. Endpoint của các tia này rơi vào ô free → tính là miss → drift báo 50-70% dù robot đứng đúng chỗ!

```
Map (chỉ có tường):    Môi trường thực:
┌──────────┐           ┌──────────┐
│          │           │ 🪑  🖥️  │
│    R     │           │    R     │
│          │           │ 🚶 📦   │
└──────────┘           └──────────┘

LiDAR thực chiếu vào ghế, người, hộp → không có trên map
→ Binary hit/miss báo drift cao sai!
```

### Cách cải tiến: Likelihood Field

Thay vì hit/miss, tính khoảng cách endpoint tới tường gần nhất, cho điểm bằng hàm Gaussian.

**Vẫn bị vấn đề tương tự:** ghế, người chặn tia → endpoint xa tường → điểm thấp → drift cao giả.

---

## 4. Nguyên lý Ray Casting Comparison

### Ý tưởng cốt lõi

Thay vì hỏi *"endpoint có gần tường không?"*, ta hỏi *"tia laser đi xa hơn hay gần hơn so với bản đồ kỳ vọng?"*

**Bước 1:** Với mỗi hướng tia laser, ta "mô phỏng" tia đó trên bản đồ → tìm khoảng cách kỳ vọng `d_map`.

**Bước 2:** So sánh với khoảng cách thực tế `d_actual`:

```
d_actual ≈ d_map  →  MATCH   (tia khớp tường trên map)
d_actual < d_map  →  DYNAMIC (có vật chắn trước tường → ghế, người, hộp)
d_actual > d_map  →  DRIFT   (tia đi XA HƠN tường → robot ở sai vị trí!)
```

### Tại sao cách này đúng?

Khi **robot đứng đúng vị trí**, mọi tia hướng vào tường:
- Tia thực tế chạm tường ở khoảng cách đúng → `d_actual ≈ d_map` → **MATCH**
- Nếu có ghế chắn giữa robot và tường → `d_actual < d_map` → **DYNAMIC, bỏ qua!**

Khi **robot bị lệch** (drift), hướng nhìn sai:
- Tia thực tế chiếu qua "khoảng trống" nơi robot nghĩ là tường → `d_actual >> d_map` → **DRIFT!**

```
Pose đúng:                         Pose lệch (robot lệch trái):
  map:   ████  tường tại d=5m        map:   ████  tường tại d=5m
  scan:    →●  d_actual=5.0m         scan:     →●  d_actual=7.2m (qua chỗ tường)
  → MATCH ✓                          → DRIFT! ✗

Có ghế chắn:
  map:   ████  tường tại d=5m
  scan:  →🪑   d_actual=2.0m (chạm ghế trước tường)
  → DYNAMIC, bỏ qua ✓
```

**Kết quả:** Vật thể động (ghế, người) **không bao giờ gây false alarm** vì chúng luôn làm tia ngắn hơn, không bao giờ dài hơn.

---

## 5. Chi tiết thuật toán từng bước

### Bước 1: Nhận dữ liệu

Node đăng ký lắng nghe 3 nguồn dữ liệu:

```
/map           → OccupancyGrid (bản đồ tĩnh, nhận 1 lần)
/scan_top      → LaserScan (dữ liệu LiDAR, ~10-20Hz)
TF tree        → vị trí laser trong map frame (liên tục)
```

### Bước 2: Subsampling tia laser

LiDAR thường có 720-1440 tia. Ta không cần tính tất cả — cứ lấy 1 tia bỏ qua 2 (`ray_skip=3`):

```
Tia gốc:  0° 0.25° 0.5° 0.75° 1.0° 1.25° 1.5° ...
Sau skip: 0°       0.5°       1.0°        1.5° ...

480 tia thay vì 1440 → tốc độ tính nhanh hơn 3×
```

### Bước 3: Lấy pose từ TF

```python
tf = lookup_transform("map", "laser_frame")
laser_x, laser_y = tf.translation.x, tf.translation.y
yaw = euler_from_quaternion(tf.rotation)  # góc xoay robot
```

### Bước 4: Vectorized Ray Casting

Đây là phần tính toán chính. Thay vì vòng lặp Python chậm, ta dùng numpy để tính tất cả tia cùng lúc.

**Idea:** Với mỗi tia, di chuyển từng bước 5cm theo hướng tia, tra cứu ô map tại mỗi bước, tìm bước đầu tiên chạm ô occupied.

```
Tia số 0 (hướng 0°, robot nhìn thẳng):
  bước 1: x=0.05m → ô map (1,0) → free (0)
  bước 2: x=0.10m → ô map (2,0) → free (0)
  ...
  bước 98: x=4.90m → ô map (98,0) → free (0)
  bước 99: x=4.95m → ô map (99,0) → free (0)
  bước 100: x=5.00m → ô map (100,0) → OCCUPIED (100) ← d_map = 5.0m
```

**Vectorized (tất cả N tia, M bước cùng lúc):**

```
Ma trận vị trí tia: N × M (ví dụ 480 tia × 200 bước)

t_arr = [0.05, 0.10, 0.15, ..., 10.0]   (M giá trị khoảng cách)

x_all[i,j] = laser_x + t_arr[j] × cos(angle[i])
y_all[i,j] = laser_y + t_arr[j] × sin(angle[i])

mx_all[i,j] = floor((x_all[i,j] - map_origin_x) / resolution)
my_all[i,j] = floor((y_all[i,j] - map_origin_y) / resolution)

map_vals[i,j] = map_data[my_all[i,j], mx_all[i,j]]

occ[i,j] = map_vals[i,j] >= 50   → ma trận True/False

first_hit[i] = argmax(occ[i,:])  → bước đầu tiên tia i chạm tường
d_map[i] = first_hit[i] × 0.05  → khoảng cách kỳ vọng (m)
```

### Bước 5: Phân loại từng tia

```python
tol = 0.20  # sai số cho phép 20cm

MATCH   : |d_actual - d_map| ≤ tol   # tia khớp tường
DRIFT   : d_actual > d_map + tol      # tia đi xa hơn tường → DRIFT!
DYNAMIC : d_actual < d_map - tol      # vật chắn trước tường → bỏ qua
```

### Bước 6: Tính chỉ số

```
n_eval    = số tia evaluable (valid hit + map có tường ở hướng đó)
n_match   = số tia MATCH
n_drift   = số tia DRIFT

drift_ratio = n_drift / n_eval      (0.0 = tốt, 1.0 = lệch hoàn toàn)
match_ratio = n_match / n_eval
dynamic_ratio = n_dynamic / n_valid  (thông tin phụ: % môi trường thay đổi)
```

---

## 6. Adaptive Baseline — Tại sao cần calibration

### Vấn đề với ngưỡng cố định

Ngay cả khi pose đúng 100%, `drift_ratio` không bao giờ = 0% vì:
- Noise nhỏ của LiDAR (±1-3cm)
- Sai số discretization của map (ô 5cm)
- Map build lúc trước, hiện tại có thêm một số thứ nhỏ

Mỗi môi trường có "nền nhiễu" khác nhau:
- Hành lang hẹp: nền ~2-5% (nhiều tường xung quanh, ít vật thể)
- Phòng rộng có nhiều đồ đạc: nền ~8-15%

Nếu dùng ngưỡng cố định `alert = 30%`, môi trường nhiễu cao có thể báo liên tục.

### Giải pháp: Đo baseline khi khởi động

```
Giai đoạn calibration (10 samples × 0.5s = 5 giây đầu):
  sample 1: drift=3.2%
  sample 2: drift=2.8%
  ...
  sample 10: drift=3.5%
  baseline = mean = 3.1%

Sau calibration:
  warn_threshold  = baseline + 15% = 18.1%
  alert_threshold = baseline + 22% = 25.1%
```

**Yêu cầu quan trọng:** Trong 5 giây đầu khi calibration, robot PHẢI đứng yên tại vị trí đã set initial pose đúng. Nếu robot di chuyển trong lúc calibration → baseline bị sai → ngưỡng bị lệch.

---

## 7. Consecutive Filter — Tránh báo giả

### Vấn đề

Drift ratio có thể dao động tạm thời do:
- Người đi qua trong 1-2 giây
- LiDAR bị nhiễu nhất thời
- Robot vừa vào góc tường, scan bị che khuất tạm thời

Nếu 1 lần vượt ngưỡng là báo alarm ngay → quá nhiều false alarm.

### Giải pháp: Đếm liên tiếp

```
consecutive_alerts_required = 3  (cần 3 lần liên tiếp)

check 1 (t=0.0s): drift=26% ≥ 25.1% → consecutive=1
check 2 (t=0.5s): drift=27% ≥ 25.1% → consecutive=2
check 3 (t=1.0s): drift=28% ≥ 25.1% → consecutive=3 → ALERT!

Nếu giữa chừng có 1 check bình thường:
check 1 (t=0.0s): drift=26% → consecutive=1
check 2 (t=0.5s): drift=20% (người đi qua xong) → consecutive=0 (reset)
→ KHÔNG báo alert
```

Counter KHÔNG reset về 0 ngay khi xuống ngưỡng mà giảm từng 1, tránh dao động nhanh.

---

## 8. Cài đặt và chạy

### Yêu cầu

- ROS2 Humble
- Navigation stack (nav2) đang chạy với `/map` và localization (AMCL hoặc slam_toolbox)
- Python packages: `numpy` (có sẵn), `scipy` (optional, dùng cho likelihood node)

### Build

```bash
cd ~/Documents/h3_simulation
colcon build --packages-select map_drift_monitor
source install/setup.bash
```

### Chạy

```bash
# Cách 1: Launch file (khuyến nghị)
ros2 launch map_drift_monitor map_drift_raycast.launch.py

# Cách 2: Với sim time (khi dùng Gazebo)
ros2 launch map_drift_monitor map_drift_raycast.launch.py use_sim_time:=true

# Cách 3: Chạy trực tiếp với config tùy chỉnh
ros2 launch map_drift_monitor map_drift_raycast.launch.py \
  config:=/path/to/my_config.yaml
```

### Thứ tự khởi động đúng

```
1. Khởi động navigation stack (map_server + localization)
        ↓
2. Set initial pose đúng cho robot (publish /initialpose)
        ↓
3. Khởi động map_drift_raycast
        ↓
4. ĐỂ ROBOT ĐỨNG YÊN trong 5 giây (calibration)
        ↓
5. Robot có thể bắt đầu di chuyển
```

---

## 9. Topics ROS2

### Subscribe (nhận dữ liệu)

| Topic | Message Type | Mô tả |
|-------|-------------|-------|
| `/map` | `nav_msgs/OccupancyGrid` | Bản đồ tĩnh (QoS: TRANSIENT_LOCAL) |
| `/scan_top` | `sensor_msgs/LaserScan` | Dữ liệu LiDAR |
| TF: `map → laser_frame` | — | Vị trí laser trong map |

### Publish (gửi ra)

| Topic | Message Type | Giá trị | Mô tả |
|-------|-------------|---------|-------|
| `/map_drift/drift_ratio` | `Float32` | 0.0–1.0 | Tỉ lệ drift hiện tại |
| `/map_drift/baseline` | `Float32` | 0.0–1.0 | Baseline đã calibrate |
| `/map_drift/drift_increase` | `Float32` | -1.0–1.0 | Mức tăng so với baseline |
| `/map_drift/dynamic_ratio` | `Float32` | 0.0–1.0 | Tỉ lệ tia bị vật động chặn |
| `/map_drift/alert` | `Bool` | true/false | Cảnh báo drift đã xác nhận |
| `/map_drift/status` | `String` | text | Trạng thái chi tiết |
| `/map_drift/reset_pose_request` | `Bool` | true | Yêu cầu reset pose |

### Theo dõi realtime

```bash
# Xem tất cả thông tin
ros2 topic echo /map_drift/status

# Chỉ xem drift ratio (số)
ros2 topic echo /map_drift/drift_ratio

# Xem tỉ lệ vật động (môi trường thay đổi bao nhiêu)
ros2 topic echo /map_drift/dynamic_ratio

# Plot đồ thị trong rqt
rqt &
# Plugins → Visualization → Plot
# Thêm: /map_drift/drift_ratio/data
#        /map_drift/baseline/data
#        /map_drift/dynamic_ratio/data
```

---

## 10. Cấu hình tham số

File: `config/map_drift_raycast.yaml`

```yaml
map_drift_raycast_node:
  ros__parameters:
    scan_topic: "/scan_top"      # Tên topic LiDAR
    map_topic: "/map"            # Tên topic map
    map_frame: "map"             # Tên frame bản đồ
    robot_frame: "base_link"     # Tên frame thân robot

    range_tolerance: 0.20        # Sai số cho phép (m)
    max_cast_range: 10.0         # Khoảng cast tối đa (m)
    ray_skip: 3                  # Lấy 1/ray_skip số tia
    min_evaluable_rays: 30       # Tối thiểu tia đánh giá được

    calibration_samples: 10      # Số mẫu để đo baseline
    drift_increase_warn: 0.15    # Tăng 15% so với baseline → WARN
    drift_increase_alert: 0.22   # Tăng 22% so với baseline → ALERT
    drift_warn_threshold: 0.35   # Ngưỡng tuyệt đối fallback WARN
    drift_alert_threshold: 0.50  # Ngưỡng tuyệt đối fallback ALERT

    consecutive_alerts_required: 3  # Lần liên tiếp vượt ngưỡng
    check_rate: 2.0                 # Tần suất kiểm tra (Hz)
    occupancy_threshold: 50         # Ngưỡng ô occupied (0-100)
```

### Hướng dẫn điều chỉnh

**`range_tolerance`** — quan trọng nhất

| Giá trị | Tác dụng | Khi nào dùng |
|---------|---------|-------------|
| 0.10m | Rất chặt | LiDAR chính xác cao, map chi tiết |
| 0.20m | Mặc định | Hầu hết robot indoor |
| 0.30m | Khoan dung | LiDAR rẻ tiền, noise nhiều |

**`ray_skip`** — đánh đổi tốc độ vs độ chính xác

| Giá trị | Số tia dùng (1440-ray lidar) | Khi nào dùng |
|---------|------------------------------|-------------|
| 1 | 1440 | Máy tính mạnh, độ chính xác tối đa |
| 3 | 480 | Khuyến nghị (cân bằng) |
| 6 | 240 | CPU yếu |

**`drift_increase_alert`** — điều chỉnh theo môi trường

| Môi trường | Giá trị khuyến nghị |
|-----------|-------------------|
| Hành lang ít người | 0.18 (nhạy hơn) |
| Văn phòng bình thường | 0.22 (mặc định) |
| Khu vực đông người | 0.30 (ít nhạy hơn) |

---

## 11. Đọc hiểu log output

### Giai đoạn calibration

```
[INFO] Map: 200x200 res=0.05m occupied=823 origin=(-5.00,-5.00)
[INFO] Scan cached: 480 rays (skip=3, total=1440) frame=laser_top
[INFO] [CALIBRATING 1/10] drift=3.2% dynamic=12.0% eval=156 — Giữ robot tại vị trí khởi động...
[INFO] [CALIBRATING 5/10] drift=2.8% dynamic=11.5% eval=154 — ...
[INFO] Calibration done! baseline=3.1% | warn>18.1% | alert>25.1%
```

Ý nghĩa từng trường:
- `drift=3.2%` — tỉ lệ tia DRIFT tại thời điểm này
- `dynamic=12.0%` — 12% tia bị vật thể chặn trước tường (bình thường)
- `eval=156` — 156 tia được đánh giá (có hit và map có tường ở hướng đó)

### Hoạt động bình thường

```
[INFO] OK | drift=3.5% base=3.1% +0.4% | dynamic=14.2% eval=148 | con=0/3
```

- `drift=3.5%` — tỉ lệ drift hiện tại
- `base=3.1%` — baseline đã calibrate
- `+0.4%` — tăng 0.4% so với baseline (rất thấp → bình thường)
- `dynamic=14.2%` — 14.2% tia gặp vật thể không trên map
- `eval=148` — số tia được đánh giá
- `con=0/3` — consecutive counter: 0 lần liên tiếp vượt ngưỡng

### Khi bắt đầu drift

```
[WARN] DRIFT_WARN  | drift=20.3% base=3.1% +17.2% | dynamic=13.1% eval=152 | con=1/3
[WARN] DRIFT_WARN  | drift=21.8% base=3.1% +18.7% | dynamic=12.9% eval=150 | con=2/3
[ERROR] DRIFT_ALERT | drift=22.5% base=3.1% +19.4% | dynamic=13.0% eval=151 | con=3/3
[ERROR] MAP DRIFT DETECTED! drift=22.5% base=3.1% +19.4% dynamic=13.0% — Please reset pose!
```

### Phân tích log khi có vấn đề

| Triệu chứng trong log | Ý nghĩa | Xử lý |
|----------------------|---------|-------|
| `drift` cao nhưng `dynamic` thấp | Robot thật sự lệch map | Reset pose |
| `drift` cao VÀ `dynamic` cao (>40%) | Nhiều vật thể chắn tầm nhìn | Tăng `range_tolerance`, kiểm tra môi trường |
| `eval` rất thấp (<30) | Ít tia đánh giá được | Giảm `min_evaluable_rays` hoặc tăng `max_cast_range` |
| `Waiting for /map...` | Chưa nhận được map | Kiểm tra nav2 map_server có chạy không |
| `TF lookup failed` | Không có transform map→laser | Kiểm tra localization node |

---

## 12. Xử lý sự cố

### Node không log gì sau khi khởi động

1. Kiểm tra các dependencies:
```bash
ros2 topic list | grep -E "map$|scan_top|clock"
```
2. Xem `/map` có publish không (lỗi QoS thường gặp):
```bash
ros2 topic info /map --verbose
```
Phải có `TRANSIENT_LOCAL` ở publisher.

3. Kiểm tra TF:
```bash
ros2 run tf2_tools view_frames  # tạo file PDF xem TF tree
```

### drift_ratio luôn = 0% sau calibration

Không có tia DRIFT nào → map không có occupied cells, hoặc `max_cast_range` quá ngắn.

```bash
# Kiểm tra số ô occupied trong map
ros2 topic echo /map_drift/status  # xem log "Map received: occupied=N"
# Nếu occupied=0 → map bị đọc sai
```

### Calibration xong nhưng drift luôn cao khi di chuyển

`dynamic_ratio` có cao không? Nếu `dynamic > 30%` → môi trường có nhiều thứ không trên map. Tăng `range_tolerance` và `drift_increase_alert`.

### Alert liên tục trong khi robot đứng yên đúng vị trí

Có thể calibration bị sai vì robot đang di chuyển lúc calibrate. Restart node khi robot đứng yên ở đúng vị trí.

---

## Tóm tắt hoạt động

```
Map (bản đồ tĩnh)
    │
    ▼
[map_drift_raycast_node]
    │
    ├── Nhận LaserScan từ LiDAR
    ├── Nhận pose từ TF (SLAM/AMCL cập nhật)
    │
    ├── Cast mỗi tia qua map → d_map (kỳ vọng)
    ├── So sánh với d_actual (thực tế):
    │     MATCH   → tia khớp tường ✓
    │     DYNAMIC → vật chắn trước tường → bỏ qua
    │     DRIFT   → tia qua tường → đáng ngờ!
    │
    ├── drift_ratio = n_drift / n_eval
    ├── So với baseline (đo khi khởi động)
    ├── Lọc consecutive (3 lần liên tiếp)
    │
    └── Publish:
          /map_drift/alert = true  ──►  Node khác nhận → reset pose
          /map_drift/status        ──►  Log / monitoring
          /map_drift/drift_ratio   ──►  Plot / dashboard
```
