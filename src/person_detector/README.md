# person_detector

ROS 2 package phát hiện người tiếp cận robot lễ tân bằng dữ liệu LiDAR 2D. Khi có người xuất hiện trong phạm vi cấu hình, node sẽ phát tín hiệu để robot thực hiện hành động chào hỏi.

---

## Mục lục

- [Nguyên lý hoạt động](#nguyên-lý-hoạt-động)
- [Cấu trúc package](#cấu-trúc-package)
- [Nodes](#nodes)
- [Topics](#topics)
- [Tham số cấu hình](#tham-số-cấu-hình)
- [Cài đặt và build](#cài-đặt-và-build)
- [Hướng dẫn sử dụng](#hướng-dẫn-sử-dụng)
- [Tích hợp hành động chào](#tích-hợp-hành-động-chào)
- [Debug và tinh chỉnh](#debug-và-tinh-chỉnh)
- [Phân biệt người với vật thể khác](#phân-biệt-người-với-vật-thể-khác)

---

## Nguyên lý hoạt động

### Tại sao dùng LiDAR ở độ cao 25cm?

LiDAR gắn ở độ cao 25cm từ mặt đất sẽ quét ngang qua **vùng ống chân người** (phần dưới bắp chân). Ở độ cao này:

- **Chân người**: tiết diện hình tròn ~8–20cm, xuất hiện thành **2 cluster riêng biệt** cách nhau 10–65cm (khoảng cách 2 chân khi đứng).
- **Tường/vách**: tạo thành arc lớn liên tục, width >> 25cm → bị lọc ra.
- **Chân bàn/ghế**: thường chỉ 2–4cm đường kính → quá hẹp hoặc có 4 điểm ở góc vuông (không phải cặp chân người).

### Thuật toán Leg-Pair Detection

Pipeline xử lý mỗi frame scan:

```
LaserScan (topic)
       │
       ▼  [1] Lọc range
   Chuyển điểm hợp lệ trong phạm vi detection_range sang tọa độ (x, y)
       │
       ▼  [2] Clustering (phân cụm)
   Nhóm các điểm liên tiếp gần nhau (< max_cluster_gap) thành cluster
       │
       ▼  [3] Lọc leg-like clusters
   Giữ lại cluster có:
     • Số điểm: min_leg_points ≤ n ≤ max_leg_points
     • Đường kính: leg_min_width ≤ width ≤ leg_max_width
       │
       ▼  [4] Ghép cặp chân (pair matching)
   Tìm 2 cluster cách nhau min_stance_width ≤ d ≤ max_stance_width
   → Trung điểm của cặp = vị trí người
       │
       ▼  [5] Temporal filter
   Yêu cầu phát hiện liên tiếp consecutive_detections frames
   → Loại bỏ nhiễu thoáng qua (gương phản xạ, vật thể bay ngang)
       │
       ▼  [6] Greeting trigger
   Nếu xác nhận có người: publish /person_detected và /greeting_trigger
   Áp dụng cooldown để không chào liên tục
```

### Ví dụ thực tế (output từ scan_debugger)

```
Frame #161 | Points in 3.0m: 392  |  Clusters: 7

  Cluster  0: n= 43 pts | width=107.2cm | dist=2.84m  ✗ too wide  → tường
  Cluster  1: n=175 pts | width=411.3cm | dist=2.16m  ✗ too many  → tường lớn
  Cluster  2: n=  7 pts | width=  8.5cm | dist=1.54m  ✓ LEG       → chân phải
  Cluster  3: n=  7 pts | width=  8.2cm | dist=1.49m  ✓ LEG       → chân trái
  Cluster  4: n=108 pts | width=239.6cm | dist=2.10m  ✗ too many  → đồ vật lớn

Pair analysis:
  Cluster 2 + 3: stance=20.4cm | dist=1.52m  ✓ PERSON  ← người ở 1.52m
```

---

## Cấu trúc package

```
person_detector/
├── person_detector/
│   ├── __init__.py
│   ├── person_detector_node.py   # Node chính: phát hiện người
│   ├── greeting_node.py          # Node phản hồi: thực hiện chào
│   └── scan_debugger.py          # Công cụ debug: phân tích cluster
├── launch/
│   └── person_detector.launch.py # Launch cả 2 nodes
├── config/
│   └── person_detector.yaml      # Tất cả tham số cấu hình
├── resource/
│   └── person_detector
├── package.xml
├── setup.py
└── setup.cfg
```

---

## Nodes

### `person_detector_node`

Node chính. Subscribe LaserScan, chạy thuật toán leg-pair detection, publish kết quả.

**Đặc điểm:**
- Hỗ trợ thay đổi `scan_topic` lúc runtime qua `ros2 param set` mà không cần restart.
- Log trạng thái mỗi 50 frame để theo dõi mà không làm tràn terminal.

### `greeting_node`

Node phản hồi. Subscribe `/greeting_trigger` và thực hiện hành động chào. Mặc định publish text lên topic TTS. Có thể mở rộng để phát âm thanh, điều khiển LED, điều hướng robot.

### `scan_debugger`

Công cụ chẩn đoán. Chạy độc lập để in ra toàn bộ kết quả clustering và phân tích cặp chân. Dùng khi cần tinh chỉnh tham số.

---

## Topics

| Topic | Type | Direction | Mô tả |
|---|---|---|---|
| `/neo_robotics/K1_demo/V0_0_0/scan_front` | `sensor_msgs/LaserScan` | Subscribe | Input: dữ liệu LiDAR (cấu hình được) |
| `/person_detected` | `std_msgs/Bool` | Publish | `true` khi có người, `false` khi người rời đi |
| `/greeting_trigger` | `std_msgs/String` | Publish | JSON payload khi trigger chào (có cooldown) |
| `/detected_persons_markers` | `visualization_msgs/MarkerArray` | Publish | Markers hình trụ xanh cho RViz |
| `/tts/text` | `std_msgs/String` | Publish | Text gửi sang hệ thống TTS (từ greeting_node) |

### Định dạng `/greeting_trigger`

```json
{
  "event": "person_detected",
  "distance": 1.25,
  "position": {"x": 1.24, "y": -0.08},
  "timestamp": 1718500000.123
}
```

---

## Tham số cấu hình

File: `config/person_detector.yaml`

### person_detector_node

| Tham số | Mặc định | Đơn vị | Mô tả |
|---|---|---|---|
| `scan_topic` | `/neo_robotics/K1_demo/V0_0_0/scan_front` | — | Topic LaserScan đầu vào |
| `detection_range` | `1.0` | m | Bán kính phát hiện người |
| `leg_min_width` | `0.10` | m | Width tối thiểu của cluster chân (10cm) |
| `leg_max_width` | `0.25` | m | Width tối đa của cluster chân (25cm) |
| `min_leg_points` | `10` | rays | Số điểm scan tối thiểu trong cluster chân |
| `max_leg_points` | `40` | rays | Số điểm scan tối đa (lọc tường, bàn) |
| `max_cluster_gap` | `0.12` | m | Khoảng cách tối đa giữa 2 điểm để ghép cluster |
| `min_stance_width` | `0.10` | m | Khoảng cách tối thiểu giữa 2 chân |
| `max_stance_width` | `0.65` | m | Khoảng cách tối đa giữa 2 chân |
| `greeting_cooldown` | `5.0` | s | Thời gian chờ giữa 2 lần chào |
| `consecutive_detections` | `4` | frames | Số frame liên tiếp cần có để xác nhận người |
| `require_movement` | `false` | bool | Bật lọc vật tĩnh (chân bàn/ghế) |
| `movement_threshold` | `0.05` | m | Dịch chuyển tối thiểu để tính là đang di chuyển |

### greeting_node

| Tham số | Mặc định | Mô tả |
|---|---|---|
| `greeting_text` | `"Xin chào! Tôi có thể giúp gì cho bạn?"` | Nội dung lời chào |
| `tts_topic` | `/tts/text` | Topic gửi text sang hệ thống TTS |

---

## Cài đặt và build

**Yêu cầu:** ROS 2 Humble, `sensor_msgs`, `std_msgs`, `visualization_msgs`

```bash
# Từ thư mục gốc workspace
cd ~/Documents/h3_simulation

# Build package
colcon build --packages-select person_detector

# Source environment
source install/setup.bash
```

---

## Hướng dẫn sử dụng

### Chạy bình thường (khuyến nghị)

```bash
source install/setup.bash
ros2 launch person_detector person_detector.launch.py
```

### Chạy với tùy chọn override

```bash
# Đổi topic và phạm vi detect
ros2 launch person_detector person_detector.launch.py \
  scan_topic:=/scan_top \
  detection_range:=2.0

# Chỉ chạy detector, không cần greeting_node
ros2 run person_detector person_detector_node --ros-args \
  -p scan_topic:=/scan_top \
  -p consecutive_detections:=3 \
  -p greeting_cooldown:=8.0
```

### Thay đổi tham số lúc runtime (không cần restart)

```bash
# Đổi scan topic
ros2 param set /person_detector_node scan_topic /scan_filtered

# Giảm phạm vi detect
ros2 param set /person_detector_node detection_range 1.5

# Tắt yêu cầu movement (phát hiện ngay lập tức)
ros2 param set /person_detector_node require_movement false
```

### Theo dõi kết quả

```bash
# Xem trạng thái phát hiện (true/false)
ros2 topic echo /person_detected

# Xem chi tiết khi trigger chào
ros2 topic echo /greeting_trigger

# Tần suất publish
ros2 topic hz /person_detected
```

### Hiển thị trong RViz

Thêm display `MarkerArray` với topic `/detected_persons_markers`. Người được phát hiện sẽ hiển thị dưới dạng **hình trụ màu xanh** kèm label khoảng cách.

---

## Tích hợp hành động chào

Mở file `person_detector/greeting_node.py` và sửa hàm `_do_greeting()`:

```python
def _do_greeting(self, distance):
    # Phát âm thanh
    import subprocess
    subprocess.Popen(['aplay', '/path/to/hello.wav'])

    # Hoặc gọi espeak TTS
    subprocess.Popen(['espeak', '-v', 'vi', self.greeting_text])

    # Hoặc điều khiển LED (publish sang driver topic)
    from std_msgs.msg import Int32
    self.led_pub.publish(Int32(data=1))  # bật LED chào

    # Hoặc điều hướng robot quay về phía người
    # (cần tích hợp với vị trí từ /greeting_trigger)
```

Nếu muốn subscribe từ node khác (không sửa greeting_node):

```python
# Trong node bất kỳ
from std_msgs.msg import Bool, String
import json

def on_greeting(msg):
    payload = json.loads(msg.data)
    distance = payload['distance']
    # ... xử lý

self.create_subscription(String, '/greeting_trigger', on_greeting, 10)
```

---

## Debug và tinh chỉnh

### Chạy scan_debugger

```bash
source install/setup.bash
ros2 run person_detector scan_debugger --ros-args -p scan_topic:=/scan_top
```

Output mẫu mỗi 20 frame:

```
============================================================
Frame #1 | Points in 3.0m: 65
Total clusters: 3

  Cluster  0: n=  7 pts | width=  8.5cm | dist=1.54m | cx= 1.54 cy=-0.07 | ✓ LEG
  Cluster  1: n=  7 pts | width=  8.2cm | dist=1.49m | cx= 1.49 cy= 0.12 | ✓ LEG
  Cluster  2: n= 43 pts | width=320.1cm | dist=2.80m | cx= 2.75 cy= 0.05 | ✗ too wide

Leg candidates: 2

Pair analysis:
  Cluster 0 + 1: stance=20.4cm | center dist=1.52m | ✓ PERSON
```

### Bảng chẩn đoán lỗi thường gặp

| Triệu chứng | Nguyên nhân | Cách fix |
|---|---|---|
| Không detect dù người đứng trước | Node subscribe topic sai | `ros2 param set /person_detector_node scan_topic /scan_top` |
| `too many pts (N > 40)` | Scan resolution cao, LiDAR gần | Tăng `max_leg_points: 60` |
| `too wide (Xcm > 25cm)` | Người mặc quần rộng hoặc đứng nghiêng | Tăng `leg_max_width: 0.35` |
| `too narrow` | Chân nhỏ hoặc LiDAR xa | Giảm `leg_min_width: 0.05` |
| Phát hiện nhầm chân ghế | Kích thước ghế trùng leg | Bật `require_movement: true` |
| Chào liên tục | `greeting_cooldown` quá thấp | Tăng `greeting_cooldown: 10.0` |
| Phát hiện chậm | `consecutive_detections` quá cao | Giảm `consecutive_detections: 2` |
| Nhiễu khi người chưa vào | `consecutive_detections` quá thấp | Tăng `consecutive_detections: 5` |

---

## Phân biệt người với vật thể khác

| Vật thể | Đặc điểm trên LiDAR 25cm | Thuật toán xử lý |
|---|---|---|
| **Người** | 2 cluster ~8–20cm, cách nhau 10–65cm | → Khớp, phát hiện |
| **Tường phẳng** | 1 cluster lớn liên tục, width > 1m | → Lọc bởi `max_leg_points` và `leg_max_width` |
| **Góc tường** | 1–2 cluster nhỏ nhưng sát tường (cluster lớn kề bên) | → Lọc bởi `max_leg_points` của cluster liền kề |
| **Chân bàn/ghế** | 4 điểm gần nhau theo hình vuông, tĩnh | → Lọc bởi `require_movement: true` |
| **Chân ghế đơn** | 1 cluster nhỏ, không có cặp đủ điều kiện | → Không ghép được cặp |
| **Vật thể thoáng qua** | Xuất hiện < `consecutive_detections` frames | → Lọc bởi temporal filter |
