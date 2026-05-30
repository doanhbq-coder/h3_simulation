# H3 Robot API Server — Tài liệu API

**Base URL:** `http://<ROBOT_IP>:8090`  
**Content-Type:** `application/json`  
**Docs (Swagger UI):** `http://<ROBOT_IP>:8090/docs`

---

## 1. Bảng thống kê các API

### REST Endpoints

| # | Nhóm | Method | Endpoint | Mô tả | Cần Nav2 |
|---|------|--------|----------|-------|:--------:|
| 1 | System | `GET` | `/robot/status` | Trạng thái tổng quan robot | ✗ |
| 2 | Chassis | `POST` | `/chassis/moves` | Di chuyển đến điểm / sạc / xoay | ✓ |
| 3 | Chassis | `GET` | `/chassis/moves/current` | Trạng thái di chuyển hiện tại | ✗ |
| 4 | Chassis | `PATCH` | `/chassis/moves/current` | Huỷ di chuyển | ✓ |
| 5 | Chassis | `GET` | `/chassis/pose` | Đọc vị trí hiện tại | ✗ |
| 6 | Chassis | `POST` | `/chassis/pose` | Đặt lại vị trí robot (AMCL) | ✗ |
| 7 | Chassis | `POST` | `/chassis/twist` | Điều khiển vận tốc tức thời | ✗ |
| 8 | Chassis | `DELETE` | `/chassis/twist` | Dừng robot ngay lập tức | ✗ |
| 9 | Chassis | `POST` | `/chassis/rotate` | Xoay đến góc chỉ định | ✗ |
| 10 | Maps | `POST` | `/chassis/current-map` | Đổi bản đồ đang dùng | ✗ |
| 11 | Maps | `GET` | `/maps/` | Danh sách bản đồ | ✗ |
| 12 | Tray | `POST` | `/tray/open` | Mở khay đựng hàng | ✗ |
| 13 | Tray | `POST` | `/tray/close` | Đóng khay đựng hàng | ✗ |
| 14 | Tray | `GET` | `/tray/status` | Trạng thái khay | ✗ |
| 15 | Safety | `POST` | `/emergency-stop` | Kích hoạt dừng khẩn cấp | ✗ |
| 16 | Safety | `POST` | `/emergency-stop/release` | Giải phóng dừng khẩn cấp | ✗ |
| 17 | Safety | `GET` | `/emergency-stop/status` | Trạng thái e-stop | ✗ |
| 18 | System | `POST` | `/system/restart` | Khởi động lại ROS node | ✗ |

### WebSocket Endpoints

| # | Endpoint | Tần suất | Dữ liệu |
|---|----------|----------|---------|
| 1 | `ws://.../ws/pose` | 200ms | Toạ độ x, y, yaw_deg |
| 2 | `ws://.../ws/battery` | 2s | Pin %, voltage, current, status |
| 3 | `ws://.../ws/speed` | 200ms | linear_x, linear_y, angular_z |
| 4 | `ws://.../ws/status` | 500ms | Tổng hợp tất cả trạng thái |

---

## 2. Chi tiết các API

---

### 2.1 `GET /robot/status` — Trạng thái tổng quan

**Mô tả:** Trả về toàn bộ trạng thái hiện tại của robot trong một lần gọi.

**Request:** Không có body.

**Response `200 OK`:**
```json
{
  "pose": { "x": 1.23, "y": -0.45, "yaw_deg": 90.0 },
  "speed": { "linear_x": 0.0, "linear_y": 0.0, "angular_z": 0.0 },
  "battery": { "percentage": 85.0, "voltage": 24.1, "current": -1.2, "status": "discharging" },
  "move_state": "idle",
  "emergency_stop": false,
  "tray_open": false,
  "current_map": "lobby_v5"
}
```

| Trường | Kiểu | Ý nghĩa |
|--------|------|---------|
| `move_state` | string | `idle` / `moving` / `completed` / `cancelled` / `failed` |
| `battery.status` | string | `charging` / `discharging` / `full` / `not_charging` / `unknown` |

```mermaid
sequenceDiagram
    autonumber
    actor User
    participant Client as Client App
    participant API as API Server
    participant ROS as ROS2 Topics

    User->>Client: Mở màn hình dashboard
    Client->>API: GET /robot/status
    activate API
    Note over API: Đọc dữ liệu từ RobotState<br/>(cập nhật liên tục từ /odom,<br/>/cmd_vel, /battery_state)
    API-->>Client: 200 OK — JSON trạng thái đầy đủ
    deactivate API
    Client->>User: Hiển thị dashboard robot
```

---

### 2.2 `POST /chassis/moves` — Di chuyển robot

**Mô tả:** Gửi lệnh di chuyển cho robot. Hỗ trợ 3 loại (`type`):
- `standard` — điều hướng đến toạ độ chỉ định
- `charge` — quay về trạm sạc
- `rotate` — xoay tại chỗ đến góc chỉ định

**Request Body:**
```json
{
  "type": "standard",
  "target_x": 2.5,
  "target_y": 1.0,
  "target_z": 0.0,
  "target_ori": 90.0,
  "target_accuracy": 0.1,
  "use_target_zone": false,
  "approach_speed_limit": null,
  "creator": "android_app"
}
```

| Trường | Bắt buộc | Mặc định | Mô tả |
|--------|:--------:|---------|-------|
| `type` | ✓ | `"standard"` | Loại di chuyển |
| `target_x` | Nếu type=standard | — | Toạ độ X đích (mét) |
| `target_y` | Nếu type=standard | — | Toạ độ Y đích (mét) |
| `target_ori` | ✗ | `0.0` | Góc quay đích (độ) |
| `target_accuracy` | ✗ | `0.1` | Độ chính xác dừng (mét) |
| `approach_speed_limit` | ✗ | `null` | Tốc độ tiếp cận tối đa (m/s) |
| `creator` | ✗ | `"api"` | Tên client gọi API |

**Response `200 OK`:**
```json
{
  "status": "accepted",
  "type": "standard",
  "goal": { "x": 2.5, "y": 1.0, "yaw_deg": 90.0 },
  "detail": "completed"
}
```

**Lỗi:**
- `409` — Robot đang di chuyển hoặc e-stop đang bật
- `422` — Thiếu `target_x` / `target_y` khi `type=standard`

#### Luồng: `type = "standard"`

```mermaid
sequenceDiagram
    autonumber
    actor User
    participant Client as Client App
    participant API as API Server (FastAPI)
    participant Nav2 as ROS2 Nav2
    participant Gazebo as Gazebo / Robot

    User->>Client: Chọn điểm đích trên bản đồ
    Note over Client: Tính toán target_x, target_y, target_ori<br/>từ toạ độ pixel → toạ độ thực

    Client->>API: POST /chassis/moves<br/>{"type":"standard","target_x":2.5,"target_y":1.0,"target_ori":90}
    activate API
    Note over API: Kiểm tra e-stop, move_state<br/>Chuyển target_ori (độ) → Quaternion (z,w)
    API->>Nav2: Action Goal: NavigateToPose<br/>geometry_msgs/PoseStamped {x,y,z,w}
    activate Nav2
    Nav2-->>API: Goal Accepted
    Note over API: move_state = "moving"
    API-->>Client: 200 OK {"status":"accepted"}
    deactivate API

    rect rgb(235, 245, 255)
        Note over Nav2,Gazebo: VÒNG LẶP ĐIỀU HƯỚNG
        loop Đến khi đến đích (~200ms/lần)
            Nav2->>Gazebo: Publish /cmd_vel<br/>(linear_x, angular_z)
            Gazebo-->>Nav2: Feedback /odom (vị trí hiện tại)
            Nav2-->>API: Action Feedback<br/>(distance_remaining)
            API-->>Client: WS /ws/status<br/>{"move_state":"moving","pose":{...}}
            Client->>User: Cập nhật vị trí robot trên bản đồ
        end
    end

    Nav2-->>API: Action Result: SUCCEEDED
    deactivate Nav2
    Note over API: move_state = "completed"
    API-->>Client: WS /ws/status<br/>{"move_state":"completed"}
    Client->>User: Thông báo "Robot đã đến đích"
```

#### Luồng: `type = "charge"`

```mermaid
sequenceDiagram
    autonumber
    actor User
    participant Client as Client App
    participant API as API Server
    participant Nav2 as ROS2 Nav2
    participant Dock as Trạm sạc

    User->>Client: Nhấn "Về trạm sạc"
    Client->>API: POST /chassis/moves<br/>{"type":"charge","creator":"app"}
    activate API
    Note over API: Lấy toạ độ trạm sạc từ params<br/>charge_x, charge_y, charge_yaw_deg
    API->>Nav2: NavigateToPose → toạ độ trạm sạc
    activate Nav2
    Nav2-->>API: Goal Accepted
    API-->>Client: 200 OK {"status":"accepted","type":"charge"}
    deactivate API

    Nav2->>Dock: Robot di chuyển về trạm sạc
    Nav2-->>API: Action Result: SUCCEEDED
    deactivate Nav2
    Note over API: move_state = "completed"
    API-->>Client: WS /ws/status {"move_state":"completed"}
    Client->>User: "Robot đã về trạm sạc"
```

---

### 2.3 `GET /chassis/moves/current` — Trạng thái di chuyển

**Mô tả:** Lấy trạng thái di chuyển hiện tại và vị trí robot.

**Response `200 OK`:**
```json
{
  "move_state": "moving",
  "pose": { "x": 1.1, "y": 0.5, "yaw_deg": 45.0 }
}
```

---

### 2.4 `PATCH /chassis/moves/current` — Huỷ di chuyển

**Mô tả:** Huỷ lệnh điều hướng đang thực hiện, robot dừng lại.

**Request Body:**
```json
{ "state": "cancelled" }
```

**Response `200 OK`:**
```json
{ "status": "cancelled" }
```

```mermaid
sequenceDiagram
    autonumber
    actor User
    participant Client as Client App
    participant API as API Server
    participant Nav2 as ROS2 Nav2

    User->>Client: Nhấn nút "Dừng / Huỷ"
    Client->>API: PATCH /chassis/moves/current<br/>{"state":"cancelled"}
    activate API
    API->>Nav2: cancel_goal_async()
    activate Nav2
    Nav2-->>API: Goal Cancelled
    deactivate Nav2
    Note over API: move_state = "cancelled"
    API-->>Client: 200 OK {"status":"cancelled"}
    deactivate API
    Client->>User: Robot dừng di chuyển
```

---

### 2.5 `GET /chassis/pose` — Đọc vị trí hiện tại

**Mô tả:** Trả về toạ độ và hướng hiện tại của robot (lấy từ topic `/odom`).

**Response `200 OK`:**
```json
{ "x": 1.23, "y": -0.45, "yaw_deg": 90.0 }
```

| Trường | Đơn vị | Mô tả |
|--------|--------|-------|
| `x` | mét | Vị trí trục X trong frame `map` |
| `y` | mét | Vị trí trục Y trong frame `map` |
| `yaw_deg` | độ | Góc quay quanh trục Z (0° = hướng Đông) |

---

### 2.6 `POST /chassis/pose` — Đặt lại vị trí robot

**Mô tả:** Gửi `initialpose` để AMCL/SLAM tái định vị robot tại vị trí chỉ định. Dùng khi robot bị lạc hoặc mới bật lên.

**Request Body:**
```json
{ "x": 0.0, "y": 0.0, "z": 0.0, "orientation": 0.0 }
```

| Trường | Mô tả |
|--------|-------|
| `x`, `y` | Toạ độ thực trên bản đồ (mét) |
| `orientation` | Góc yaw ban đầu (độ) |

**Response `200 OK`:**
```json
{
  "status": "ok",
  "pose": { "x": 0.0, "y": 0.0, "yaw_deg": 0.0 }
}
```

```mermaid
sequenceDiagram
    autonumber
    actor User
    participant Client as Client App
    participant API as API Server
    participant AMCL as AMCL / SLAM Toolbox

    User->>Client: Chỉ định vị trí ban đầu trên bản đồ
    Client->>API: POST /chassis/pose<br/>{"x":0.0,"y":0.0,"orientation":0.0}
    activate API
    Note over API: Chuyển orientation (độ) → Quaternion<br/>Tạo PoseWithCovarianceStamped
    API->>AMCL: Publish /initialpose
    AMCL-->>API: (AMCL tự cập nhật particle filter)
    Note over API: Cập nhật state.pose nội bộ
    API-->>Client: 200 OK {"status":"ok","pose":{...}}
    deactivate API
    Client->>User: Robot đã được định vị lại trên bản đồ
```

---

### 2.7 `POST /chassis/twist` — Điều khiển vận tốc

**Mô tả:** Gửi lệnh vận tốc trực tiếp đến robot (publish `/cmd_vel`). Thích hợp cho điều khiển thủ công.

> ⚠️ **Lưu ý:** Lệnh này ghi đè Nav2. Không dùng đồng thời với navigation.

**Request Body:**
```json
{ "linear_x": 0.3, "linear_y": 0.0, "angular_z": 0.5 }
```

| Trường | Đơn vị | Phạm vi | Mô tả |
|--------|--------|---------|-------|
| `linear_x` | m/s | `[-1.0, 1.0]` | Tốc độ tiến/lùi |
| `linear_y` | m/s | `[-1.0, 1.0]` | Tốc độ trượt ngang (robot holonomic) |
| `angular_z` | rad/s | `[-2.0, 2.0]` | Tốc độ xoay (+ = trái) |

**Response `200 OK`:**
```json
{
  "status": "ok",
  "twist": { "linear_x": 0.3, "linear_y": 0.0, "angular_z": 0.5 }
}
```

**Lỗi:** `409` — E-stop đang bật.

```mermaid
sequenceDiagram
    autonumber
    actor User
    participant Client as Client App
    participant API as API Server
    participant CmdVel as /cmd_vel (ROS2 Topic)
    participant Gazebo as Gazebo / Robot

    User->>Client: Nhấn giữ nút điều hướng (joystick)
    Client->>API: POST /chassis/twist<br/>{"linear_x":0.3,"angular_z":0.0}
    activate API
    Note over API: Kiểm tra emergency_stop<br/>Tạo geometry_msgs/Twist
    API->>CmdVel: Publish Twist message
    CmdVel->>Gazebo: diff_drive plugin nhận lệnh
    Gazebo-->>Client: (odom cập nhật → WS /ws/pose)
    API-->>Client: 200 OK {"status":"ok"}
    deactivate API

    User->>Client: Nhả nút (thả tay)
    Client->>API: DELETE /chassis/twist
    API->>CmdVel: Publish Twist {0,0,0}
    Note over Gazebo: Robot dừng lại
```

---

### 2.8 `DELETE /chassis/twist` — Dừng robot ngay

**Mô tả:** Gửi lệnh vận tốc zero, robot dừng ngay lập tức.

**Response `200 OK`:**
```json
{ "status": "stopped" }
```

---

### 2.9 `POST /chassis/rotate` — Xoay đến góc chỉ định

**Mô tả:** Xoay robot tại chỗ đến góc yaw mong muốn (không cần Nav2).

**Request Body:**
```json
{ "angle": 90.0, "angular_speed": 0.4 }
```

| Trường | Mặc định | Mô tả |
|--------|---------|-------|
| `angle` | — | Góc đích (độ, tuyệt đối trong frame map) |
| `angular_speed` | `0.5` | Tốc độ xoay (rad/s) |

**Response `200 OK`:**
```json
{ "status": "accepted", "target_yaw_deg": 90.0, "detail": "completed" }
```

```mermaid
sequenceDiagram
    autonumber
    actor User
    participant Client as Client App
    participant API as API Server
    participant CmdVel as /cmd_vel
    participant Odom as /odom

    User->>Client: Nhập góc xoay 90°
    Client->>API: POST /chassis/rotate<br/>{"angle":90.0,"angular_speed":0.4}
    activate API
    Note over API: Tính góc lệch = target - current_yaw<br/>Xác định chiều xoay

    loop Mỗi 50ms cho đến khi đến góc (sai số < 2°)
        API->>CmdVel: Publish Twist {angular_z: ±0.4}
        Odom-->>API: Cập nhật yaw hiện tại
        Note over API: Kiểm tra |remaining| < 2°
    end

    API->>CmdVel: Publish Twist {0,0,0} — Dừng
    Note over API: move_state = "completed"
    API-->>Client: 200 OK {"status":"accepted","detail":"completed"}
    deactivate API
    Client->>User: Robot đã xoay đến góc 90°
```

---

### 2.10 `POST /chassis/current-map` — Đổi bản đồ

**Mô tả:** Cập nhật bản đồ đang sử dụng (thay đổi metadata nội bộ; để tải bản đồ thực vào Nav2 cần restart map_server).

**Request Body:**
```json
{ "map_id": "elevator" }
```

**Response `200 OK`:**
```json
{ "status": "ok", "current_map": "elevator" }
```

---

### 2.11 `GET /maps/` — Danh sách bản đồ

**Mô tả:** Liệt kê tất cả file bản đồ `.yaml` trong thư mục `maps_dir` (cấu hình qua launch param).

**Response `200 OK`:**
```json
{
  "maps": [
    { "map_id": "elevator", "path": "/share/h3_slam/maps/elevator.yaml" },
    { "map_id": "lobby_v5", "path": "/share/h3_slam/maps/lobby_v5.yaml" },
    { "map_id": "warehouse", "path": "/share/h3_slam/maps/warehouse.yaml" }
  ],
  "count": 3
}
```

---

### 2.12 `POST /tray/open` & `POST /tray/close` — Điều khiển khay

**Mô tả:** Điều khiển khay đựng hàng của robot. Publish lên topic `/tray/command` (std_msgs/Bool).

**Response `200 OK`:**
```json
{ "status": "ok", "tray": "open" }
{ "status": "ok", "tray": "closed" }
```

**`GET /tray/status`:**
```json
{ "tray_open": true }
```

```mermaid
sequenceDiagram
    autonumber
    actor User
    participant Client as Client App
    participant API as API Server
    participant Tray as /tray/command (ROS2 Topic)
    participant Robot as Cơ cấu khay vật lý

    User->>Client: Nhấn "Mở khay"
    Client->>API: POST /tray/open
    activate API
    Note over API: tray_open = true
    API->>Tray: Publish Bool(data=true)
    Tray->>Robot: Kích hoạt servo / actuator mở khay
    API-->>Client: 200 OK {"status":"ok","tray":"open"}
    deactivate API
    Client->>User: Icon khay chuyển sang "Mở"

    User->>Client: Nhấn "Đóng khay"
    Client->>API: POST /tray/close
    activate API
    Note over API: tray_open = false
    API->>Tray: Publish Bool(data=false)
    Tray->>Robot: Đóng khay
    API-->>Client: 200 OK {"status":"ok","tray":"closed"}
    deactivate API
```

---

### 2.13 `POST /emergency-stop` & `POST /emergency-stop/release` — Dừng khẩn cấp

**Mô tả:**
- `/emergency-stop` — Dừng robot ngay, huỷ navigation, khoá mọi lệnh di chuyển.
- `/emergency-stop/release` — Giải phóng, cho phép điều khiển lại.

**Response `200 OK`:**
```json
{ "status": "ok", "emergency_stop": true }
{ "status": "ok", "emergency_stop": false }
```

**`GET /emergency-stop/status`:**
```json
{ "emergency_stop": false }
```

```mermaid
sequenceDiagram
    autonumber
    actor User
    participant Client as Client App
    participant API as API Server
    participant Nav2 as ROS2 Nav2
    participant CmdVel as /cmd_vel

    User->>Client: Nhấn nút E-STOP (khẩn cấp)
    Client->>API: POST /emergency-stop
    activate API
    Note over API: emergency_stop = true
    API->>CmdVel: Publish Twist {0,0,0}
    API->>Nav2: cancel_goal_async()
    Nav2-->>API: Goal Cancelled
    Note over API: move_state = "cancelled"
    API-->>Client: 200 OK {"emergency_stop":true}
    deactivate API
    Client->>User: Hiển thị cảnh báo E-STOP đỏ

    Note over Client: Sau khi kiểm tra an toàn...
    User->>Client: Nhấn "Giải phóng E-STOP"
    Client->>API: POST /emergency-stop/release
    activate API
    Note over API: emergency_stop = false
    API-->>Client: 200 OK {"emergency_stop":false}
    deactivate API
    Client->>User: Robot sẵn sàng hoạt động trở lại
```

---

### 2.14 `POST /system/restart` — Khởi động lại

**Mô tả:** Shutdown ROS2 node. Thường dùng kết hợp với systemd/supervisor để tự khởi động lại.

**Response `200 OK`:**
```json
{ "status": "restarting" }
```

---

## 3. WebSocket Endpoints

### Cách kết nối

**JavaScript:**
```javascript
const ws = new WebSocket('ws://ROBOT_IP:8090/ws/status');
ws.onmessage = (event) => {
  const data = JSON.parse(event.data);
  console.log(data);
};
```

**Python:**
```python
import asyncio, websockets, json

async def listen():
    async with websockets.connect('ws://ROBOT_IP:8090/ws/status') as ws:
        async for msg in ws:
            print(json.loads(msg))

asyncio.run(listen())
```

---

### 3.1 `WS /ws/pose` — Toạ độ real-time (200ms)

```json
{ "x": 1.23, "y": -0.45, "yaw_deg": 90.0 }
```

### 3.2 `WS /ws/battery` — Pin (2s)

```json
{ "percentage": 85.0, "voltage": 24.1, "current": -1.2, "status": "discharging" }
```

### 3.3 `WS /ws/speed` — Tốc độ (200ms)

```json
{ "linear_x": 0.3, "linear_y": 0.0, "angular_z": 0.0 }
```

### 3.4 `WS /ws/status` — Tổng hợp (500ms)

```json
{
  "pose": { "x": 1.23, "y": -0.45, "yaw_deg": 90.0 },
  "speed": { "linear_x": 0.3, "linear_y": 0.0, "angular_z": 0.0 },
  "battery": { "percentage": 85.0, "voltage": 24.1, "current": -1.2, "status": "discharging" },
  "move_state": "moving",
  "emergency_stop": false,
  "tray_open": false,
  "current_map": "lobby_v5"
}
```

```mermaid
sequenceDiagram
    autonumber
    participant Client as Client App
    participant API as API Server
    participant Odom as /odom (ROS2)
    participant Bat as /battery_state (ROS2)
    participant CmdVel as /cmd_vel (ROS2)

    Client->>API: WS Connect ws://.../ws/status

    loop Mỗi 500ms (ROS2 timer)
        Odom-->>API: Odometry callback → cập nhật pose
        CmdVel-->>API: Cmd_vel callback → cập nhật speed
        Bat-->>API: Battery callback → cập nhật battery
        API-->>Client: JSON broadcast<br/>{pose, speed, battery, move_state, ...}
        Client->>Client: Cập nhật UI (bản đồ, pin, tốc độ)
    end

    Client->>API: WS Disconnect
    Note over API: Xoá client khỏi danh sách broadcast
```

---

## 4. Luồng tích hợp đầy đủ — Android App

```mermaid
sequenceDiagram
    autonumber
    actor User
    participant App as Android App
    participant API as API Server
    participant Nav2 as ROS2 Nav2
    participant Robot as Robot (Gazebo/Vật lý)

    App->>API: GET /maps/ → Tải danh sách bản đồ
    App->>API: GET /robot/status → Kiểm tra trạng thái ban đầu
    App->>API: WS Connect /ws/status → Bắt đầu nhận dữ liệu real-time

    User->>App: Đặt vị trí ban đầu trên bản đồ
    App->>API: POST /chassis/pose {x,y,orientation}
    API->>Robot: Publish /initialpose (AMCL tái định vị)

    User->>App: Chọn điểm đích
    App->>API: POST /chassis/moves {type:"standard", target_x, target_y, target_ori}
    API->>Nav2: NavigateToPose action goal
    Nav2-->>API: Goal Accepted
    API-->>App: 200 OK

    loop Đang di chuyển
        Robot->>Nav2: /odom feedback
        Nav2->>API: Action feedback
        API-->>App: WS {"move_state":"moving","pose":{...}}
        App->>User: Cập nhật vị trí trên bản đồ
    end

    alt Đến đích
        Nav2-->>API: SUCCEEDED
        API-->>App: WS {"move_state":"completed"}
        App->>User: "Đã đến đích!"
        App->>API: POST /tray/open
    else Sự cố
        User->>App: Nhấn E-STOP
        App->>API: POST /emergency-stop
        API->>Nav2: Cancel goal
        API->>Robot: /cmd_vel {0,0,0}
        API-->>App: WS {"move_state":"cancelled","emergency_stop":true}
    end
```

---

## 5. Cấu hình Launch Parameters

| Parameter | Mặc định | Mô tả |
|-----------|---------|-------|
| `host` | `0.0.0.0` | Địa chỉ bind server |
| `port` | `8090` | Cổng HTTP/WebSocket |
| `maps_dir` | `<h3_slam>/maps` | Thư mục chứa bản đồ |
| `charge_x` | `0.0` | Toạ độ X trạm sạc |
| `charge_y` | `0.0` | Toạ độ Y trạm sạc |
| `charge_yaw_deg` | `0.0` | Góc quay tại trạm sạc (độ) |

```bash
ros2 launch h3_api_server h3_api_server.launch.py \
  host:=0.0.0.0 \
  port:=8090 \
  charge_x:=1.5 \
  charge_y:=-0.5 \
  charge_yaw_deg:=180.0
```

---

## 6. Mã lỗi HTTP

| Code | Ý nghĩa | Ví dụ |
|------|---------|-------|
| `200` | Thành công | — |
| `409` | Xung đột trạng thái | Robot đang di chuyển / E-stop đang bật |
| `422` | Dữ liệu không hợp lệ | Thiếu `target_x` khi `type=standard` |
| `500` | Lỗi server | Nav2 không phản hồi |
