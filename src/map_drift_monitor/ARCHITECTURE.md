# Kiến trúc Map Drift Monitor — Phương pháp Pose-Correction

> Tài liệu thiết kế cho hướng tiếp cận mới, thay thế cách đo `drift_ratio` vô hướng
> hiện tại bằng cách **ước lượng độ lệch định vị theo đơn vị vật lý (mét / độ)**.

---

## 1. Vấn đề của 3 node hiện tại

`map_drift_monitor_node`, `map_drift_likelihood_node`, `map_drift_raycast_node`
đều tính một con số vô hướng `drift_ratio` = tỉ lệ tia laser không khớp map **tại
pose hiện tại**, so với một `baseline` đo lúc khởi động, rồi alert khi vượt ngưỡng.

| # | Lỗi gốc | Hệ quả |
|---|---------|--------|
| ① | **Conflation** — `drift_ratio` trộn lẫn: lệch định vị thật, vật thể động, vùng chưa map, noise sensor | Tỉ lệ cao ≠ robot lệch → báo nhầm/bỏ sót |
| ② | **Không đơn vị vật lý** — chỉ là tỉ lệ 0..1 | Không biết lệch bao nhiêu mét / hướng nào; ngưỡng `0.22` tùy ý, đổi map là sai |
| ③ | **Calibration mong manh** — baseline đo 1 lần lúc start | Pose start lệch nhẹ → ngưỡng sai ở mọi nơi |
| ④ | **Đo residual, không đo correction** — chỉ xét pose hiện tại, không xét "có pose gần đó khớp tốt hơn không" | Lệch dọc hành lang mà vẫn khớp → bỏ sót; đông người mà định vị đúng → báo nhầm |
| ⑤ | **Mù với yaw** — tỉ lệ scan-map gần như không thấy lệch góc | Bỏ sót drift xoay |

> slam_toolbox (mode localization) **đã** làm scan-matching nội bộ. Các node cũ
> cài lại một phiên bản kém hơn của chính residual mà slam_toolbox tối thiểu hóa,
> thay vì đo **mức độ thất bại** của nó.

---

## 2. Ý tưởng cốt lõi

Đổi câu hỏi:

- **Cũ:** "Scan khớp map tốt thế nào *tại pose hiện tại*?"
- **Mới:** "Pose nào khớp map *tốt nhất*, và nó *cách pose hiện tại bao xa*?"

Khoảng cách giữa pose hiện tại và pose-khớp-tốt-nhất **chính là drift**, tính bằng
**mét và radian** — tự nhiên không cần baseline.

### Vì sao khử được nhiễu (điểm mấu chốt)

Vật thể động / đồ đạc / người làm **giảm điểm khớp tuyệt đối ở MỌI offset như
nhau** → **không dịch đỉnh** của hàm điểm. Drift đo bằng *vị trí đỉnh*, không phải
*độ cao đỉnh*. Nên nhiễu không gây báo lệch.

```
        điểm khớp theo offset dx
  cao │       ╭─╮                    Định vị ĐÚNG: đỉnh tại offset=0
      │      ╱   ╲                   → drift ≈ 0 m
      │ ___╱       ╲___
      └──────●──────────► dx (đỉnh tại 0)

  cao │              ╭─╮             LỆCH 0.4m: đỉnh dịch sang ≈0.4
      │ ___________╱   ╲__           → drift = 0.4 m, có hướng
      └──────●──────────┴──► dx
            0          đỉnh thật
```

---

## 3. Kiến trúc 5 tầng

```
        /map (OccupancyGrid, TRANSIENT_LOCAL)
            │  [1] một lần khi nhận map
            ▼
   ┌─────────────────────┐
   │ Distance Field (EDT)│  D[y,x] = khoảng cách tới tường gần nhất (m)
   └─────────────────────┘  (tái dùng _build_distance_field từ likelihood_node)
            │
   /scan_top ─┐
   TF map→base_link ─┐
            ▼  [2] mỗi chu kỳ (1–2 Hz)
   ┌────────────────────────────┐
   │ Lọc tia "ổn định" (gating)  │  Giữ tia có endpoint sát tường mapped.
   │                            │  Loại: tia vào vùng unknown; tia ngắn bất
   │                            │  thường (vật cản động trước tường).
   └────────────────────────────┘
            │  stable rays (toạ độ local trong base_link)
            ▼  [3] LÕI: tìm pose khớp nhất
   ┌────────────────────────────┐
   │ Local pose search (dx,dy,dθ)│  Quét lưới nhỏ quanh pose hiện tại
   │  coarse → fine + quadratic  │  (±search_xy, ±search_yaw).
   │  c* = (dx*, dy*, dθ*)       │  drift_xy=√(dx*²+dy*²), drift_yaw=|dθ*|
   └────────────────────────────┘
            │
            ▼  [4] Độ tin cậy (observability)
   ┌────────────────────────────┐
   │ Curvature/Hessian của đỉnh  │  Đỉnh nhọn → tin cậy cao.
   │                            │  Đỉnh phẳng (hành lang dài / mở) → KHÔNG
   │                            │  tin offset hướng đó → hạ confidence.
   └────────────────────────────┘
            │  (c*, confidence)
            ▼  [5] State machine + lọc thời gian
   ┌────────────────────────────┐
   │ OK → WARN → ALERT (hysteresis)│ Drift thật: bền vững, nhất quán hướng,
   │ N chu kỳ liên tiếp:          │ thường tăng. Transient: nhảy lung tung
   │ confidence cao & |c*|>ngưỡng │ rồi tắt → bị lọc.
   └────────────────────────────┘
            │
            ▼
   Publish: drift_x/y/yaw, confidence, state, correction;
            (tuỳ chọn) auto reset → /initialpose
```

### 3.1. Tầng [1] — Distance Field (likelihood field)

- Khi nhận `/map`: `EDT(~occupied) * resolution` → `D[y,x]` (mét).
- Clip ở `max_obstacle_distance` (vd 2.0 m) để giới hạn memory + tránh outlier.
- Điểm khớp 1 tia: `score_i = exp(-d_i² / 2σ²)` với `d_i = D[endpoint_i]`.
- Tổng điểm 1 pose: `S(pose) = Σ score_i` (chỉ trên tập tia ổn định).
- Tái dùng nguyên `_build_distance_field()` đã có trong `map_drift_likelihood_node`.

### 3.2. Tầng [2] — Lọc tia ổn định

Mục tiêu: loại nhiễu **trước** khi ước lượng, để hàm điểm sạch.

1. Bỏ tia không hữu hạn / ngoài `[range_min, range_max)`.
2. Tại pose hiện tại, tra `d_i = D[endpoint]`.
   - `d_i > stable_gate` (vd 0.5 m) → endpoint rơi vào vùng trống/unknown →
     **không phải tường mapped** → loại (có thể là vùng thay đổi / chưa map).
3. (Tuỳ chọn) so với ray-cast `d_map`: tia ngắn hơn nhiều so với tường kỳ vọng →
   vật cản động chắn trước tường → loại. (Tái dùng logic raycast đã có.)
4. Yêu cầu tối thiểu `min_stable_rays` (vd 40) mới đánh giá; thiếu → confidence=0.

### 3.3. Tầng [3] — Local pose search (lõi)

Quét perturbation quanh pose hiện tại `T0 = (x0, y0, θ0)`:

```
for dθ in linspace(-search_yaw, +search_yaw, Nθ):
    xform stable rays bằng (dx, dy, dθ)  # vectorized numpy
    for (dx, dy) in lưới [-search_xy .. +search_xy]:
        S(dx,dy,dθ) = Σ exp(-D[endpoint(dx,dy,dθ)]² / 2σ²)
c* = argmax S
```

- **Coarse → fine:** lưới thô (vd bước 0.10 m, 3°) tìm vùng đỉnh, rồi lưới mịn
  quanh đó (bước 0.02 m, 0.5°). Hoặc fit parabol 1D quanh đỉnh thô để nội suy
  sub-cell → không cần lưới quá dày.
- **Chi phí:** ~`Nx*Ny*Nθ` lần đánh giá; mỗi lần là tra distance-field cho ~Nrays
  điểm bằng numpy. Lưới 9×9×9 × 200 tia ≈ 150k lookups → nhẹ ở 1–2 Hz.
- Kết quả: `drift_xy = √(dx*² + dy*²)`, `drift_yaw = |dθ*|`,
  correction `c* = (dx*, dy*, dθ*)`.

### 3.4. Tầng [4] — Độ tin cậy (observability)

Fit mặt bậc 2 cho `S` quanh đỉnh → Hessian `H` (2×2 cho x,y; xét riêng θ).

- Trị riêng lớn của `H` (đỉnh nhọn theo hướng đó) → ràng buộc tốt → tin cậy cao.
- Trị riêng nhỏ (đỉnh phẳng) → hướng đó **không quan sát được** (hành lang dài,
  phòng trống) → **không tin offset hướng đó**, hạ `confidence`.
- `confidence ∈ [0,1]` tổng hợp từ: số tia ổn định, độ nhọn đỉnh theo hướng có
  offset, và độ cao đỉnh tương đối.
- Đây là thứ giải quyết false-alarm ở hành lang đối xứng — điều ngưỡng đơn không
  bao giờ làm được.

### 3.5. Tầng [5] — State machine + lọc thời gian

```
trạng thái: OK ──(|c*|>warn_m & conf>conf_min, k chu kỳ)──► WARN
            WARN ─(|c*|>alert_m & conf>conf_min, N chu kỳ)─► ALERT
            ALERT ─(|c*|<exit_m, M chu kỳ)──────────────────► OK   (hysteresis)
```

- Ngưỡng theo **mét/độ** (vd `warn=0.15 m / 5°`, `alert=0.30 m / 10°`).
- Yêu cầu **nhất quán hướng**: vector correction các chu kỳ phải cùng chiều (drift
  thật bền vững; transient nhảy hướng → loại).
- Hysteresis: ngưỡng vào ALERT > ngưỡng ra → không rung trạng thái.
- (Tuỳ chọn) so correction với tốc độ odom: nhảy đột ngột lớn = kidnap hoặc
  transient → xử lý riêng.

---

## 4. Tham số (đề xuất)

```yaml
map_drift_corrector_node:
  ros__parameters:
    scan_topic: "/scan_top"
    map_topic: "/map"
    map_frame: "map"
    base_frame: "base_link"
    odom_frame: "odom"
    check_rate: 2.0

    # Likelihood field
    sigma: 0.20
    occupancy_threshold: 50
    max_obstacle_distance: 2.0

    # Lọc tia ổn định
    stable_gate: 0.50          # m, endpoint xa tường hơn mức này → loại
    reject_dynamic: true       # loại tia ngắn bất thường (vật cản động)
    min_stable_rays: 40
    ray_skip: 2

    # Local pose search
    search_xy: 0.50            # m, bán kính quét tịnh tiến
    search_yaw: 0.175          # rad (~10°), bán kính quét xoay
    coarse_step_xy: 0.10
    coarse_step_yaw: 0.0524    # ~3°
    fine_refine: true          # nội suy parabol quanh đỉnh

    # Ngưỡng vật lý
    warn_drift_m: 0.15
    alert_drift_m: 0.30
    warn_drift_yaw: 0.087      # ~5°
    alert_drift_yaw: 0.175     # ~10°
    exit_drift_m: 0.10         # hysteresis thoát ALERT
    min_confidence: 0.5

    # Lọc thời gian
    warn_consecutive: 3
    alert_consecutive: 5
    exit_consecutive: 5

    # Hành động khi ALERT
    auto_correct: false        # true → tự publish /initialpose = pose + c*
    auto_correct_min_confidence: 0.8
```

---

## 5. Giao tiếp (Topics)

### Subscribe
| Topic | Type | Ghi chú |
|-------|------|---------|
| `/map` | `nav_msgs/OccupancyGrid` | QoS TRANSIENT_LOCAL |
| `/scan_top` | `sensor_msgs/LaserScan` | |
| TF `map→base_link` | — | từ slam_toolbox localization |
| `/odom` (tuỳ chọn) | `nav_msgs/Odometry` | cross-check tốc độ correction |

### Publish
| Topic | Type | Ghi chú |
|-------|------|---------|
| `/map_drift/drift_xy` | `std_msgs/Float32` | độ lệch tịnh tiến (m) |
| `/map_drift/drift_yaw` | `std_msgs/Float32` | độ lệch xoay (rad) |
| `/map_drift/drift_severity` | `std_msgs/Float32` | **tỉ lệ lệch chuẩn hoá**: `max(drift_m/alert_drift_m, drift_yaw/alert_drift_yaw)`. <0.5 ổn, 0.5–1.0 vùng WARN, ≥1.0 mức ALERT |
| `/map_drift/stable_ratio` | `std_msgs/Float32` | tỉ lệ tia khớp map (n_stable/total). Thấp → dữ liệu nghèo / lệch nặng |
| `/map_drift/match_quality` | `std_msgs/Float32` | mức khớp scan-map tại pose tốt nhất (0..1) |
| `/map_drift/correction` | `geometry_msgs/PoseStamped` | **pose giả định khớp-nhất** (gợi ý reset) |
| `/map_drift/scan_corrected` | `visualization_msgs/Marker` | **laser giả định** theo pose khớp-nhất (XANH) — dán vào tường map |
| `/map_drift/scan_current` | `visualization_msgs/Marker` | laser theo pose hiện tại (ĐỎ) — lệch khỏi tường khi drift |
| `/map_drift/confidence` | `std_msgs/Float32` | 0..1 |
| `/map_drift/state` | `std_msgs/String` | OK / WARN / ALERT |
| `/map_drift/alert` | `std_msgs/Bool` | |
| `/map_drift/reset_pose_request` | `std_msgs/Bool` | giữ tương thích node cũ |
| `/initialpose` | `PoseWithCovarianceStamped` | chỉ khi `auto_correct=true` |
| `/map_drift/score_grid` (debug) | `nav_msgs/OccupancyGrid` | hình ảnh hoá hàm điểm để chỉnh tham số |

### Debug trực quan trên RViz

Thêm các display (Fixed Frame = `map`):
- **Map** → `/map`
- **Marker** → `/map_drift/scan_current` (ĐỎ): laser tại pose hiện tại
- **Marker** → `/map_drift/scan_corrected` (XANH): laser tại pose giả định khớp-nhất
- **Pose** → `/map_drift/correction`: mũi tên pose giả định khớp-nhất

Đọc hình: khi **định vị đúng** → đỏ và xanh trùng nhau, cùng dán vào tường. Khi
**lệch** → đỏ tách khỏi tường, xanh vẫn dán vào tường; khoảng cách đỏ→xanh chính
là độ lệch (`drift_xy`), hướng đỏ→xanh là hướng cần kéo pose về.

Tắt marker để tiết kiệm băng thông: đặt `publish_debug_markers: false`.

---

## 6. So sánh nhanh

| Tiêu chí | Cách cũ (ratio) | Cách mới (pose-correction) |
|----------|-----------------|----------------------------|
| Đơn vị | tỉ lệ 0..1 (vô nghĩa vật lý) | **mét / độ** |
| Calibration | bắt buộc, mong manh | **không cần** |
| Vật thể động | gây false alarm | **miễn nhiễm** (không dịch đỉnh) |
| Lệch yaw | mù | **phát hiện được** |
| Hành lang đối xứng | báo nhầm | **hạ confidence, không báo** |
| Output | chỉ alert | alert + **correction vector** dùng được |
| CPU | thấp | trung bình (vẫn nhẹ với numpy) |

---

## 7. Hạn chế còn lại

- **Đỉnh cục bộ:** nếu drift lớn hơn `search_xy` thì đỉnh thật nằm ngoài lưới quét
  → chỉ thấy một phần. Xử lý: khi correction "đụng biên" lưới liên tục → coi là
  drift lớn / kidnap, báo ALERT mức cao + (tuỳ chọn) mở rộng lưới.
- **Map sai/cũ:** nếu chính map sai thì pose-khớp-tốt-nhất cũng sai. Đây là giả
  định nền (map tĩnh, đúng) — giống mọi phương pháp localization.
- **Vẫn là monitor:** không thay thế slam_toolbox; chỉ đo & cảnh báo mức độ lệch.
```
