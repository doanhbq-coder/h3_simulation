# Elevator Navigation Package

Package này chứa thuật toán để robot di chuyển vào thang máy và công cụ visualize để debug.

## Nodes

### 1. free_space_goal
- **Chức năng**: Xử lý laser scan để tìm vùng free space trong thang máy và publish goal pose
- **Subscribe**: `/scan` (LaserScan)
- **Publish**: `/goal_pose` (PoseStamped)

### 3. simple_gui (Matplotlib)
- **Chức năng**: GUI đơn giản hiển thị free space và goal
- **Subscribe**: `/scan`, `/goal_pose`
- **Hiển thị**: Matplotlib window với real-time update

## Cách chạy

### GUI Đơn Giản (Khuyến nghị):
```bash
# Terminal 1: Chạy thuật toán
ros2 run elevator_navigation free_space_goal

# Terminal 2: Chạy GUI đơn giản
ros2 run elevator_navigation simple_gui
```

### RViz (Chi tiết hơn):
```bash
# Terminal 1: Chạy thuật toán
ros2 run elevator_navigation free_space_goal

# Terminal 2: Chạy visualizer cho RViz
ros2 run elevator_navigation elevator_visualizer

# Terminal 3: Mở RViz
rviz2
# Load config: src/elevator_navigation/rviz/elevator_debug.rviz
```

## GUI Đơn Giản - Hiển thị:

### 🎨 **Màu sắc:**
- **⚫ Chấm đen**: Vị trí robot (0,0)
- **🔴 Chấm đỏ nhỏ**: Laser scan points
- **🟢 Vùng xanh**: Free space regions (mỗi vùng màu khác nhau)
- **🟢 Chấm xanh lớn**: Goal pose (điểm đích)

### 📊 **Thông tin:**
- Title hiển thị số regions tìm được
- Update real-time mỗi 0.5 giây
- Grid 5m x 5m quanh robot

### 🎯 **Cách debug:**
1. Quan sát laser points (đỏ) → robot có scan được thang máy?
2. Quan sát vùng xanh → thuật toán detect free space đúng?
3. Quan sát chấm xanh lớn → goal pose có hợp lý?

## Màu sắc trong visualization

- 🔴 **Đỏ**: Laser scan points
- 🔴 **Đỏ cam**: Occupied cells (obstacles)
- 🟢 **Xanh lá**: Free regions (mỗi vùng có màu khác nhau)
- 🟢 **Xanh lá bóng**: Goal pose (quả bóng)

## Tham số có thể điều chỉnh

Trong `free_space_goal.py`:
- `robot_radius`: Bán kính robot (m) - ảnh hưởng đến inflate
- `grid_resolution`: Độ phân giải grid (m/cell)
- `grid_size`: Kích thước grid (cells)
- `roi_angle`: Góc ROI phía trước (radian)
- `max_range`: Khoảng cách max để phát hiện cabin (m)
- `min_area`: Diện tích tối thiểu cho vùng free (cells)

## Debug

- Kiểm tra topics: `ros2 topic list`
- Kiểm tra data: `ros2 topic echo /goal_pose`
- Xem logs: `ros2 run elevator_navigation free_space_goal` (có output screen)