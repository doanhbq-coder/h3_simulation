## h3_simulation 
- Thiết kế urdf đang có 1 số lỗi về physical
- Khi di chuyển bị trôi
- Nên áp dụng các thuật toán chưa phản ánh đúng thực tế
- Dùng robot_simulation nếu cần thiết

## File config nav2
### h3_nav2_params_v2_mppi
- chạy ổn định tốt, fix được phần xoay đúng hướng rồi với di chuyển
- Không đi được lùi vì sử dụng nav2_shim_rotation_controller

### h3_nav2_params_v3_mppi:
- Hoạt động tương đối tốt, ổn định
- Có thể đi lùi nên không xảy ra lỗi lệch hướng di chuyển ban đầu
- Có thể ra vào thang máy

### h3_nav2_params_v1_dwb:
- Hoạt động ổn định
- Không fix được lỗi di chuyển lệnh hướng ban đầu
- Ra vào thang máy ổn định