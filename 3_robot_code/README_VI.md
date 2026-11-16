# 3_robot_code — Mã chạy trên máy tính robot (NUC)

[한국어](README.md) · [English](README_EN.md) · Tài liệu đầy đủ: [../README_VI.md](../README_VI.md) (Chương 10, Chương 21–22)

## zaber_stage_node.py

Nút kết nối bàn trượt hai trục Zaber X-LSM-E với ROS 2, nối trực tiếp với NUC qua RS-232 (115200 bps).
Servo vị trí nằm trong firmware của Zaber; nút này chỉ gửi lệnh vị trí và đọc vị trí (luật điều khiển là PenComp trong Simulink).

| Topic | Kiểu | Hướng | Nội dung |
|---|---|---|---|
| `/stage/target_pos` | geometry_msgs/Point | đăng ký | Mục tiêu x, y [mm]. Giá trị cuối được gửi bằng `move abs` với tần số `cmd_rate` |
| `/stage/cmd_vel` | geometry_msgs/Vector3 | đăng ký | x, y [mm/s]. Dừng nếu không có lệnh mới trong `cmd_timeout` |
| `/stage/stop` | std_msgs/Empty | đăng ký | Giảm tốc và dừng |
| `/stage/pos` | geometry_msgs/PointStamped | phát | x, y [mm] với tần số `publish_rate` Hz |

| Tham số | Mặc định | Ý nghĩa |
|---|---|---|
| `port` | `/dev/ttyUSB0` | Cổng nối tiếp |
| `x_device`, `y_device` | 1, 2 | Địa chỉ trong chuỗi daisy-chain |
| `publish_rate` | 20.0 Hz | Tần số phát `/stage/pos` |
| `cmd_rate` | 25.0 Hz | Tần số gửi `move abs` |
| `cmd_deadband` | 0.001 mm | Thay đổi nhỏ hơn giá trị này không gửi lại |
| `cmd_timeout` | 0.5 s | Bộ giám sát lệnh tốc độ |
| `home_on_start` | True | Đưa các trục về vị trí gốc (home) khi khởi động |

## Chạy

```bash
pip install zaber-motion
python3 zaber_stage_node.py --ros-args -p port:=/dev/ttyUSB0
```

- Trục: trục X = $x_B$ của thân robot (tiến), trục Y = $y_B$ (trái). 0 = vị trí gốc, 50.8 mm = giữa, 101.6 mm = cuối hành trình
- Thông số và lệnh của bàn trượt: [../5_references/manuals/](../5_references/manuals/)
