# 1_simulation — Mô hình Simulink và script

[한국어](README.md) · [English](README_EN.md) · Tài liệu đầy đủ: [../README_VI.md](../README_VI.md) (Chương 13–17)

## Tệp

| Tệp | Nội dung |
|---|---|
| `XY_setup.m` | Toàn bộ tham số. **Chạy đầu tiên** |
| `build_xy_models.m` | Bộ tạo mô hình. Các tệp `.slx` dưới đây chỉ được tạo bằng tệp này (không sửa tay) |
| `XY_1_pen_sim.slx` | Thí nghiệm ảo: bộ điều khiển + mô hình đối tượng robot/bàn trượt + cảm biến, vòng lặp ROS 2 (`ROS_DOMAIN_ID` 77) |
| `XY_2_pen_ros2.slx` | Robot thật: chỉ bộ điều khiển, nối với robot và bàn trượt thật qua cùng các topic (`ROS_DOMAIN_ID` 0) |
| `XY_1_pen_sim_v2.slx`, `XY_2_pen_ros2_v2.slx` | Phiên bản 2: có vòng PI trong (tốc độ và tốc độ góc yaw) của mô hình robot thật (Mục 21.2) |
| `XY_1_pen_sim_run.m` | Chạy thí nghiệm ảo một lần |
| `XY_run_all.m` | Thiết lập → dựng → chạy → hình → video; kết quả từng bước trong `logs/status.txt` |
| `XY_compare.m`, `XY_compare_plot.m` | So sánh 10 phương án thiết kế (Chương 20) |
| `XY_animate.m`, `XY_stop.m` | Màn hình thời gian thực cho nút Run; hình kết quả khi dừng |
| `XY_record_live.m` | Video bảng điều khiển (kết xuất ngoài màn hình) |
| `XY_plot.m` | Hình kết quả (`img/XY_1_result_*.png`) |
| `img/`, `video/` | Hình mô hình, đồ thị kết quả, video |

## Chạy

```matlab
XY_setup                      % tham số
build_xy_models               % hai mô hình phiên bản 1 (phiên bản 2: build_xy_models([1 2], 2))
open_system('XY_1_pen_sim')   % nút Run → màn hình thời gian thực, hình kết quả khi kết thúc
```

- Yêu cầu: MATLAB/Simulink R2024b, ROS Toolbox, Stateflow
- Kết quả chuẩn (30 mm/s): sai số ngang của bút RMS khoảng 1.2 mm, khoảng 14–15 mm khi không bù
- Nếu không có `../4_CAD/AVT-3C.stl`, robot trong hoạt ảnh được vẽ bằng hộp 420 × 343 × 120 mm
- Chú thích trong mọi tệp `.m` và trong các khối MATLAB Function của mô hình được viết bằng tiếng Hàn và tiếng Anh
