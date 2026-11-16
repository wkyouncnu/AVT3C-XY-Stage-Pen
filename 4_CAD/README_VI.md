# 4_CAD — Hình dạng robot (CAD)

[한국어](README.md) · [English](README_EN.md)

| Tệp | Nội dung | Kho mã |
|---|---|---|
| `AVT-3C.stl` | Hình dạng ngoài của robot AVT-3C, 420 × 343 × 100 mm | Không tải lên (thiết kế bảo mật) |

- Tệp STL chỉ được lưu trong thư mục chia sẻ nội bộ của phòng thí nghiệm. Nếu đặt vào thư mục này, tệp bị loại trừ bởi `.gitignore` và không được đẩy lên GitHub.
- Nơi sử dụng: `1_simulation/XY_animate.m` (màn hình 3D thời gian thực) và `stl_file` trong `XY_setup.m`.
- Mọi mô hình và script vẫn chạy khi không có STL; chỉ robot trong hoạt ảnh được vẽ bằng hộp 420 × 343 × 120 mm.
- Chiều dài tiếp xúc của xích `l_trk = 0.30 m` được ước lượng từ chiều dài STL 420 mm (`XY_setup.m`).
