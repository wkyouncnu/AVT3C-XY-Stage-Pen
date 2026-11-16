# 4_CAD — Hình dạng robot (CAD)

[한국어](README_KO.md) · [English](README.md)

| Tệp | Nội dung |
|---|---|
| [AVT-3C.stl](AVT-3C.stl) | Hình dạng ngoài của robot AVT-3C, 420 × 343 × 100 mm (3.5 MB) |

- Nơi sử dụng: `1_simulation/XY_animate.m` (màn hình 3D thời gian thực) và `stl_file` trong `XY_setup.m`.
- Mọi mô hình và script vẫn chạy khi không có STL; khi đó chỉ robot trong hoạt ảnh được vẽ bằng hộp 420 × 343 × 120 mm.
- Chiều dài tiếp xúc của xích `l_trk = 0.30 m` được ước lượng từ chiều dài STL 420 mm (`XY_setup.m`).
- Mở tệp trên GitHub sẽ thấy hình 3D có thể xoay.
