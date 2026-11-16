# 5_references — Tài liệu tham khảo

[한국어](README_KO.md) · [English](README.md)

## Tài liệu bàn trượt (Zaber)

| Tệp | Nội dung |
|---|---|
| [manuals/X-LSM-E-Manual.pdf](manuals/X-LSM-E-Manual.pdf) | Hướng dẫn sử dụng dòng X-LSM-E (lắp đặt, đấu dây, lệnh, thông số) |
| [manuals/X-LSM-E-Datasheet.pdf](manuals/X-LSM-E-Datasheet.pdf) | Bảng thông số X-LSM-E (hành trình, tốc độ, lực đẩy, độ chính xác) |

- Hai trục X và Y dùng hai bàn trượt cùng dòng, nên một tài liệu áp dụng cho cả hai trục.
- Giao thức lệnh (ASCII): <https://www.zaber.com/protocol-manual?protocol=ASCII>

## Bài báo tham khảo

| Số | Bài báo | Nơi sử dụng trong tài liệu |
|---|---|---|
| [1] | T. Zou, J. Angeles, F. Hassani, "Dynamic modeling and trajectory tracking control of unmanned tracked vehicles," *Robotics and Autonomous Systems*, vol. 110, pp. 102–111, 2018. [doi:10.1016/j.robot.2018.09.008](https://doi.org/10.1016/j.robot.2018.09.008) | Động học và động lực học robot bánh xích, dấu của mô-men cản quay, vòng ngoài (Mục 8.2–8.5) |
| [2] | A. D. Sabiha, M. A. Kamel, E. Said, W. M. Hussein, "ROS-based trajectory tracking control for autonomous tracked vehicle using optimized backstepping and sliding mode control," *Robotics and Autonomous Systems*, vol. 152, 104058, 2022. [doi:10.1016/j.robot.2022.104058](https://doi.org/10.1016/j.robot.2022.104058) | Đối chiếu động học và động lực học, lỗi đánh máy trong bài báo gốc (Mục 8.2–8.3), so sánh bộ điều khiển (Chương 20) |

- Hai bài báo thuộc bản quyền nhà xuất bản, nên tệp PDF không được đưa lên kho mã công khai. Bản lưu nội bộ của phòng thí nghiệm nằm trong `papers/`, được loại trừ bởi `.gitignore`.
