# _tools — Công cụ tạo, sắp xếp và kiểm tra mô hình Simulink

[한국어](README.md) · [English](README_EN.md)

Tập hàm MATLAB được `1_simulation/build_xy_models.m` gọi khi dựng mô hình. Đây là cùng bộ công cụ dùng trong các môn Simulink của phòng thí nghiệm (capstone và cao học); phần trợ giúp của từng tệp (`help tên_hàm`) viết bằng tiếng Hàn và tiếng Anh. Hiếm khi cần gọi trực tiếp.

| Nhóm | Hàm | Chức năng |
|---|---|---|
| Khối | `sl_fresh`, `sl_const`, `sl_gain`, `sl_sat`, `sl_pi`, `sl_fcn`, `set_mlfcn`, `sl_goto`, `sl_from`, `sl_note`, `sl_solver`, `add_sum`, `add_ssa`, `add_subsys` | Mô hình trống, khối cơ bản, mã MATLAB Function, chú thích, thiết lập ode4 bước cố định |
| Hộp | `add_logging_box`, `add_animate_box`, `add_switch_box`, `add_cmd_publisher`, `add_pose_subscriber` | Hộp ghi dữ liệu và vẽ thời gian thực không có cổng, công tắc, hộp phát/đăng ký ROS |
| Bố trí | `gnc_chain`, `gnc_roles`, `lay_chain`, `lay_boxes`, `pack_boxes`, `lay_sinks`, `lay_notes`, `align_to`, `fit_span`, `row_feed`, `straighten_chain`, `nudge_labels`, `mss_style` | Bố trí chuỗi tín hiệu từ trái sang phải, kích thước và vị trí khối |
| Đường nối | `draw_line`, `cut_line`, `lane_line`, `lay_links`, `lay_feedback`, `settle_links`, `flatten_lines`, `square_lines`, `snug_tags`, `drop_tag`, `feed_from`, `tag_feedback`, `from_name`, `keep_signals`, `spot_free`, `port_xy`, `name_box` | Đường thẳng, làn hồi tiếp, bố trí thẻ Goto/From |
| Màu | `gnc_colour`, `paint_roles` | Màu nền khối theo vai trò |
| Kiểm tra (đạt = 0) | `check_lines`, `check_flow`, `check_tags`, `check_colour`, `check_overlaps`, `check_straight`, `check_sums`, `check_gains`, `check_discrete`, `check_fill`, `fit_gains` | Chỗ gấp, chồng lấn, đường ngược chiều, tên thẻ, thiếu màu, khối rời rạc |
| Tất cả một lần | `tidy_model`, `tidy_layout`, `tidy_all` | Bố trí → đường nối → màu → kiểm tra theo thứ tự |
| Hình | `export_diagram`, `export_model_pngs`, `scope_legend` | PNG sơ đồ mô hình (`1_simulation/img/`), chú giải Scope |
| Khác | `step_metrics` | Độ vọt lố, thời gian xác lập, thời gian tăng của đáp ứng bậc thang |
