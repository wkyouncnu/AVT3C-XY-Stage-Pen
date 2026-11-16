# _tools — Simulink 모델 생성·정리·검사 도구

[English](README_EN.md) · [Tiếng Việt](README_VI.md)

`1_simulation/build_xy_models.m` 이 모델을 만들 때 부르는 MATLAB 함수 모음. 연구실 Simulink 강의(캡스톤·대학원)와 같은 도구이며, 각 파일의 도움말(`help 함수이름`)은 한국어·영어 병기다. 직접 부를 일은 거의 없다.

| 분류 | 함수 | 하는 일 |
|---|---|---|
| 블록 만들기 | `sl_fresh`, `sl_const`, `sl_gain`, `sl_sat`, `sl_pi`, `sl_fcn`, `set_mlfcn`, `sl_goto`, `sl_from`, `sl_note`, `sl_solver`, `add_sum`, `add_ssa`, `add_subsys` | 빈 모델, 기본 블록, MATLAB Function 코드, 주석, 고정 간격 ode4 설정 |
| 상자 만들기 | `add_logging_box`, `add_animate_box`, `add_switch_box`, `add_cmd_publisher`, `add_pose_subscriber` | 포트 없는 기록·실시간 그림 상자, 스위치, ROS 발행·구독 상자 |
| 배치 | `gnc_chain`, `gnc_roles`, `lay_chain`, `lay_boxes`, `pack_boxes`, `lay_sinks`, `lay_notes`, `align_to`, `fit_span`, `row_feed`, `straighten_chain`, `nudge_labels`, `mss_style` | 왼쪽→오른쪽 신호 사슬 배치, 블록 크기·자리 맞춤 |
| 선 정리 | `draw_line`, `cut_line`, `lane_line`, `lay_links`, `lay_feedback`, `settle_links`, `flatten_lines`, `square_lines`, `snug_tags`, `drop_tag`, `feed_from`, `tag_feedback`, `from_name`, `keep_signals`, `spot_free`, `port_xy`, `name_box` | 선을 직선으로, 되먹임 통로, Goto/From 태그 배치 |
| 색 | `gnc_colour`, `paint_roles` | 역할별 블록 배경색 |
| 검사 (합격선 0) | `check_lines`, `check_flow`, `check_tags`, `check_colour`, `check_overlaps`, `check_straight`, `check_sums`, `check_gains`, `check_discrete`, `check_fill`, `fit_gains` | 꺾임·겹침·역방향 선, 태그 이름, 색 누락, 이산 블록 |
| 한 번에 | `tidy_model`, `tidy_layout`, `tidy_all` | 배치 → 선 → 색 → 검사를 차례로 실행 |
| 그림 | `export_diagram`, `export_model_pngs`, `scope_legend` | 모델 도면 PNG (`1_simulation/img/`), Scope 범례 |
| 기타 | `step_metrics` | 계단응답 오버슛·정착시간·상승시간 |
