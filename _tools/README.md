# _tools — Simulink model building, layout and check tools

[한국어](README_KO.md) · [Tiếng Việt](README_VI.md)

MATLAB functions called by `1_simulation/build_xy_models.m` when it builds the models. They are the same tools as in the lab's Simulink courses (capstone and graduate); each file's help (`help functionName`) is in Korean and English. They rarely need to be called directly.

| Category | Functions | Purpose |
|---|---|---|
| Blocks | `sl_fresh`, `sl_const`, `sl_gain`, `sl_sat`, `sl_pi`, `sl_fcn`, `set_mlfcn`, `sl_goto`, `sl_from`, `sl_note`, `sl_solver`, `add_sum`, `add_ssa`, `add_subsys` | Empty model, basic blocks, MATLAB Function code, notes, fixed-step ode4 setting |
| Boxes | `add_logging_box`, `add_animate_box`, `add_switch_box`, `add_cmd_publisher`, `add_pose_subscriber` | Port-less logging and live-plot boxes, switches, ROS publish/subscribe boxes |
| Layout | `gnc_chain`, `gnc_roles`, `lay_chain`, `lay_boxes`, `pack_boxes`, `lay_sinks`, `lay_notes`, `align_to`, `fit_span`, `row_feed`, `straighten_chain`, `nudge_labels`, `mss_style` | Left-to-right signal-chain layout, block size and position |
| Lines | `draw_line`, `cut_line`, `lane_line`, `lay_links`, `lay_feedback`, `settle_links`, `flatten_lines`, `square_lines`, `snug_tags`, `drop_tag`, `feed_from`, `tag_feedback`, `from_name`, `keep_signals`, `spot_free`, `port_xy`, `name_box` | Straight lines, feedback lanes, Goto/From tag placement |
| Colour | `gnc_colour`, `paint_roles` | Block background colour by role |
| Checks (pass = 0) | `check_lines`, `check_flow`, `check_tags`, `check_colour`, `check_overlaps`, `check_straight`, `check_sums`, `check_gains`, `check_discrete`, `check_fill`, `fit_gains` | Bends, overlaps, backward lines, tag names, missing colours, discrete blocks |
| All in one | `tidy_model`, `tidy_layout`, `tidy_all` | Layout → lines → colour → checks in sequence |
| Figures | `export_diagram`, `export_model_pngs`, `scope_legend` | Model diagram PNGs (`1_simulation/img/`), Scope legends |
| Other | `step_metrics` | Step-response overshoot, settling time, rise time |
