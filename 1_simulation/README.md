# 1_simulation — Simulink models and scripts

[한국어](README_KO.md) · [Tiếng Việt](README_VI.md) · Full manual: [../README.md](../README.md) (Chapters 13–17)

## Files

| File | Content |
|---|---|
| `XY_setup.m` | All parameters. **Run first** |
| `build_xy_models.m` | Model builder. The `.slx` files below are created only by this file (never edited by hand) |
| `XY_1_pen_sim.slx` | Virtual experiment: controller + robot/stage plant models + sensors, ROS 2 loopback (`ROS_DOMAIN_ID` 77) |
| `XY_2_pen_ros2.slx` | Real robot: controller only, connected to the real robot and stage through the same topics (`ROS_DOMAIN_ID` 0) |
| `XY_1_pen_sim_v2.slx`, `XY_2_pen_ros2_v2.slx` | Version 2: includes the inner PI (speed and yaw rate) of the real-robot model (manual Section 21.2) |
| `XY_1_pen_sim_run.m` | Runs the virtual experiment once |
| `XY_run_all.m` | Setup → build → run → figures → video; per-step result in `logs/status.txt` |
| `XY_compare.m`, `XY_compare_plot.m` | Comparison of 10 design choices (manual Chapter 20) |
| `XY_animate.m`, `XY_stop.m` | Real-time view for the Run button; result figures at stop |
| `XY_record_live.m` | Dashboard video (off-screen rendering) |
| `XY_plot.m` | Result figures (`img/XY_1_result_*.png`) |
| `img/`, `video/` | Model figures, result graphs, videos |

## Running

```matlab
XY_setup                      % parameters
build_xy_models               % the two version-1 models (version 2: build_xy_models([1 2], 2))
open_system('XY_1_pen_sim')   % Run button → real-time view, result figures at the end
```

- Requires MATLAB/Simulink R2024b, ROS Toolbox, Stateflow
- Baseline result (30 mm/s): pen cross-track error RMS about 1.2 mm, about 14–15 mm without compensation
- Without `../4_CAD/AVT-3C.stl` the robot in the animation is drawn as a 420 × 343 × 120 mm box
- Comments in every `.m` file and in the MATLAB Function blocks inside the models are in Korean and English
