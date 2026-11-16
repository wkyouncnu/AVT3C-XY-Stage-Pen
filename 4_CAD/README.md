# 4_CAD — Robot geometry (CAD)

[한국어](README_KO.md) · [Tiếng Việt](README_VI.md)

| File | Content |
|---|---|
| [AVT-3C.stl](AVT-3C.stl) | AVT-3C robot outline, 420 × 343 × 100 mm (3.5 MB) |

- Used by: `1_simulation/XY_animate.m` (3D real-time view) and `stl_file` in `XY_setup.m`.
- All models and scripts run without the STL; only the robot in the animation is then drawn as a 420 × 343 × 120 mm box.
- The track contact length `l_trk = 0.30 m` is estimated from the STL length of 420 mm (`XY_setup.m`).
- Opening the file on GitHub shows an interactive 3D view.
