# 4_CAD — Robot geometry (CAD)

[한국어](README.md) · [Tiếng Việt](README_VI.md)

| File | Content | Repository |
|---|---|---|
| `AVT-3C.stl` | AVT-3C robot outline, 420 × 343 × 100 mm | Not uploaded (confidential design) |

- The STL is kept only in the lab's internal shared folder. If placed in this folder it is excluded by `.gitignore` and never pushed to GitHub.
- Used by: `1_simulation/XY_animate.m` (3D real-time view) and `stl_file` in `XY_setup.m`.
- All models and scripts run without the STL; only the robot in the animation is drawn as a 420 × 343 × 120 mm box.
- The track contact length `l_trk = 0.30 m` is estimated from the STL length of 420 mm (`XY_setup.m`).
