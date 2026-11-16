# 5_references — Reference material

[한국어](README_KO.md) · [Tiếng Việt](README_VI.md)

## Stage manuals (Zaber)

| File | Content |
|---|---|
| [manuals/X-LSM-E-Manual.pdf](manuals/X-LSM-E-Manual.pdf) | X-LSM-E series user manual (installation, wiring, commands, specifications) |
| [manuals/X-LSM-E-Datasheet.pdf](manuals/X-LSM-E-Datasheet.pdf) | X-LSM-E datasheet (travel, speed, thrust, accuracy) |

- The X and Y axes use two stages of the same series, so one manual applies to both axes.
- Command protocol (ASCII): <https://www.zaber.com/protocol-manual?protocol=ASCII>

## Reference papers

| No. | Paper | Used in the manual |
|---|---|---|
| [1] | T. Zou, J. Angeles, F. Hassani, "Dynamic modeling and trajectory tracking control of unmanned tracked vehicles," *Robotics and Autonomous Systems*, vol. 110, pp. 102–111, 2018. [doi:10.1016/j.robot.2018.09.008](https://doi.org/10.1016/j.robot.2018.09.008) | Tracked-robot kinematics and dynamics, turning-resistance sign, outer loop (Sections 8.2–8.5) |
| [2] | A. D. Sabiha, M. A. Kamel, E. Said, W. M. Hussein, "ROS-based trajectory tracking control for autonomous tracked vehicle using optimized backstepping and sliding mode control," *Robotics and Autonomous Systems*, vol. 152, 104058, 2022. [doi:10.1016/j.robot.2022.104058](https://doi.org/10.1016/j.robot.2022.104058) | Kinematics and dynamics cross-check and a typo in the original paper (Sections 8.2–8.3), controller comparison (Chapter 20) |

- Both papers are under publisher copyright, so the PDFs are not in the public repository. The lab copy is kept in `papers/`, which is excluded by `.gitignore`.
