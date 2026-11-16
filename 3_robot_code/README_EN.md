# 3_robot_code — Code running on the robot computer (NUC)

[한국어](README.md) · [Tiếng Việt](README_VI.md) · Full manual: [../README_EN.md](../README_EN.md) (Chapter 10, Chapters 21–22)

## zaber_stage_node.py

Node that connects the two-axis Zaber X-LSM-E stage to ROS 2, wired directly to the NUC over RS-232 (115200 bps).
The position servo runs inside the Zaber firmware; this node only sends position commands and reads positions (the control law is PenComp in Simulink).

| Topic | Type | Direction | Content |
|---|---|---|---|
| `/stage/target_pos` | geometry_msgs/Point | subscribe | x, y [mm] target. The last value is sent as `move abs` at `cmd_rate` |
| `/stage/cmd_vel` | geometry_msgs/Vector3 | subscribe | x, y [mm/s]. Stops if no new command arrives within `cmd_timeout` |
| `/stage/stop` | std_msgs/Empty | subscribe | Decelerate and stop |
| `/stage/pos` | geometry_msgs/PointStamped | publish | x, y [mm] at `publish_rate` Hz |

| Parameter | Default | Meaning |
|---|---|---|
| `port` | `/dev/ttyUSB0` | Serial port |
| `x_device`, `y_device` | 1, 2 | Daisy-chain addresses |
| `publish_rate` | 20.0 Hz | `/stage/pos` publish rate |
| `cmd_rate` | 25.0 Hz | `move abs` send rate |
| `cmd_deadband` | 0.001 mm | Smaller changes are not resent |
| `cmd_timeout` | 0.5 s | Velocity-command watchdog |
| `home_on_start` | True | Home the axes at start |

## Running

```bash
pip install zaber-motion
python3 zaber_stage_node.py --ros-args -p port:=/dev/ttyUSB0
```

- Axes: X axis = robot body $x_B$ (forward), Y axis = $y_B$ (left). 0 = home, 50.8 mm = centre, 101.6 mm = end
- Stage specifications and commands: [../5_references/manuals/](../5_references/manuals/)
