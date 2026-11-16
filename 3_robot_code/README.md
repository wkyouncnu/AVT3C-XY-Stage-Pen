# 3_robot_code — 로봇 컴퓨터(NUC)에서 도는 코드

[English](README_EN.md) · [Tiếng Việt](README_VI.md) · 전체 매뉴얼: [../README.md](../README.md) (10 장, 21–22 장)

## zaber_stage_node.py

Zaber X-LSM-E 두 축 스테이지와 ROS 2 를 잇는 노드. NUC 에 RS-232(115200 bps)로 직결한다.
위치 서보는 Zaber 펌웨어 안에 있고, 이 노드는 위치 명령을 보내고 위치를 읽는 다리 역할만 한다(제어 법칙은 Simulink PenComp).

| 토픽 | 형식 | 방향 | 내용 |
|---|---|---|---|
| `/stage/target_pos` | geometry_msgs/Point | 구독 | x, y [mm] 목표. 마지막 값을 `cmd_rate` 로 `move abs` |
| `/stage/cmd_vel` | geometry_msgs/Vector3 | 구독 | x, y [mm/s]. `cmd_timeout` 동안 새 명령이 없으면 정지 |
| `/stage/stop` | std_msgs/Empty | 구독 | 감속 정지 |
| `/stage/pos` | geometry_msgs/PointStamped | 발행 | x, y [mm], `publish_rate` Hz |

| 파라미터 | 기본값 | 의미 |
|---|---|---|
| `port` | `/dev/ttyUSB0` | 직렬 포트 |
| `x_device`, `y_device` | 1, 2 | 데이지 체인 주소 |
| `publish_rate` | 20.0 Hz | `/stage/pos` 발행 주기 |
| `cmd_rate` | 25.0 Hz | `move abs` 전송 주기 |
| `cmd_deadband` | 0.001 mm | 이보다 작은 변화는 다시 보내지 않음 |
| `cmd_timeout` | 0.5 s | 속도 명령 워치독 |
| `home_on_start` | True | 시작할 때 홈 복귀 |

## 실행

```bash
pip install zaber-motion
python3 zaber_stage_node.py --ros-args -p port:=/dev/ttyUSB0
```

- 축 방향: X 축 = 로봇 바디 $x_B$ (전진), Y 축 = $y_B$ (왼쪽). 0 = 홈, 50.8 mm = 중앙, 101.6 mm = 끝
- 스테이지 사양·명령: [../5_references/manuals/](../5_references/manuals/)
