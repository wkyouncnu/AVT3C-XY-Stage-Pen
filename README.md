# AVT-3C 벽면 로봇 + Zaber XY 스테이지 직선 마킹 (Simulink ↔ ROS 2, 펜 기구학 보상)

- 작성: 충남대학교 자율운항시스템공학과 윤원근 교수
- 저장소: <https://github.com/wkyouncnu/AVT3C-XY-Stage-Pen>
- 매뉴얼(PDF): [docs/manual/AVT3C_XY_Stage_Pen.pdf](docs/manual/AVT3C_XY_Stage_Pen.pdf)

![시스템 구성](docs/img/diagram_overview.svg)

*그림 1. 시스템 구성*

## 목차

1. [개요](#1-개요)
2. [통신 구조](#2-통신-구조)
3. [제어기 계층 구조](#3-제어기-계층-구조)
4. [수학적 정식화](#4-수학적-정식화)
5. [운동모델](#5-운동모델)
6. [Simulink 모델 구성](#6-simulink-모델-구성)
7. [설치](#7-설치)
8. [실행](#8-실행)
9. [프로그램별 입출력](#9-프로그램별-입출력)
10. [검증](#10-검증)
11. [설계 선택 비교](#11-설계-선택-비교)
12. [실장비 적용](#12-실장비-적용)
13. [문제 해결](#13-문제-해결)
14. [확인 필요 사항](#14-확인-필요-사항)

---

## 1. 개요

### 1.1 목적

- 수직 철판 벽에 자석으로 붙는 궤도형 로봇 AVT-3C 위에 Zaber X-LSM100B 두 축 XY 스테이지를 얹고, 스테이지 끝 마커로 벽에 직선 하나를 그음
- 로봇(macro)은 진행과 큰 범위 이동을 맡고, 스테이지(micro)는 직선에서 벗어난 횡오차와 heading 오차를 지움
- 가상 실험 모델과 실기 모델이 같은 ROS 2 토픽 이름·메시지 형식을 쓰므로 시뮬레이션의 제어기를 그대로 실기에 연결함

### 1.2 주요 사양

| 항목 | 값 | 근거 |
|---|---|---|
| 그릴 직선 | A = (0.30, 0.70) m → B = (0.30, −0.50) m, 수평 1.2 m | `XY_setup.m` 2 절 |
| 로봇 | AVT-3C 궤도형, 외형 420 × 343 × 100 mm | CAD STL |
| 스테이지 | Zaber X-LSM100B-E03 × 2 (X 아래 축, Y 위 축), 배치 3안 (펜 = 로봇 중심) | 데이터시트, CAD |
| 스테이지 행정 · 최고속도 | 101.6 mm · 104 mm/s | 데이터시트 |
| 위치 센서 | ZeroKey RTLS 태그 1개 (로봇), 약 20 Hz | 실기 구성 |
| heading 센서 | Xsens IMU `/imu/data` (상대 요각, 철판이라 자력계 사용 불가) | 실기 구성 |
| 제어기 주기 · 운동모델 적분 스텝 | 0.01 s (100 Hz) · 0.5 ms | 실기 Simulink 와 같음 · 4.3 절 |
| 직선 구간 속도 | 30 mm/s (펜 다운 약 39.3 s) | `v_ref` |
| 결과 (기준 조건) | 펜 횡오차 RMS 1.17–1.18 mm, 최대 3.6–4.0 mm | 10 장 |
| 스테이지 없이 로봇만 | 펜 횡오차 RMS 14.45 mm, 최대 19.3 mm | 10 장 |

### 1.3 외부 인터페이스

| 구분 | 내용 |
|---|---|
| 제어기 입력 | `/zerokey/position`, `/zerokey/vel`, `/imu/data`, `/stage/pos` |
| 제어기 출력 | `/motor_pwm` (로봇), `/stage/target_pos` (스테이지) |
| 스테이지 노드 | `zaber_stage_node.py` — `/stage/target_pos` → RS-232 `move abs`, `get pos` → `/stage/pos` |
| ROS 2 도메인 | 가상 실험 77, 실기 0 |

### 1.4 저장소 구성

```
├── README.md                 이 문서 (원본)
├── docs/img/                 다이어그램 SVG, 수식 그림 PNG (src/make_fig_math.m)
├── docs/manual/              README 에서 생성한 PDF
├── 1_시뮬레이션/
│   ├── XY_setup.m            파라미터 (먼저 실행)
│   ├── build_xy_models.m     모델 생성기 — 두 .slx 는 이것으로만 만듦
│   ├── XY_1_pen_sim.slx      가상 실험 (제어기 + 운동모델 + 센서, ROS 2 루프백)
│   ├── XY_2_pen_ros2.slx     실기 (제어기만)
│   ├── XY_run_all.m          설정 → 빌드 → 실행 → 그림 → 동영상
│   ├── XY_compare.m          설계 선택 비교 (11 장), XY_compare_plot.m
│   ├── XY_animate.m          Run 버튼 실시간 대시보드, XY_stop.m (StopFcn: 결과 그림)
│   ├── XY_record_live.m      대시보드 동영상 (실행 뒤 기록 재생, 화면 밖 렌더링)
│   ├── XY_plot.m             결과 그림
│   ├── img/                  모델 그림, 결과 그래프
│   └── video/                결과 동영상
├── 3_로봇탑재코드/zaber_stage_node.py
└── _tools/                   Simulink 정리·검사 도구 (paint_roles, check_lines, check_flow, check_tags …)
```

---

## 2. 통신 구조

### 2.1 토픽

| 토픽 | 형식 | 발행 → 구독 | 주기 | 내용 |
|---|---|---|---|---|
| `/zerokey/position` | geometry_msgs/PointStamped | 센서 → 제어기 | 20 Hz | 태그 월드 좌표 [m]. x = 벽 수평(왼쪽 +), z = 높이 |
| `/zerokey/vel` | geometry_msgs/TwistStamped | 센서 → 제어기 | 20 Hz | 태그 속도 [m/s] (linear.x, linear.z 사용) |
| `/zerokey/orientation` | geometry_msgs/QuaternionStamped | 센서 → (실기 모델) | 20 Hz | ZeroKey INS 자세. 이 제어기는 사용하지 않음, 실기 모델 호환용 |
| `/imu/data` | sensor_msgs/Imu | 센서 → 제어기 | 100 Hz | orientation 쿼터니언 → 요각 |
| `/stage/pos` | geometry_msgs/PointStamped | 스테이지 노드 → 제어기 | 20 Hz | 스테이지 X, Y [mm] |
| `/motor_pwm` | geometry_msgs/Point | 제어기 → 로봇 | 100 Hz | x = 왼쪽, y = 오른쪽 PWM count (중립 1500 ± 400) |
| `/stage/target_pos` | geometry_msgs/Point | 제어기 → 스테이지 노드 | 100 Hz | 목표 X, Y [mm] (노드가 25 Hz 로 솎아 전송) |
| `/stage/cmd_vel`, `/stage/stop` | Vector3, Empty | 보조 → 스테이지 노드 | 필요 시 | 속도 명령 (워치독 0.5 s), 정지 |

- 가상 실험에서 운동모델·센서·스테이지 모델은 위 토픽을 실제로 발행하고, 제어기는 구독만 함
- 따라서 제어기는 참값(`q_true`, `x_s`)을 보지 못하고 실기와 같은 정보만으로 동작함

### 2.2 좌표 규약

| 기호 | 정의 |
|---|---|
| p₁ | world z, 높이 (천장 +) |
| p₂ | world x, 수평 (벽 정면에서 왼쪽 +) |
| ψ | p₁ 축에서 p₂ 축으로 도는 각 (천장 방향 0, 왼쪽 +90°) |
| 바디 | x_B 전진, y_B 좌측. R(ψ) = [cos ψ −sin ψ; sin ψ cos ψ] 가 바디 → 벽면 |
| 스테이지 | 축 1 = x_B, 축 2 = y_B, 단위 mm, 중앙 s_c = 50.8 mm |

- 실기 Simulink 의 `plane_project` 와 같은 규약

---

## 3. 제어기 계층 구조

![제어기 계층](docs/img/diagram_hierarchy.svg)

*그림 2. 제어기 계층 구조와 단별 신호*

### 3.1 단별 역할

| 단 | 블록 | 입력 | 출력 | 주기 |
|---|---|---|---|---|
| 0 측정 | RosIn → Nav | ZeroKey 위치·속도, IMU 쿼터니언 | `pose` = [p₁; p₂; ψ] | 0.01 s |
| 1 미션 | Guards → ModeChart → Waypoints | `pose` | `mode`, `WP_prev`, `WP_target`, `t_line`, `lineAB` | 0.01 s |
| 2 macro | RobotCtrl → PwmMap → RosOut | `WP_*`, `mode`, `t_line`, `pose`, `stage_pos_meas` | `v_cmd`, `w_cmd` → `/motor_pwm` | 연속 / 0.01 s 발행 |
| 2 micro | PenCtrl → RosOut | `lineAB`, `mode`, `pose` | `stage_target_pos` → `/stage/target_pos`, `pen_on` | 연속 / 0.01 s 발행 |
| 3 내부 루프 | ESC 속도 루프, Zaber 펌웨어 위치 루프 | `/motor_pwm`, `/stage/target_pos` | 궤도 속도, 스테이지 위치 | 장비 내부 |

### 3.2 상태 값

| 이름 | 크기 | 단위 | 위치 | 의미 |
|---|---|---|---|---|
| `pose` | 3 | m, m, rad | Nav 출력 | 측정 태그 위치 [p₁; p₂] + heading ψ |
| `mode` | 1 | – | ModeChart | 1 출발점 이동, 2 회전, 3 직선 추종, 4 정지, 5 종료 |
| `WP_prev`, `WP_target` | 2, 2 | m | Waypoints | 로봇(태그)이 따라갈 선분의 시작·끝 |
| `lineAB` | 4 | m | Waypoints | 펜이 그릴 직선 [A; B] |
| `t_line` | 1 | s | Waypoints | 모드 3 진입 시각 (기준점 진행 시작) |
| `v_cmd`, `w_cmd` | 1, 1 | m/s, rad/s | RobotCtrl | 로봇 속도 명령 |
| `e_cross` | 1 | m | RobotCtrl M3 | 로봇 기준점 대비 바디 횡오차 |
| `motor_pwm` | 2 | count | PwmMap | [왼쪽; 오른쪽] |
| `stage_target_pos` | 2 | mm | PenCtrl | 스테이지 목표 [X; Y] |
| `pen_on` | 1 | – | PenCtrl | 펜 다운 (0 ≤ σ ≤ L) |
| `stage_pos_meas` | 2 | mm | RosIn | `/stage/pos` 측정값 |
| `q_true`, `V_true` | 3, 2 | m, rad / m/s, rad/s | RobotPlant | 참 자세, 참 속도 (평가용, 제어기는 모름) |
| `x_t` | 4 | mm, mm/s | StagePlant | 펌웨어 궤적 상태 |
| `x_s` | 6 | m, m/s, m·s | StagePlant | 스테이지 위치·속도·적분 상태 |
| `pen_true`, `e_pen` | 2, 1 | m | PenTrace | 참 펜 위치, 펜 횡오차 (평가) |
| `pen_nocomp`, `e_nocomp` | 2, 1 | m | PenTrace | 스테이지를 중앙에 고정했을 때의 펜 위치·횡오차 |

### 3.3 미션 상태기계

![미션 모드](docs/img/diagram_modes.svg)

*그림 3. ModeChart 전이*

| 모드 | 제어 법칙 (6WP_sim · 실기 모델과 같은 식) | 펜 |
|---|---|---|
| 1 Goto_Start | v = K_dist·d·max(0, cos α), ω = K_yaw·α + 0.5 sin α cos α | 올림, 스테이지 중앙 |
| 2 Turning | v = 0, ω = K_yaw·Δθ | 올림 |
| 3 Line_Tracking | 시간 기준점 x_r = WP_prev + v_ref (t − t_line) u, 바디 오차 e = Rᵀ(θ)(x_r − p): v = v_ref cos e_θ + K_px e_x, ω = K_py e_y + K_pθ sin e_θ | 보상, 0 ≤ σ ≤ L 에서 다운 |
| 4 Stop_Destination | 모드 1 식을 끝점에 대해 계속 적용 (d < 0.01 m 에서 ω = 0) | 올림 |

- 로봇이 따라갈 선분 = 펜 직선을 R·d 만큼 평행 이동 + 앞뒤로 `line_pre` 60 mm, `line_post` 40 mm 연장
- 모드 3 에 들어서면 펜 다운 전부터 스테이지를 목표로 보내 시작 과도를 없앰

---

## 4. 수학적 정식화

- 운동모델은 Zou 등 (2018) [1] 의 궤도차량 동역학, 제어 구조는 Zou [1] 와 Sabiha 등 (2022) [2] 의 외부 기구학 루프 + 내부 속도 루프 형태를 따름
- 벽면 흡착, 중력, 원 논문 식의 부호 오류, 6WP_sim · 실기 모델의 제어 법칙에 맞춰 바꾼 부분은 각 절에 명시함
- 식 번호 (1)–(22) 는 이 문서 안에서만 쓰는 번호임

![기하](docs/img/fig_geometry.png)

*그림 4. 벽면 좌표계와 펜 보상 기하 (레버암·스테이지 크기는 읽기 쉽게 확대, 축척 아님)*

![제어 구조](docs/img/fig_control.png)

*그림 5. 제어 구조 (위: 로봇 macro 루프, 아래: 스테이지 micro 루프, 맨 아래: 측정 되먹임)*

### 4.1 좌표계와 기호

벽면 좌표계 $\mathcal{W}$ 는 수직 강판 위의 평면 좌표이고, 바디 좌표계 $\mathcal{B}$ 는 로봇 중심에 붙음.

$$
R(\psi) = \begin{bmatrix} \cos\psi & -\sin\psi \\ \sin\psi & \cos\psi \end{bmatrix}, \qquad
J = \begin{bmatrix} 0 & -1 \\ 1 & 0 \end{bmatrix} \tag{1}
$$

| 기호 | 읽는 법 | 정의 | 단위 |
|---|---|---|---|
| $p_1,\ p_2$ | 피 원, 피 투 | 벽면 좌표. $p_1$ = 높이 (천장 +), $p_2$ = 수평 (벽 정면에서 왼쪽 +) | m |
| $\psi$ | 프사이 | heading. $p_1$ 축에서 $x_B$ 축까지의 각 (천장 방향 0) | rad |
| $R(\psi)$ | 알 프사이 | 바디 → 벽면 회전행렬. $\dot R = \omega J R$ | – |
| $c$ | 씨 | 로봇 중심 위치 | m |
| $p$ | 피 | ZeroKey 태그 위치 $p = c + R(\psi)\,r_{tag}$ | m |
| $q = [c^{\top},\ \psi]^{\top}$ | 큐 | 로봇 자세 | m, rad |
| $V = [v,\ \omega]^{\top}$ | 브이 | 전진 속도, 요 각속도 | m/s, rad/s |
| $A,\ B$ | 에이, 비 | 그릴 직선의 시작점·끝점 | m |
| $u,\ n$ | 유, 엔 | 직선 방향 단위벡터 $u = (B-A)/\lVert B-A\rVert$, 법선 $n = Ju$ | – |
| $d$ | 디 | 태그 → 펜 레버암 (바디) $d = r_{pen,0} - r_{tag}$ | m |
| $s,\ s_c$ | 에스 | 스테이지 위치 [X; Y], 중앙 50.8 mm | mm |
| $\Delta s$ | 델타 에스 | 캐리지 이동량 $(s - s_c)/1000$ | m |
| $P$ | 대문자 피 | 펜 끝 위치 | m |
| $e$ | 이 | 펜 횡오차 $e = n^{\top}(P - A)$ | m |

### 4.2 궤도 로봇 기구학

$$
\dot q = S(q)\,V + w(t), \qquad
S(q) = \begin{bmatrix} \cos\psi & u_c \sin\psi \\ \sin\psi & -u_c\cos\psi \\ 0 & 1 \end{bmatrix} \tag{2}
$$

$$
w(t) = \begin{bmatrix} -k_s\,\lvert v\rvert \\ 0 \\ \dot\psi_w(t) \end{bmatrix}, \qquad
\psi_w(t) = a_w \sin(2\pi f_w t) + \tfrac{1}{2} a_w \sin(2\pi\,2.3 f_w t + 1) \tag{3}
$$

| 기호 | 정의 | 값 |
|---|---|---|
| $u_c$ | 순간 회전 중심 (ICR) 의 길이방향 이동량. Zou [1] 식 (14) $u_c = -\mathrm{sgn}(\omega)\,R_t \sin\alpha$, $R_t = \lvert v/\omega\rvert$ | $\alpha = 0$ 이면 0 |
| $\alpha$ | 미끄럼각 (slip angle) | `alpha_s` = 0 |
| $k_s$ | 자석 궤도 크리프: 이동 속도에 비례한 중력 방향 미끄러짐 | 0.08 [실측 필요] |
| $a_w,\ f_w$ | 궤도 요철에 의한 heading 흔들림 진폭·주파수 | 1.5°, 0.35 Hz [실측 필요] |

- 식 (2) 는 Zou [1] 식 (17)–(19), Sabiha [2] 식 (17)–(18) 의 $S(q)$ 와 같은 형태 (제 3 성분을 $\psi$ 로 씀)
- 식 (3) 은 원 논문에 없는 벽면 외란. 수평 지면과 달리 중력이 진행 평면 안에 있어 궤도가 아래로 조금씩 밀리는 현상을 반영함

### 4.3 동역학 (벽면판)

두 논문의 축약 동역학 (Zou 식 (33b), Sabiha 식 (29)) 에 벽면 흡착과 중력을 넣은 식. 제 2 속도 성분은 차원을 맞추기 위해 $L\omega$ 로 씀.

$$
\tilde M\,\dot\nu + \tilde C\,\nu + S^{\top}(f_1 + f_2) = \tilde B\,\tau + S^{\top} f_g, \qquad
\nu = \begin{bmatrix} v \\ L\omega \end{bmatrix},\quad \tau = \begin{bmatrix} F_r \\ F_l \end{bmatrix} \tag{4}
$$

$$
\tilde M = S^{\top} M S = m\begin{bmatrix} 1 & 0 \\ 0 & 1 + u_c^2/L^2 \end{bmatrix}, \quad
\tilde C = S^{\top} M \dot S, \quad
\tilde B = \begin{bmatrix} 1 & 1 \\ b/2L & -b/2L \end{bmatrix}, \quad
L = \sqrt{(l^2 + b^2)/12} \tag{5}
$$

$$
f_1 = \mu_l N \begin{bmatrix} \cos\psi \\ \sin\psi \\ 0 \end{bmatrix} \tanh\!\frac{v}{\varepsilon_v}, \qquad
f_2 = \begin{bmatrix} F_y \sin\psi \\ -F_y\cos\psi \\ M_r / L \end{bmatrix}, \qquad
f_g = \begin{bmatrix} -m g \\ 0 \\ 0 \end{bmatrix} \tag{6}
$$

$$
F_y = 2\mu_t \frac{N}{l}\,u_c \tanh\!\frac{\omega}{\varepsilon_\omega}, \qquad
M_r = \mu_t \frac{N}{l}\left(\frac{l^2}{4} - u_c^2\right) \tanh\!\frac{\omega}{\varepsilon_\omega} \tag{7}
$$

| 기호 | 정의 | 값 |
|---|---|---|
| $m$ | 로봇 질량 (스테이지 포함) | 12 kg [실측 필요] |
| $l,\ b$ | 궤도 접지 길이, 좌우 궤도 중심 간격 | 0.30 m, 0.28 m |
| $L$ | 회전 반경 (직사각형 판의 $\sqrt{I/m}$) | 0.118 m |
| $N$ | 자석 흡착력 (벽 수직 방향) | 400 N [실측 필요] |
| $\mu_l,\ \mu_t$ | 구름 저항 계수, 횡 미끄럼 마찰 계수 | 0.05, 0.5 [실측 필요] |
| $F_r,\ F_l$ | 오른쪽·왼쪽 궤도 구동력, $\lvert F\rvert \le F_{max}$ | $F_{max}$ = 200 N |
| $\varepsilon_v,\ \varepsilon_\omega$ | $\mathrm{sgn}(\cdot)$ 을 대신하는 $\tanh$ 의 폭 | 0.005 m/s, 0.01 rad/s |

원 논문 식과 다른 점

| 번호 | 원 논문 | 이 모델 | 이유 |
|---|---|---|---|
| 1 | 수직 하중 $W = mg$ (Zou 식 (24)–(27)) | 흡착력 $N$ | 벽에서는 마찰을 만드는 수직력이 중력이 아니라 자석 흡착력 |
| 2 | 중력 항 없음 | $f_g$ (식 (6)) | 진행 평면이 수직이므로 중력이 운동 평면 안에 있음 |
| 3 | $M_r = \mathrm{sgn}(\omega)\,\mu_t \frac{W}{l}\left(u_c^2 - \frac{l^2}{4}\right)$ (Zou 식 (27), Sabiha 식 (24)) | $\frac{l^2}{4} - u_c^2$ | 원식은 $\lvert u_c\rvert < l/2$ 에서 $M_r$ 이 $\omega$ 와 반대 부호가 되어 회전 저항이 회전을 도움. Wong 의 크기 $\mu_t W l / 4$ ($u_c = 0$) 와 부호가 맞도록 수정 |
| 4 | $\upsilon = [v,\ \dot\theta]^{\top}$ (Zou 식 (19), Sabiha 식 (18)) | $\nu = [v,\ L\omega]^{\top}$ | $q_3 = L\theta$ 일 때 $S$ 의 제 3 행 $[0\ \ 1]$ 과 제 1 행 $u_c\sin\psi/L$ 의 차원이 맞으려면 제 2 성분이 $L\dot\theta$ 여야 함 |
| 5 | $M = (F_l + F_r)\,b/2$ (Sabiha 식 (23)) | $(F_l - F_r)\,b/2$ | 오타. 같은 논문 식 (26c), $\bar B$ 는 차이로 씀 |
| 6 | $\mathrm{sgn}(\cdot)$ | $\tanh(\cdot/\varepsilon)$ | 정지 근처 수치 떨림 방지 |

수치 적분 스텝

- $\omega = 0$ 근처에서 회전 저항의 기울기는 $\partial M_r/\partial\omega = \mu_t N l / (4\varepsilon_\omega)$ = 1500 N·m·s, 이를 요 관성 $mL^2$ = 0.168 kg·m² 로 나눈 고유값 $\lambda \approx 8.9\times10^{3}\ \mathrm{s^{-1}}$
- 고정 스텝 RK4 (ode4) 의 안정 조건 $\lvert\lambda h\rvert \lesssim 2.8$ 에 대해 $h$ = 5 ms 는 $\lambda h$ = 44 → $\omega$ 가 수치적으로 진동 (10.5 절에서 확인)
- $h$ = 0.5 ms 에서 $\lambda h$ = 4.5 로 선형 기준은 아직 넘지만 기울기가 $\lvert\omega\rvert < \varepsilon_\omega$ 에서만 크므로 진동이 사라짐. $\int\omega\,dt$ 와 $\Delta\psi$ 의 차이 2.5° 이내 (실측)

### 4.4 구동계와 내부 속도 루프

제어기가 내는 $(v_c, \omega_c)$ 는 PWM 으로 바뀌어 ESC 로 가고, 운동모델 안의 속도 PI 가 궤도 구동력을 만듦.

$$
\bar V = \mathrm{sat}_{0.5}\!\left(\frac{v_c}{2v_{full}}\right), \quad
\bar W = \mathrm{sat}_{0.5}\!\left(\frac{\omega_c}{2\omega_{full}}\right), \quad
\begin{aligned}
u_L &= 1500 + T(\bar V) - T_s(\bar W) \\
u_R &= 1500 - T(\bar V) - T_s(\bar W)
\end{aligned} \tag{8}
$$

$$
T(\bar V) = d_v \tanh\!\frac{\bar V}{0.02} + (400 - d_v)\,\frac{\bar V}{0.5}, \qquad
T_s(\bar W) = d_\omega \tanh\!\frac{\bar W}{0.10} + (400 - d_\omega)\,\frac{\bar W}{0.5} \tag{9}
$$

$$
\tau_X = K_{P,v}\,e_v + K_{I,v}\!\int e_v\,dt, \quad
\tau_N = K_{P,\omega}\,L e_\omega + K_{I,\omega}\!\int L e_\omega\,dt, \quad
\tau = \tilde B^{-1}\begin{bmatrix} \tau_X \\ \tau_N \end{bmatrix} \tag{10}
$$

| 기호 | 정의 | 값 |
|---|---|---|
| $v_{full},\ \omega_{full}$ | PWM 400 count 일 때 속도 | 0.20 m/s, 1.37 rad/s |
| $d_v,\ d_\omega$ | 실기 `calc_pwm` 데드밴드 역보상 | 47, 20 count |
| $u_L,\ u_R$ | `/motor_pwm` [x; y] (오른쪽 모터 반전 포함), 1100–1900 | count |
| $e_v,\ e_\omega$ | ESC 가 정한 궤도 속도 명령과 실제 속도의 차 | m/s, rad/s |
| $K_{P,v},\ K_{I,v},\ K_{P,\omega},\ K_{I,\omega}$ | 속도 PI (6WP_sim PID Controller 두 개) | 1500, 15000, 1500, 15000 |

- ESC 는 데드존 30 count 와 1 차 지연 $\tau_{trk}$ = 0.15 s 로 모델링함
- 식 (10) 은 Zou [1] 식 (42) 의 계산 토크 + PID 중 PI 부분만 남긴 형태. 모델 역 ($\tilde M\dot\nu_c + \tilde C\nu_c + \tilde f$) 을 쓰지 않음
- 실기 제어기 (`Tracked_UGV_ZeroKey_XsensIMU`) 는 이 자리에 PI 두 개를 둠 (속도 $K_P$ 0.15, $K_I$ 0.8, 요 각속도 $K_P$ 1.5, $K_I$ 1.5, 출력 ±0.5). 차이는 12.1 절

### 4.5 외부 루프 (모드별 제어 법칙)

모드 3 (직선 추종). 시간에 따라 움직이는 기준점을 바디 좌표 오차로 따라감.

$$
x_r(t) = W_{prev} + v_r\,(t - t_{line})\,u_r, \qquad
\begin{bmatrix} e_x \\ e_y \end{bmatrix} = R^{\top}(\psi)\,(x_r - p), \qquad
e_\theta = \mathrm{wrap}(\theta_r - \psi) \tag{11a}
$$

$$
v_c = v_r \cos e_\theta + k_x e_x, \qquad
\omega_c = k_y e_y + k_\psi \sin e_\theta \tag{11b}
$$

모드 1 (출발점으로), 모드 4 (끝점에서 정지). 극좌표형 법칙.

$$
\rho = \lVert W - p\rVert, \quad
\alpha = \mathrm{wrap}\!\left(\mathrm{atan2}(W - p) - \psi\right), \quad
v_c = k_\rho\,\rho \max(0, \cos\alpha), \quad
\omega_c = k_\alpha\,\alpha + \tfrac{1}{2}\sin\alpha\cos\alpha \tag{12}
$$

모드 2 (방향 맞추기). $v_c = 0$, $\omega_c = k_\alpha\,\mathrm{wrap}(\theta_r - \psi)$. 모든 모드에서 $\lvert v_c\rvert \le v_{max}$, $\lvert\omega_c\rvert \le \omega_{max}$.

| 기호 | 정의 | 값 (`XY_setup.m` 이름) |
|---|---|---|
| $W_{prev},\ W$ | 로봇 선분 시작점, 목표점 | `WP_prev`, `WP_target` |
| $u_r,\ \theta_r$ | 로봇 선분 방향, 그 각 | – |
| $v_r$ | 직선 구간 속도 | 0.03 m/s (`v_ref`) |
| $k_x,\ k_y,\ k_\psi$ | 진행·횡·heading 게인 | 0.3 s⁻¹, 5.0 rad m⁻¹ s⁻¹, 1.4 s⁻¹ (`K_px`, `K_py`, `K_ptheta`) |
| $k_\rho,\ k_\alpha$ | 거리·방향 게인 | 1.0 s⁻¹, 2.0 s⁻¹ (`K_dist`, `K_yaw`) |
| $v_{max},\ \omega_{max}$ | 명령 한계 | 0.20 m/s, 45 °/s |

- 식 (11b) 는 Kanayama 형 (Zou 식 (38), Sabiha 식 (40)) $\omega_c = \omega_r + v_r\,[k_y' e_y + k_\theta \sin e_\theta]$ 에서 $\omega_r = 0$ (직선) 이고 $v_r$ 을 게인에 흡수한 형태. $k_y = v_r k_y'$, $k_\psi = v_r k_\theta$
- 기준점은 시간 기준이므로 로봇이 늦거나 빠르면 $e_x$ 가 $v_c$ 를 조절함 (10.5 절의 속도 차이 참조)

### 4.6 펜 기구학 보상 (유도)

펜 끝 위치는 태그, 레버암, 캐리지 이동의 합.

$$
P = p + R(\psi)\,(d + \Delta s) = c_0 + R(\psi)\,\Delta s, \qquad c_0 = p + R(\psi)\,d \tag{13}
$$

- $c_0$ 는 스테이지가 중앙에 있을 때의 펜 위치
- 목표: 펜이 직선 위에 있게 하되, 직선 방향 진행은 로봇에게 맡김 → 펜 목표를 $c_0$ 를 직선에 내린 수선의 발로 둠

$$
p^{\star} = A + \sigma\,u, \quad \sigma = u^{\top}(c_0 - A)
\quad\Rightarrow\quad
P = p^{\star}
\ \Leftrightarrow\
\Delta s = R^{\top}(\psi)\,(p^{\star} - p) - d,
\qquad
s^{\star} = s_c + 1000\,\Delta s \tag{14}
$$

보상 없는 횡오차 $e_0 = n^{\top}(c_0 - A)$ 를 쓰면 $p^{\star} - c_0 = -e_0\,n$ 이므로

$$
\Delta s = -e_0\,R^{\top}(\psi)\,n = -e_0 \begin{bmatrix} \sin(\psi - \theta_l) \\ \cos(\psi - \theta_l) \end{bmatrix},
\qquad u = \begin{bmatrix}\cos\theta_l \\ \sin\theta_l\end{bmatrix} \tag{15}
$$

- 스테이지는 $e_0$ 만큼만 움직이고, 그 방향은 로봇과 직선의 각도차 $\psi - \theta_l$ 로 정해짐
- $\lvert\psi - \theta_l\rvert$ 가 작으므로 X 축 (진행 방향) 은 $\lvert e_0\sin(\psi-\theta_l)\rvert$ 로 거의 움직이지 않고 Y 축이 $e_0$ 를 거의 전부 지움. 시뮬레이션 사용 범위 X 51.0–52.5 mm, Y 58.7–70.6 mm 와 일치
- $p^{\star}$ 를 $[A, B]$ 로 자르지 않음. 자르면 펜이 A 앞에 있을 때 $\sigma < 0$ 인 거리를 스테이지가 메우려 해 X 가 행정 끝에 닿음

$$
\mathrm{pen\_on} = \mathbb{1}\left[\,0 \le \sigma \le \lVert B - A\rVert\,\right], \qquad
s^{\star} \leftarrow \mathrm{rate}_{83\,\mathrm{mm/s}}\!\left(\mathrm{sat}_{[0,\,101.6]}(s^{\star})\right) \tag{16}
$$

### 4.7 스테이지 모델

Zaber 펌웨어의 사다리꼴 궤적 생성기 뒤에 축마다 위치 서보가 있는 구조로 모델링함.

$$
m_i\,\ddot x_i + c_s\,\dot x_i = \mathrm{sat}_{F_{max}(\dot x_i)}\!\left(K_{P,i} e_i + K_{D,i}\,\dot e_i + K_{I,i}\!\int e_i\,dt\right) - m_i\,g_{B,i}(\psi), \qquad i \in \{X, Y\} \tag{17}
$$

$$
K_{P,i} = m_i\,\omega_{n,s}^2, \quad K_{D,i} = 2\zeta_s m_i\,\omega_{n,s}, \quad K_{I,i} = K_{P,i}\,\omega_{n,s}/5, \quad
g_B(\psi) = R^{\top}(\psi)\begin{bmatrix} -g \\ 0 \end{bmatrix} \tag{18}
$$

| 기호 | 정의 | 값 |
|---|---|---|
| $m_X,\ m_Y$ | X 축 = 캐리지 + Y 스테이지 + 펜, Y 축 = 캐리지 + 펜 | 0.55 kg, 0.16 kg |
| $e_i$ | 궤적 생성기 출력과 캐리지 위치의 차 | m |
| $F_{max}(\dot x)$ | 데이터시트 추력-속도 곡선 (10 점 보간) | 32 N → 9 N |
| $\omega_{n,s},\ \zeta_s$ | 스테퍼 강성 + 펌웨어 보정을 묶은 대역, 감쇠 | 2π·20 rad/s, 0.8 [실측 필요] |
| $c_s$ | 점성 마찰 | 2.0 N·s/m |

### 4.8 측정 모델

$$
\hat p(t_k) = p(t_k - \tau_{zk}) + \eta_p, \quad \eta_p \sim \mathcal{N}(0, \sigma_p^2 I), \qquad
\hat\psi(t) = \psi(t) + \eta_\psi + \dot b_\psi\,t, \quad \eta_\psi \sim \mathcal{N}(0, \sigma_\psi^2) \tag{19}
$$

| 기호 | 정의 | 값 |
|---|---|---|
| $\sigma_p,\ \tau_{zk}$ | ZeroKey 위치 잡음, 지연 (20 Hz) | 1.0 mm, 50 ms [실측 필요] |
| $\sigma_\psi,\ \dot b_\psi$ | Xsens 요각 잡음, 드리프트 (100 Hz, 강판이라 자력계 없음) | 0.3°, 0.02 °/s |

### 4.9 오차 전파와 오차 예산

보상기는 측정값 $\hat p,\ \hat\psi$ 로 식 (14) 를 풀므로 $\hat p + R(\hat\psi)(d + \Delta s)$ 가 직선 위에 놓임. 실제 펜과의 차를 1 차로 전개하면

$$
P = \underbrace{\hat p + R(\hat\psi)(d + \Delta s)}_{\text{on the line}} - \delta p + \big(R(\psi) - R(\hat\psi)\big)(d + \Delta s),
\qquad R(\psi) - R(\hat\psi) \approx -\delta\psi\,J R(\psi) \tag{20}
$$

$$
e = n^{\top}(P - A) \approx -\,n^{\top}\delta p \;+\; \delta\psi\;\ell_u, \qquad
\ell_u = u^{\top} R(\psi)\,(d + \Delta s) \tag{21}
$$

- $\delta p = \hat p - p$, $\delta\psi = \hat\psi - \psi$. $n^{\top}J = -u^{\top}$ 을 씀
- 위치 오차는 직선 법선 성분만, heading 오차는 레버암의 직선 방향 성분 $\ell_u$ 를 곱해 펜 오차가 됨. 현재 배치에서 $\ell_u \approx d_x$ = 0.08 m

$$
\sigma_e^2 \approx \sigma_p^2 + \ell_u^2\,\sigma_\psi^2 + \ell_u^2\,\frac{(\dot b_\psi T)^2}{3}
= (1.0)^2 + (0.42)^2 + (0.63)^2\ \mathrm{mm}^2
\ \Rightarrow\ \sigma_e \approx 1.25\ \mathrm{mm} \tag{22}
$$

- $T$ = 39.3 s (펜 다운 시간). 드리프트 항은 0 에서 $\dot b_\psi T$ 까지 선형 증가하는 오차의 RMS
- 시뮬레이션 RMS 1.17–1.61 mm 와 같은 크기. 각 항을 하나씩 0 으로 둔 시뮬레이션 결과는 10.5 절 표

### 4.10 선택 기능 (기본 꺼짐)

| 기능 | 위치 | 식 | 파라미터 |
|---|---|---|---|
| ZeroKey 지연 보상 | Nav | $\hat p \leftarrow \hat p + \tau_{pred}\,\hat{\dot p}$ | `tau_pred` (기본 0) |
| 스테이지 중앙 유지 | RobotCtrl M3 | $e_y \leftarrow e_y + k_c e_s + k_{ci}\int e_s\,dt$, $e_s = (s_{Y,meas} - s_c)/1000$ | `K_c`, `K_ci` (기본 0) |

- 효과는 11 장 비교 참조

### 4.11 참고문헌

1. T. Zou, J. Angeles, F. Hassani, "Dynamic modeling and trajectory tracking control of unmanned tracked vehicles," *Robotics and Autonomous Systems*, vol. 110, pp. 102–111, 2018. doi:10.1016/j.robot.2018.09.008
2. A. D. Sabiha, M. A. Kamel, E. Said, W. M. Hussein, "ROS-based trajectory tracking control for autonomous tracked vehicle using optimized backstepping and sliding mode control," *Robotics and Autonomous Systems*, vol. 152, 104058, 2022. doi:10.1016/j.robot.2022.104058
3. J. Y. Wong, *Theory of Ground Vehicles*, 4th ed., Wiley, 2008 (회전 저항 $\mu_t W l/4$)

---

## 5. 운동모델

### 5.1 로봇 (6WP_sim 운동모델의 벽면판)

| 단계 | 블록 | 식·값 |
|---|---|---|
| ESC 역변환 | EscInv | PWM → 궤도 속도, 데드존 `pwm_dz` = 30 count, 400 count = 0.2 m/s |
| 모터 지연 | TrackLag | 1차 지연 τ_trk = 0.15 s |
| 속도 루프 | PID_v, PID_w | 6WP PID Controller 두 개 (Kp 1500, Ki 15000) |
| 구동력 배분 | Control_allocation | B̃ = [1 1; b/2L −b/2L], \|F\| ≤ F_max = 200 N |
| 동역학 | Dynamics | 6WP 식의 벽면판 (4.3 절) |
| 기구학 | Kinematics | q̇ = S(q)V + 벽면 외란 (중력 방향 크리프 8 %, heading 요철 ±1.5°, 0.35 Hz) |
| 적분 | ode4 고정 스텝 | `Ts_plant` = 0.5 ms (5 ms 에서는 회전 마찰 tanh 때문에 ω 가 수치 진동) |

- 식과 원 논문 대비 변경점: 4.2–4.4 절
- 6WP_sim Dynamics 는 Zou [1] 식을 그대로 구현한 것이므로 4.3 절의 부호 수정 (M_r) 이 6WP 원본에도 해당함

### 5.2 Zaber 스테이지 (데이터시트 + CAD)

| 스펙 (X-LSM100B-E03) | 모델 반영 | 값 |
|---|---|---|
| 행정 | PenCtrl `Travel`, StagePlant `Encoder` 포화 | 0–101.6 mm |
| 최고속도 | ZaberTraj 사다리꼴 궤적 | 104 mm/s |
| 가감속 | ZaberTraj | 500 mm/s² [실측 필요: `get accel`] |
| 추력-속도 곡선 | ZaberServo 추력 한계 (10 점 보간) | 32 N (정지) → 9 N (104 mm/s) |
| 이동 질량 | X 축 = 캐리지 + Y 스테이지 + 펜, Y 축 = 캐리지 + 펜 | 0.55 kg, 0.16 kg |
| 벽면 중력 | 축 방향 성분 m g | X 5.4 N, Y 1.6 N (연속 추력 25 N, 역구동력 24 N 이하) |
| 위치 루프 | 스테퍼 강성 + 펌웨어 보정 | 대역 20 Hz, 감쇠 0.8 [실측 필요] |
| 명령 주기·지연 | CmdHold, CmdDelay | 25 Hz (노드 `cmd_rate`), 10 ms |
| 위치 보고 | `/stage/pos` | 20 Hz |
| 정확도 · 반복정밀도 | 반영하지 않음 | 45 µm · 6 µm (센서 잡음 1 mm 대비 무시) |

### 5.3 센서

| 센서 | 잡음 (1σ) | 지연 | 주기 |
|---|---|---|---|
| ZeroKey 위치 | 1.0 mm | 50 ms [실측 필요] | 20 Hz |
| IMU 요각 | 0.3°, 드리프트 0.02°/s | – | 100 Hz |
| ZeroKey INS 자세 | 0.9° | – | 20 Hz |

---

## 6. Simulink 모델 구성

![XY_1_pen_sim 최상위](1_시뮬레이션/img/XY_1_pen_sim.png)

*그림 6. XY_1_pen_sim 최상위 (윗줄 제어기, 오른쪽 아래 가상 실험장, 왼쪽 아래 평가·기록·실시간 화면)*

| 모델 | 구성 | 용도 |
|---|---|---|
| `XY_1_pen_sim.slx` | 제어기 (RosIn, Nav, Mission, RobotCtrl, PwmMap, PenCtrl, RosOut) + 가상 실험장 (PlantRosIn, RobotPlant, StagePlant, Sensors, PlantRosOut) + PenTrace, Logging, Animate | 가상 실험 (도메인 77) |
| `XY_2_pen_ros2.slx` | 같은 빌더 함수로 만든 제어기 윗줄만 | 실기 (도메인 0) |

- 모든 계산 코드는 모델 안의 MATLAB Function 블록에 있음 (외부 .m 함수 없음)
- 왼쪽 → 오른쪽 배치: 항법 (RosIn, Nav) → 미션 → 제어 → 배분 → ROS 경계·플랜트 → 측정
- 상자 사이 신호는 선으로 바로 연결 (선에 신호 이름). 상자는 앞 상자의 포트 높이에 맞춰 선이 수평 직선
- 서브시스템 안도 선으로 연결 (Simulink 층 배치). Goto/From 은 두 경우에만 사용
  - 기록·평가용 탭 (Logging, PenTrace, Animate 가 꺼내 쓰는 신호)
  - RobotCtrl 안에서 모드 상자 네 개가 같은 입력을 나눠 쓰는 곳
- 제어기와 가상 실험장은 선이 아니라 ROS 2 토픽으로만 연결 (RosOut → PlantRosIn, PlantRosOut → RosIn)
- 블록에는 숫자 대신 파라미터 이름 (`XY_setup.m` 하나만 고치면 됨)
- 제어기는 연속, ode4 고정 스텝 0.0005 s (`Ts_plant`). 이산 블록 (센서 ZOH, Nav persistent, 명령 ZOH) 은 Description 에 이유 기재
- 기록은 Bus Creator → ZOH (0.01 s) → To Workspace `log` 하나

| 서브시스템 그림 | 파일 |
|---|---|
| 미션 | `1_시뮬레이션/img/XY_1_pen_sim__Mission.png` |
| 로봇 제어 | `1_시뮬레이션/img/XY_1_pen_sim__RobotCtrl.png`, `__RobotCtrl__M3_Line.png` |
| 펜 보상 | `1_시뮬레이션/img/XY_1_pen_sim__PenCtrl.png` |
| 로봇 운동모델 | `1_시뮬레이션/img/XY_1_pen_sim__RobotPlant.png` |
| 스테이지 모델 | `1_시뮬레이션/img/XY_1_pen_sim__StagePlant.png` |
| 센서 | `1_시뮬레이션/img/XY_1_pen_sim__Sensors.png` |

![StagePlant](1_시뮬레이션/img/XY_1_pen_sim__StagePlant.png)

*그림 7. StagePlant 내부 (명령 ZOH → 지연 → 펌웨어 궤적 → 서보 → 엔코더)*

---

## 7. 설치

### 7.1 요구 사항

| 항목 | 버전 |
|---|---|
| MATLAB / Simulink | R2025a 이상 |
| 툴박스 | ROS Toolbox, Stateflow |
| 실기 NUC | Ubuntu 24.04, ROS 2 Jazzy, Python 3, `zaber-motion` |
| 애니메이션 (선택) | `4_CAD/AVT-3C.stl` (저장소에 포함하지 않음) |

### 7.2 절차

| 단계 | 명령 | 결과 |
|---|---|---|
| 1 | `git clone https://github.com/wkyouncnu/AVT3C-XY-Stage-Pen.git` | 저장소 |
| 2 | MATLAB 에서 `1_시뮬레이션` 으로 이동, `XY_setup` | 경로·파라미터, `ROS_DOMAIN_ID` 77 |
| 3 | `build_xy_models` | 두 .slx 생성, 검사 결과 출력 |
| 4 (NUC) | `pip install zaber-motion` | 스테이지 노드 의존성 |

---

## 8. 실행

| 목적 | 명령 | 소요 |
|---|---|---|
| Simulink 에서 바로 | `XY_setup` 후 `XY_1_pen_sim.slx` 를 열고 Run 버튼 | 시뮬레이션 48 s 를 약 15–17 s 에. 실행 중 대시보드, 끝나면 결과 그림 두 장 |
| 전체 (설정 → 빌드 → 실행 → 그림 → 동영상) | `XY_run_all` | 약 4 분 |
| 가상 실험 한 번 | `XY_1_pen_sim_run` | 약 20 s + 동영상 약 100 s (`make_video = 0` 이면 생략) |
| 결과 그림만 | `M = XY_plot(out)` | – |
| 대시보드 동영상 | `XY_record_live(out)` 또는 `XY_record_live` | 약 100 s → `video/XY_1_live_view.mp4` |
| 설계 선택 비교 | `XY_compare; XY_compare_plot` | 10 경우, 약 3–4 분 |
| 실기 | 12 장 | – |

- 가상 실험은 `PacingRate` 4 (실시간의 최대 4 배) 로 진행. ROS 2 루프백 토픽의 발행·구독 순서는 유지되며, 실행마다 수치가 조금 다름 (10.5 절)
- 실기 모델은 실시간 1×
- 미션이 끝나면 (모드 5) 자동 정지

![대시보드](1_시뮬레이션/img/XY_1_dashboard.png)

*그림 8. 실시간 대시보드 (왼쪽 3D 벽면·펜 확대, 가운데 v·ω 명령 대비 실제와 펜 횡오차, 오른쪽 스테이지 캐리지와 현재 게인)*

Run 버튼으로 실행할 때의 동작

| 항목 | 내용 |
|---|---|
| 대시보드 | Animate 상자 → `XY_animate.m`. 입력마다 Zero-Order Hold (`anim_Ts` = 0.2 s) 를 두어 함수가 0.2 s 마다만 호출됨 |
| 대시보드 내용 | 3D 벽면 (STL 로봇, 중력 방향, 목표 직선, 펜 자국), 펜 확대, $v_c$ 대 $v$ [m/s], $\omega_c$ 대 $\omega$ [deg/s], 펜 횡오차 [mm] (현재·RMS·최대), 스테이지 캐리지 위치 [mm], 현재 게인 (LaTeX 표기, 4.4–4.7 절 기호) |
| 끄기 | `animate = 0` 후 다시 Run. 게인 탐색처럼 여러 번 돌릴 때 권장 (속도 차이는 작음) |
| 종료 후 | StopFcn `XY_stop.m` 이 `log` 를 `out.log` 로 옮기고 `XY_plot(out)` 실행 → 벽면 궤적, 시간 응답 그림, 명령창에 RMS |
| 동영상 | `XY_record_live(out)`: 기록을 0.1 s 간격으로 `XY_animate` 에 다시 넣어 그림 창 하나를 화면 밖에서 렌더링 (`print -RGBImage`), 20 fps (2 배속). 화면 캡처가 아니므로 다른 창이 찍히지 않음 |
| 스크립트 실행 | `XY_batch = 1`, `animate = 0` 으로 두 기능을 끄고 `sim(..., 'ReturnWorkspaceOutputs','on')` 반환값 사용 |

---

## 9. 프로그램별 입출력

### 9.1 파라미터 (`XY_setup.m`)

| 절 | 이름 | 값 | 비고 |
|---|---|---|---|
| 1 | `Ts_ctrl`, `Ts_plant` | 0.01, 0.0005 s | 제어 주기, 적분 스텝 (4.3 절) |
| 2 | `pen_A`, `pen_B` | (0.30, 0.70), (0.30, −0.50) m | 그릴 직선 (1.2 m) |
| 3 | `q0` | (0.10, 0.85, 0) | 출발 자세 |
| 4 | `K_dist`, `K_yaw` | 1.0, 2.0 | 모드 1, 2 |
| 4 | `v_ref`, `K_px`, `K_py`, `K_ptheta` | 0.03 m/s, 0.3, 5.0, 1.4 | 모드 3 (실기 ZeroKey 모델 값, 속도만 낮춤) |
| 4 | `K_c`, `K_ci` | 0, 0 | 스테이지 중앙 유지 (선택) |
| 5 | `r_tag`, `r_pen0` | (−0.08, 0.06), (0, 0) m | [실측 필요] |
| 6 | `m_r`, `N_mag`, `mu_l`, `mu_t` | 12 kg, 400 N, 0.05, 0.5 | [실측 필요] |
| 7 | 스테이지 | 5.2 절 | |
| 8 | `zk_*`, `imu_*`, `tau_pred` | 5.3 절, 0 | |
| 9 | `animate`, `anim_Ts`, `XY_batch` | 1, 0.2 s, 0 | 대시보드 켜기, 갱신 주기, 스크립트 실행 표시 |

### 9.2 변경 항목

| 변경 | 수정 파일 | 재빌드 |
|---|---|---|
| 게인, 직선, 질량, 센서 잡음 | `XY_setup.m` | 불필요 (다시 `XY_setup`) |
| 블록 구성, 함수 식 | `build_xy_models.m` | `build_xy_models` |
| 스테이지 노드 주기 | `zaber_stage_node.py` 파라미터 `cmd_rate` + `XY_setup` `stage_cmd_Ts` | 모델 재빌드 불필요 |

### 9.3 스테이지 노드 (`zaber_stage_node.py`)

| 파라미터 | 기본값 | 의미 |
|---|---|---|
| `port` | `/dev/ttyUSB0` | RS-232 포트 (115200) |
| `x_device`, `y_device` | 1, 2 | 데이지 체인 주소 |
| `publish_rate` | 20 Hz | `/stage/pos` |
| `cmd_rate` | 25 Hz | `move abs` 전송 주기 (마지막 목표만) |
| `cmd_deadband` | 0.001 mm | 변화가 이보다 작으면 재전송하지 않음 |
| `cmd_timeout` | 0.5 s | `/stage/cmd_vel` 워치독 |
| `home_on_start` | True | 기동 시 원점 복귀 |

- 제어기는 100 Hz 로 목표를 보내지만 시리얼 한 줄(요청+응답)이 수 ms 걸리므로 노드가 25 Hz 로 솎음. 시뮬레이션 `stage_cmd_Ts` = 0.04 s 와 같음

---

## 10. 검증

### 10.1 검증 항목

| 구분 | 실행 | 대상 | 기준 |
|---|---|---|---|
| 모델 구조 | `build_xy_models` | 두 모델 | `check_flow` 0, `check_tags` 0, `check_lines` 겹침·관통·사선·블록겹침 0 |
| ROS 루프백 | `XY_1_pen_sim_run` | 토픽 발행·구독 | 미션 모드 1 → 5 완주 |
| 실기 모델 | `update diagram` (도메인 0) | `XY_2_pen_ros2` | 오류 없음 |
| 펜 정확도 | `XY_plot` | 펜 다운 구간 | 횡오차 RMS ≤ 2 mm, 스테이지 포화 없음 |

### 10.2 모델 구조 결과

| 모델 | check_flow | check_tags | 겹침·관통·사선·블록겹침 | 3회 이상 꺾임 | Goto / From |
|---|---|---|---|---|---|
| XY_1_pen_sim | 0 | 0 | 0 | 10 | 26 / 49 |
| XY_2_pen_ros2 | 0 | 0 | 0 | 2 | 19 / 29 |

- 3회 이상 꺾인 선은 운동모델 안의 적분기 되먹임 (V, q, x_t, x_s 를 앞 블록으로 되돌리는 선) 과 최상위의 긴 연결 (pose, q_true, stage_target_pos)
- From 이 44 → 49 로 는 것은 Animate 상자가 꺼내는 신호 (v_cmd, w_cmd, V_true, mode, e_pen) 가 늘었기 때문
- RobotPlant 안에서 compute_u → Dynamics 선이 dist_p 상수를 스치던 관통 1 건은 빌더에서 상수를 옮겨 없앰
- 이전 판 (Goto/From 연결, 78 / 123) 은 이 항목이 0 이었으나 신호 흐름이 선으로 보이지 않아 선 연결로 바꿈. 되먹임 선을 다시 Goto/From 으로 숨기지 않음

### 10.3 가상 실험 결과 (기준 조건)

| 항목 | 값 |
|---|---|
| 직선 | A (0.30, 0.70) → B (0.30, −0.50) m, 1.2 m, 30 mm/s |
| 모드 전이 | 2 → 2.2 s, 3 → 4.6 s, 4 → 46.0 s, 종료 47.8–48.0 s |
| 펜 다운 시간 | 39.3 s |
| 펜 횡오차 (스테이지 보상) | RMS 1.17–1.18 mm, 최대 3.5–4.0 mm |
| 펜 횡오차 (보상 없음, 같은 주행) | RMS 14.45 mm, 최대 19.3 mm |
| 로봇 기준점 횡오차 `e_cross` | RMS 22.9 mm |
| 스테이지 사용 범위 | X 51.0–52.5 mm, Y 58.8–70.6 mm (행정 0–101.6) |

- 두 축 모두 움직임. X (진행 방향) 는 1.5 mm 폭으로 거의 고정, Y (직선 법선) 가 횡오차를 지움 (식 (15))
- 실행마다 수치 차이 있음 (10.5 절)

![벽면 결과](1_시뮬레이션/img/XY_1_result_wall.png)

*그림 9. 벽면 궤적 (파랑 목표 직선, 빨강 보상 펜 자국, 회색 보상 없는 펜 자국, 점선 로봇 태그 경로)*

![시간 응답](1_시뮬레이션/img/XY_1_result_time.png)

*그림 10. 펜 횡오차, 스테이지 위치, 모드, 펜 상태*

### 10.4 동영상

| 파일 | 내용 |
|---|---|
| `1_시뮬레이션/video/XY_1_live_view.mp4` | 그림 8 대시보드, 1600 × 900, 20 fps, 23.9 s (시뮬레이션 47.8 s 를 2 배속) |

- `XY_record_live` 로 생성. 기록 재생 + 화면 밖 렌더링이므로 녹화 중 다른 창·메신저가 찍히지 않음

### 10.5 내부 루프와 오차 요인 진단

대시보드의 명령 대비 실제 값 (그림 8 가운데) 으로 내부 루프와 게인을 점검한 결과.

요 각속도 떨림

| 항목 | 적분 스텝 5 ms | 0.5 ms (현재) |
|---|---|---|
| ω 추종 오차 RMS | 6.3 deg/s | 1.9–2.0 deg/s |
| $\int\omega\,dt$ 와 $\Delta\psi$ 일치 | 맞지 않음 (수치 진동) | 2.5° 이내 |
| 원인 | 회전 저항 $\tanh(\omega/\varepsilon_\omega)$ 의 기울기에 비해 스텝이 큼 ($\lambda h \approx 44$, 4.3 절) | – |

- 떨림은 게인 문제가 아니라 수치 적분 문제였음. 스텝을 줄인 뒤 실행 시간은 대시보드 갱신 주기 조정 (`anim_Ts`) 으로 오히려 줄어듦 (113 s → 15–17 s)

게인 변경 (경우마다 3 회 실행 평균, 기준은 평균 ± 표준편차, `logs/gain_sweep.mat`)

| 경우 | 펜 RMS [mm] | 로봇 RMS [mm] |
|---|---|---|
| 기준 | 1.44 ± 0.24 | 24.5 |
| $k_y$ × 0.5 | 1.61 | 41.1 |
| $k_y$ × 2 | 1.55 | 13.8 |
| $k_\psi$ × 0.5 | 1.56 | 14.2 |
| $k_\psi$ × 2 | 1.65 | 40.8 |
| 내부 PI 모두 × 0.5 | 1.45 | 24.2 |
| 내부 PI 모두 × 2 | 1.56 | 25.2 |
| ZeroKey 잡음 0 (비교용) | 1.32 | 25.0 |

- 외부·내부 게인을 바꾸면 로봇 횡오차는 14–41 mm 로 크게 변하지만 펜 횡오차는 반복 편차 (± 0.24 mm) 안에서만 변함
- 로봇 오차는 스테이지가 지우므로 게인 튜닝으로 펜 오차를 줄일 수 없음. 게인은 스테이지 행정 여유 (로봇 오차 < 50 mm) 만 지키면 됨
- 게인 비교 표 (외부 루프)

| 출처 | 진행 | 횡 | heading | 비고 |
|---|---|---|---|---|
| 이 모델 (실기 ZeroKey 모델 값) | $k_x$ 0.3 | $k_y$ 5.0 | $k_\psi$ 1.4 | $v_r$ 을 게인에 흡수, 식 (11b) |
| Zou [1] 식 (38) | $k_x$ 12 | $k_y$ 6.3 | $k_\theta$ 3.3 | 지상 차량, $v_r$ 곱함, 내부 PID 36 / 30 / 16 |
| Sabiha [2] (최적화 backstepping + SMC) | 5.2 | 3.7 | 2.1 | $K_1$ 5.4, $K_2$ 7.3, $\rho$ 9.5 |

- 논문 값은 지상 주행·다른 질량에서 최적화된 값이므로 그대로 옮기지 않음

오차 예산 (각 요인만 0 으로 둔 시뮬레이션, 펜 RMS)

| 경우 | 펜 RMS [mm] | 식 (22) 의 항 |
|---|---|---|
| 기준 | 1.57 | 전체 1.25 |
| IMU 잡음 0 | 1.30 | $\ell_u\sigma_\psi$ = 0.42 제거 |
| IMU 드리프트 0 | 1.24 | $\ell_u \dot b_\psi T/\sqrt3$ = 0.63 제거 |
| heading 요철 0 | 1.54 | – |
| 크리프 0 | 1.46 | – |
| 센서 모두 이상적 | 0.39 | 남는 것은 스테이지 추종 지연 |
| 센서 이상적 + 요철 0 | 0.04 | – |

- 펜 정확도의 상한은 센서 (ZeroKey 위치 1 mm, Xsens heading × 레버암) 이며 1 차 해석 식 (22) 과 시뮬레이션이 같은 크기
- 정확도를 높이려면 게인이 아니라 레버암 $\ell_u$ 를 줄이거나 (태그를 펜 가까이), heading 드리프트를 보정해야 함

속도 루프 정상 편차

- 펜 다운 구간 평균: 명령 $v_c$ 0.0226 m/s, 실제 $v$ 0.0306 m/s
- 원인: 제어기의 PWM 역보상 데드밴드 $d_v$ = 47 count 와 운동모델 ESC 데드존 30 count 의 차이. 개루프 PWM 사상이 과하게 밀고, 외부 루프 $k_x e_x$ 가 명령을 줄여 시간 기준점에 맞춤
- 모드 4 끝에서 ω 명령이 45 deg/s 까지 오르는 것은 끝점을 향해 도는 모드 4 법칙 (식 (12)) 의 정상 동작

실행마다 다른 수치

- 시드 (21, [11 12 13], 31) 를 고정해도 ROS 2 루프백 수신 시점이 실행마다 달라 펜 RMS 1.17–1.61 mm 범위로 변함
- 설계 비교는 같은 조건을 반복해 평균으로 판단해야 함

---

## 11. 설계 선택 비교

`XY_compare.m` 으로 기준 조건 (1.2 m 직선, 30 mm/s) 에서 한두 값만 바꿔 10 경우를 같은 ROS 2 루프백 모델로 실행함. 경우마다 1 회 실행이므로 0.3 mm 이하 차이는 실행 간 편차 (10.5 절) 안에 있음.

![비교 막대](1_시뮬레이션/img/XY_compare_bar.png)

*그림 11. 경우별 펜 횡오차 (왼쪽 스테이지 보상, 오른쪽 같은 주행에서 스테이지를 중앙에 고정했을 때)*

### 11.1 결과

| 경우 | 바꾼 값 | 펜 RMS [mm] | 펜 최대 [mm] | 로봇만 RMS [mm] | 스테이지 Y [mm] |
|---|---|---|---|---|---|
| 기준 | – | 1.17 | 3.61 | 14.45 | 58.7–70.6 |
| 속도 15 mm/s | `v_ref` 0.015 (펜 다운 79.4 s) | 1.60 | 6.01 | 16.39 | 58.1–73.1 |
| 속도 60 mm/s | `v_ref` 0.060 (펜 다운 19.6 s) | 1.20 | 3.46 | 15.20 | 57.8–72.9 |
| 60 mm/s + 지연 보상 | `v_ref` 0.060, `tau_pred` 0.05 s | 1.17 | 3.32 | 15.58 | 57.8–73.2 |
| 지연 보상 | `tau_pred` 0.05 s | 1.22 | 3.79 | 14.78 | 59.0–71.1 |
| 스테이지 느슨 | 대역 2 Hz, 명령 5 Hz | 1.73 | 5.07 | 14.45 | 58.1–70.2 |
| 중앙 유지 | `K_c` 0.5, `K_ci` 0.3 | 1.23 | 3.79 | 3.74 | 43.9–64.9 |
| 지연 보상 + 중앙 유지 | 세 값 | 1.23 | 3.68 | 3.80 | 43.8–63.4 |
| ZeroKey 잡음 0.5 mm | `zk_sigma` 0.0005 | 0.88 | 3.28 | 14.45 | 59.2–70.0 |
| 센서 잡음 0 | ZeroKey·IMU 잡음·드리프트 0 | 0.35 | 0.89 | 13.83 | 59.9–69.3 |

![비교 시계열](1_시뮬레이션/img/XY_compare_time.png)

*그림 12. 위: 펜 횡오차 (스테이지 느슨 · 기준 · 센서 잡음 0), 아래: 스테이지 Y (기준 · 중앙 유지)*

### 11.2 해석

| 항목 | 결과 | 근거 |
|---|---|---|
| 주행 속도 | 30–60 mm/s 에서 1.2 mm, 15 mm/s 에서 1.6 mm | 느리면 펜 다운 시간이 길어 heading 드리프트가 쌓임 (식 (22) 의 $\dot b_\psi T$ 항) |
| 지연 보상 | 차이 없음 (1.17 → 1.22 mm, 실행 편차 안) | 직선 위에서 ZeroKey 지연은 진행 방향 오차가 되고, 펜 보상은 직선 법선 성분만 씀 (식 (21) 의 $n^{\top}\delta p$) |
| 스테이지 서보를 느슨하게 | 1.17 → 1.73 mm (+48 %) | 대역 2 Hz 는 heading 요철 (0.35, 0.8 Hz) 을 다 따라가지 못함. 수 Hz 이상 필요 |
| 오차를 정하는 요인 | 센서 | 잡음 절반 → 0.88 mm, 잡음 0 → 0.35 mm |
| 로봇만으로 | 13.8–16.4 mm | 궤도 로봇은 횡방향으로 직접 움직이지 못하고 (회전으로만 고침), 벽면 크리프·요철을 다 지우지 못함 |
| 중앙 유지 루프 | 펜 RMS 그대로, 로봇 오차 14.5 → 3.7 mm, 스테이지 Y 범위 중앙 (50.8 mm) 쪽으로 이동 | 스테이지 편차의 적분이 벽면 크리프로 생긴 정상 편차를 로봇 쪽에서 없앰 → 행정 여유 확보 |

### 11.3 정지 후 긋기와 주행 중 긋기

| 구분 | 정지 후 긋기 (구간 약 90 mm 씩 이어 붙임) | 주행 중 긋기 (현재 방식) |
|---|---|---|
| 구간 안 직선도 | 정지 중 크리프·요철 없음, ZeroKey 2 s 평균 시 잡음 1/√40 → 약 0.2 mm (계산값) | RMS 약 1.2 mm (시뮬레이션) |
| 구간 이음매 | 이음매마다 계단. 크기 ≈ √2 × ZeroKey 절대 오차 (편향은 평균으로 줄지 않음) | 없음 |
| 1.2 m 소요 | 정지·출발 14 회 이상 | 연속 39 s |
| 적합한 경우 | 짧은 선 (100 mm 이하), 이음매 허용, 0.5 mm 이하 요구 | 연속 직선, mm 급 요구 |

- 정지 후 긋기는 시뮬레이션으로 확인하지 않음 (계산값). ZeroKey 정지 절대 오차의 실측 필요

### 11.4 결론

- 이중 구동 (macro–micro) 구조 유지: 로봇만 13.8–16.4 mm, 스테이지 보상 1.2 mm
- 스테이지 서보 자체는 Zaber 펌웨어에 맡기고 목표 위치만 25 Hz 로 보내면 충분함. 단, 2 Hz 수준으로 느리면 오차 증가
- 정확도의 상한은 보상 계산에 들어가는 측정값: ZeroKey 잡음, IMU heading 과 레버암, 스테이지 축 장착 각도. 제어 게인은 상한을 바꾸지 못함 (10.5 절)
- 행정 여유를 위해 중앙 유지 루프 (`K_ci` 0.3) 를 실기에서 켜는 것을 권장함

---

## 12. 실장비 적용

| 단계 | 내용 |
|---|---|
| 1 | NUC 에서 ZeroKey, Xsens, 로봇 브리지 노드 기동 (`ROS_DOMAIN_ID=0`) |
| 2 | NUC 에서 `python3 zaber_stage_node.py --ros-args -p port:=/dev/ttyUSB0` (원점 복귀 후 `/stage/pos` 발행 확인) |
| 3 | MATLAB: `XY_setup; setenv('ROS_DOMAIN_ID', num2str(ros_domain_real))` |
| 4 | `XY_2_pen_ros2.slx` 열고 실행 (`T_run_ros` = 300 s) |
| 5 | 펜 없이 한 번 주행해 `/stage/target_pos` 범위가 10–90 mm 안인지 확인 후 펜 장착 |

- 가상 실험 (도메인 77) 과 실기 (도메인 0) 가 같은 망에 있어도 서로의 `/motor_pwm` 을 받지 않음
- 실기 전 확인할 부호: 스테이지 축 1 이 x_B (전진), 축 2 가 y_B (좌측) 방향으로 증가하는지

### 12.1 시뮬레이션 제어기의 실기 사용 범위

`XY_2_pen_ros2` 는 가상 실험의 제어기 윗줄을 같은 빌더 함수로 만든 것이므로 토픽·식·게인이 같음.

| 항목 | 가상 실험 (XY_1) | 실기 (XY_2) | 상태 |
|---|---|---|---|
| 위치 | `/zerokey/position` (운동모델 + 잡음 발행) | ZeroKey 노드 | 같음 |
| 속도 | `/zerokey/vel` | ZeroKey 노드 | 같음 (지연 보상 켤 때만 사용) |
| heading | `/imu/data` orientation → 상대 요각 | Xsens 노드 | 같음 |
| 스테이지 위치 | `/stage/pos` (StagePlant 발행) | `zaber_stage_node.py` | 같음 (중앙 유지 켤 때만 사용) |
| 로봇 명령 | `/motor_pwm` | Arduino 브리지 | 같음 |
| 스테이지 명령 | `/stage/target_pos` | `zaber_stage_node.py` → RS-232 `move abs` | 같음 |
| 내부 속도 루프 | 운동모델 안 PI (ESC 대역 가정) | 없음 | 차이 |

- 실기 ZeroKey + Xsens 모델 (`Tracked_UGV_ZeroKey_XsensIMU`) 은 `calc_pwm` 앞에 PI 두 개를 둠
  - 속도: $K_P$ 0.15, $K_I$ 0.8, 출력 [0, 0.5], 되먹임 = ZeroKey 바디 속도
  - 요 각속도: $K_P$ 1.5, $K_I$ 1.5, 출력 ±0.5, 되먹임 = IMU 자이로 z
- XY 모델은 이 PI 없이 $(v_c, \omega_c)$ 를 개루프 PWM 사상 (식 (8)–(9)) 으로 보냄. 가상 실험에서는 외부 루프가 속도 편차 (10.5 절) 를 메워 펜 정확도에 영향이 없었으나, 실기의 ESC 데드존·벽면 하중 차이가 크면 같은 PI 를 넣는 것이 안전함
- 실기 모델이 쓰는 `/zerokey/orientation` 은 XY 제어기에서 사용하지 않음 (heading 은 Xsens)

---

## 13. 문제 해결

| 증상 | 원인 | 조치 |
|---|---|---|
| 모드 3 시작 때 스테이지 X 가 행정 끝 (95 mm) 근처까지 감 | p* 를 A 로 잘라 남은 진행 거리를 스테이지가 메움 | p* 를 자르지 않는 수선의 발로 변경 |
| heading 2 Hz 떨림, 횡오차 3.6 mm | 6WP 회전 저항 부호 (u² − l²/4) | (l²/4 − u²) 로 수정 |
| Merge 샘플 시간 불일치 오류 | 모드 3 의 연속 Clock 과 다른 모드의 0.01 s | 모드 3 에 Digital Clock (Ts_ctrl) |
| To Workspace 샘플 시간 경고 | 서로 다른 주기 신호를 한 Bus 로 기록 | Bus 뒤 ZOH (0.01 s) |
| 실기 스테이지 명령이 밀림 | 100 Hz `move abs` 가 시리얼 115200 bps 를 포화 | 노드에서 마지막 목표만 25 Hz 로 전송 |
| 시뮬레이션 `/motor_pwm` 이 실제 로봇을 움직일 위험 | 같은 도메인 | 가상 실험 도메인 77 분리 |
| VideoWriter 권한 오류 | 이전 VideoReader 가 파일을 잡고 있음 | `clear` 후 재실행 |
| 실행마다 수치가 다름 | ROS 2 수신 시점 | 정상 (펜 RMS 1.17–1.61 mm 범위). 비교는 반복 평균으로 |
| 대시보드 실행이 매우 느림 (48 s 에 113 s) | Animate 의 MATLAB Function 입력이 연속이라 0.5 ms 스텝마다 그림 함수 호출 | 입력마다 Zero-Order Hold (`anim_Ts` 0.2 s) |
| ω 가 크게 떨림 (추종 오차 6 deg/s) | 회전 저항 tanh 에 비해 적분 스텝 5 ms 가 큼 | `Ts_plant` 0.5 ms (4.3 절) |
| `XY_compare` 중 변수 오류 | base 작업공간에 남은 `r`, `s` 가 스크립트 변수와 충돌 | `XY_setup` 뒤 `clear r s` |
| 녹화에 다른 창이 찍힘 | 화면 캡처 방식 | 화면 밖 렌더링 (`print -RGBImage`) 으로 변경. 화면 캡처로 되돌리지 않음 |

---

## 14. 확인 필요 사항

| 항목 | 현재 설정 | 확인 대상 |
|---|---|---|
| 태그·펜 위치 `r_tag`, `r_pen0` | (−0.08, 0.06), (0, 0) m | 레이저 트래커 실측 |
| 스테이지 축 부호·장착 각도 | 축 1 = x_B, 축 2 = y_B, 0° | 실기 조립 확인 |
| 스테이지 가감속 | 500 mm/s² | `get accel` |
| 스테이지 위치 루프 대역 · 명령 지연 | 20 Hz · 10 ms | 계단 명령 응답 측정 |
| 로봇 질량 · 흡착력 · 마찰 | 12 kg · 400 N · μ_l 0.05, μ_t 0.5 | 실측 |
| ESC 데드존 · 궤도 시정수 · 최대 구동력 | 30 count · 0.15 s · 200 N | 실측 |
| ZeroKey 지연 · 잡음 · 정지 절대 오차 | 50 ms · 1 mm · 미정 | 정지 로그 |
| ZeroKey 태그 수 | 1개 (로봇) | 진행 보고서에는 마커부 + MC 2개로 기재됨. 실제 구성 확인 |
| 벽면 크리프 · heading 요철 | 8 % · ±1.5°, 0.35 Hz | 실기 영상·로그 |
| 펜 홀더 질량 | 0.10 kg | 실측 |

잔여 작업

- 실측값 반영 후 `XY_compare` 재실행
- 중앙 유지 루프의 진입 과도 개선 (적분 서서히 켜기) 후 기본값 결정
- 실기 시험 (`XY_2_pen_ros2`) 결과와 시뮬레이션 대조
- 실기 모델의 내부 PI (속도·요 각속도) 를 XY 제어기에 넣을지 결정 (12.1 절)
- heading 드리프트 보정 또는 태그를 펜 가까이 옮기는 방안 검토 (오차 예산, 10.5 절)
- 영문·베트남어 매뉴얼

---

PDF 는 이 README 에서 Chrome 헤드리스 (marked.js) 로 생성함.
