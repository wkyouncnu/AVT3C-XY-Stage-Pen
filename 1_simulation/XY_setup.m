% XY_SETUP  AVT-3C 벽면 마킹 로봇 + XY 스테이지 시뮬레이션 파라미터
%           Simulation parameters of the AVT-3C wall-marking robot + XY stage
%
%   >> XY_setup        % 모델을 돌리기 전에 항상 먼저 실행 / always run first, before running a model
%
%   모델 두 개 (build_xy_models 가 만든다)
%   Two models (made by build_xy_models)
%     XY_1_pen_sim.slx   가상 실험: 운동모델·센서·스테이지가 ROS 2 토픽을 발행하고
%                        제어기는 그 토픽만 구독한다 (2_시뮬레이션_simulink/6WP_sim 과 같은 방식)
%                        Virtual experiment: the plant model, sensors and stage publish ROS 2 topics and
%                        the controller only subscribes to them (same scheme as 2_시뮬레이션_simulink/6WP_sim)
%     XY_2_pen_ros2.slx  실기: 같은 제어기 상자만 남긴 모델. 실제 로봇·스테이지 토픽에 붙는다
%                        Real robot: a model with only the same controller box, attached to the real robot/stage topics
%
%   좌표 규약 (ROS REP-103 형, 2026-11-13 변경)
%     벽면 x : 수평, 벽을 정면에서 볼 때 오른쪽 +
%     벽면 y : 높이, 천장 +
%     벽면 z : 벽 바깥 (로봇 z_B 와 같은 방향) -> 오른손 좌표계
%     psi    : x 축에서 y 축으로 도는 각 (오른쪽 0, 천장 +90 deg, 반시계 +)
%     바디   : x_B 전진, y_B 좌측, z_B 벽 바깥.  R(psi) = [c -s; s c] 가 바디 -> 벽면
%     ZeroKey 월드 -> 벽면: x = zk_xsign * zk.x,  y = zk.z  (zk.y 는 벽 법선, 버림)
%   Coordinates (ROS REP-103 style): wall x right, wall y up, z out of the wall,
%   psi from x toward y (0 = right, +90 deg = up). Body x_B forward, y_B left.
%   Earlier versions used p1 = up, p2 = left (real plane_project): x = -p2, y = p1, psi = psi_old + 90 deg.
%   Coordinate convention (ROS REP-103 style, changed 2026-11-13)
%     wall x : horizontal, + to the right when facing the wall
%     wall y : height, + toward the ceiling
%     wall z : out of the wall (same direction as the robot z_B) -> right-handed frame
%     psi    : angle from the x axis toward the y axis (right 0, ceiling +90 deg, counter-clockwise +)
%     body   : x_B forward, y_B left, z_B out of the wall.  R(psi) = [c -s; s c] maps body -> wall
%     ZeroKey world -> wall: x = zk_xsign * zk.x,  y = zk.z  (zk.y is the wall normal, discarded)
%
%   단위: 위치 m, 각도 rad, 스테이지 위치 mm (토픽 /stage/* 와 같음)
%   [실측 필요] 가 붙은 값은 가정값이다. 실측하면 이 파일만 고친다.
%   Units: position m, angle rad, stage position mm (same as the /stage/* topics)
%   Values marked [실측 필요] (measurement needed) are assumptions. After measuring, edit only this file.

%% 0. 경로 · ROS 2 도메인 / Path · ROS 2 domain
here = fileparts(mfilename('fullpath'));
addpath(here);
addpath(fullfile(here, '..', '_tools'));
%  가상 실험은 실기와 다른 도메인에서 돈다. 같은 망에 실제 로봇이 켜져 있어도
%  시뮬레이션이 발행한 /motor_pwm 이 실제 로봇을 움직이지 않게 하기 위해서다.
%  The virtual experiment runs in a domain different from the real robot, so that /motor_pwm
%  published by the simulation does not move a real robot that is switched on in the same network.
ros_domain_sim  = 77;          % XY_1_pen_sim (가상 실험 / virtual experiment)
ros_domain_real = 0;           % XY_2_pen_ros2 (실기 NUC 와 같은 값으로 맞출 것 / set equal to the real-robot NUC)
setenv('ROS_DOMAIN_ID', num2str(ros_domain_sim));

%% 1. 시뮬레이션 / Simulation
Ts_ctrl  = 0.01;         % 제어기 주기 = ROS 구독 주기 (100 Hz, 실기 모델과 같음) / controller period = ROS subscribe period (100 Hz, same as real model)
Ts_plant = 0.0005;       % 운동모델 적분 고정 스텝 (ode4). 5 ms 는 회전 마찰 tanh(w/0.01) 에 비해 커서 yaw rate 가 수치적으로 떨린다 (2026-11-11 확인)
                         % plant model fixed integration step (ode4). 5 ms is too large for the turn friction tanh(w/0.01): yaw rate chatters numerically (checked 2026-11-11)
T_run    = 70;           % 최대 시간 [s] (미션이 끝나면 자동 정지) / maximum time [s] (stops automatically when the mission ends)
T_run_ros = 300;         % 실기 모델 최대 시간 [s] / real-robot model maximum time [s]

%% 2. 그릴 직선 (펜 기준, 벽면 좌표 [x; y] m) / Line to draw (pen point, wall frame [x; y] m)
pen_A = [-0.70; 0.30];   % 시작점 / start point
pen_B = [ 0.50; 0.30];   % 끝점 (높이 0.30 m 에서 오른쪽으로 1.2 m) / end point (1.2 m to the right at height 0.30 m)
line_pre  = 0.06;        % 펜이 A 에 닿기 전 로봇이 미리 직진할 거리 [m] / distance the robot drives straight before the pen reaches A [m]
line_post = 0.04;        % B 를 지나 더 갈 거리 [m] / distance to continue past B [m]

%% 3. 로봇 초기 자세 (태그 위치) / Robot initial pose (tag position)
q0 = [-0.85; 0.10; pi/2];  % [x; y; psi]  천장 방향 (psi = 90 deg) 을 보고 출발 / starts facing the ceiling (psi = 90 deg)

%% 4. 로봇 주행 제어 게인 — 6WP_sim / 실기 모델의 모드 함수와 같은 식
%     Robot driving controller gains — same equations as the mode functions of 6WP_sim / the real model
K_dist   = 1.0;   K_yaw = 2.0;          % Goto_Start / Turning (6WP 값 / 6WP values)
v_ref    = 0.03;                        % 직선 구간 속도 [m/s] (실기 0.07, 긋는 동안 낮춤) / line speed [m/s] (real 0.07, lowered while drawing)
K_px     = 0.3;   K_py  = 5.0;   K_ptheta = 1.4;   % Line_Tracking (실기 ZeroKey 모델 값 / real ZeroKey model values)
v_max    = 0.20;  w_max = deg2rad(45);  % 모드 상자 안 Saturation / Saturation inside the mode box
K_c      = 0;     K_ci  = 0;        % 스테이지 중앙 유지 (비례, 적분 [1/s]) — 0 이면 끔 (비교: XY_compare)
                                    % stage centring (proportional, integral [1/s]) — 0 disables it (comparison: XY_compare)
d_reach  = 0.03;  th_tol = deg2rad(1);  % Stateflow 전이 조건 (실기 Chart 와 같음) / Stateflow transition conditions (same as real Chart)

%% 5. 펜·태그 기하 (바디 좌표, m)   [실측 필요 — 레이저 트래커]
%     Pen/tag geometry (body frame, m)   [measurement needed — laser tracker]
r_tag   = [-0.080;  0.060];   % 로봇 중심 기준 ZeroKey 태그 위치 / ZeroKey tag position relative to the robot centre
r_pen0  = [ 0.000;  0.000];   % 로봇 중심 기준 펜 위치 (스테이지 중앙일 때, 배치 3안) / pen position from robot centre (stage centred, layout 3)
pen_d   = r_pen0 - r_tag;     % 태그 -> 펜 레버암 / tag -> pen lever arm
%  q0 는 태그 자세. 운동모델은 중심 c 를 적분하므로 초기 중심 q0c = [q0(1:2) - R(psi0) r_tag; psi0]
%  q0 is the tag pose. The plant integrates the centre c, so its initial state is q0c
q0c = [q0(1:2) - [cos(q0(3)) -sin(q0(3)); sin(q0(3)) cos(q0(3))] * r_tag; q0(3)];

%% 6. 로봇 운동모델 — 6WP_sim Dynamics 의 식, AVT-3C 크기로   [실측 필요]
%     Robot plant model — equations of 6WP_sim Dynamics, at AVT-3C size   [measurement needed]
m_r    = 12.0;        % 로봇 질량 [kg] (스테이지 포함) / robot mass [kg] (stage included)   [실측 필요]
l_trk  = 0.30;        % 궤도 접지 길이 [m]   (STL 길이 420 mm 에서 추정) / track contact length [m] (estimated from STL length 420 mm)
b_trk  = 0.28;        % 좌우 궤도 중심 간격 [m] (STL 폭 343 mm 에서 추정) / left-right track centre spacing [m] (estimated from STL width 343 mm)
L_r    = sqrt((l_trk^2 + b_trk^2)/12);   % 6WP 와 같은 정의 / same definition as 6WP
mu_l   = 0.05;        % 구름 저항 계수 (6WP 0.6 = 흙, 여기는 강판) / rolling resistance coefficient (6WP 0.6 = soil, here steel plate)   [실측 필요]
mu_t   = 0.5;         % 횡 미끄럼 마찰 계수 (회전 저항) / lateral sliding friction coefficient (turning resistance)   [실측 필요]
N_mag  = 400;         % 자석 흡착력 [N] — 6WP 의 W(=mg) 자리 / magnetic adhesion force [N] — in place of W(=mg) in 6WP   [실측 필요]
g_acc  = 9.81;
alpha_s = 0;          % ICR 이동각 (6WP compute_u 의 alpha). 0 이면 u = 0 / ICR shift angle (alpha of 6WP compute_u). 0 gives u = 0
dyn_p  = [m_r; l_trk; b_trk; mu_l; mu_t; N_mag; g_acc];

%  궤도 구동부: /motor_pwm -> ESC -> 모터 속도 루프 (6WP 의 PID Controller 두 개)
%  Track drive: /motor_pwm -> ESC -> motor speed loop (the two PID Controllers of 6WP)
B_eff   = 0.30;               % 조향 계산용 궤도 유효 간격 [m] / effective track spacing for steering [m]   [실측 필요]
pwm_dz  = 30;                 % ESC 데드존 [count] / ESC dead zone [count]               [실측 필요]
v_full  = 0.20;               % PWM 400 count 일 때 궤도 속도 [m/s] / track speed at PWM 400 count [m/s] [실측 필요]
k_pwm   = v_full/(400 - pwm_dz);
tau_trk = 0.15;               % ESC 램프 + 모터 시정수 [s] / ESC ramp + motor time constant [s]         [실측 필요]
trk_A = -eye(2)/tau_trk;  trk_B = eye(2)/tau_trk;  trk_C = eye(2);  trk_D = zeros(2);
Kp_v = 1500;  Ki_v = 15000;   % 속도 PID (감속기 모터는 벽에서 미끄러지지 않고 버틴다)
                              % speed PID (the geared motor holds on the wall without slipping)
Kp_w = 1500;  Ki_w = 15000;
F_max = 200;                  % 궤도 한쪽 최대 구동력 [N] / maximum drive force of one track [N]         [실측 필요]

%  실기 calc_pwm 상수 / Real-robot calc_pwm constants
db_v = 47;   db_w = 20;
w_full  = 2*k_pwm*(400 - db_w)/B_eff;

%  벽면 외란 (영상·ZeroKey 로그에서 본 현상)                         [실측 필요]
%  Wall disturbances (effects seen in videos and ZeroKey logs)       [measurement needed]
slip_k   = 0.08;              % 이동 속도에 비례한 중력 방향 미끄러짐 (자석 궤도 크리프) / downward slip proportional to speed (magnetic track creep)
wob_amp  = deg2rad(1.5);      % 궤도 요철에 의한 heading 흔들림 진폭 / heading wobble amplitude due to track unevenness
wob_f    = 0.35;              % 그 주파수 [Hz] / its frequency [Hz]
dist_p   = [slip_k; wob_amp; wob_f];

%% 7. XY 스테이지 — Zaber X-LSM100B-E03 두 축 (X-LSM-E Datasheet · Manual + CAD)
%     XY stage — two Zaber X-LSM100B-E03 axes (X-LSM-E Datasheet · Manual + CAD)
%  데이터시트 (B 형 = medium pitch lead screw):
%    행정 101.6 mm, 최고속도 104 mm/s, 연속 추력 25 N, 피크 추력 25 N, 역구동력 24 N (+-30 %)
%    이동 질량(캐리지) 0.063 kg, 스테이지 무게 0.39 kg, 정확도 45 um, 반복정밀도 < 6 um
%    마이크로스텝 0.1905 um (모터 1 회전 2.4384 mm = 200 step x 64), 2 상 스테퍼 + 200 CPR 엔코더
%    RS-232 115200 bps, Zaber ASCII.  추력은 속도가 빠를수록 줄어든다 (Thrust-Speed 그림)
%  Datasheet (type B = medium pitch lead screw):
%    travel 101.6 mm, maximum speed 104 mm/s, continuous thrust 25 N, peak thrust 25 N, back-drive force 24 N (+-30 %)
%    moving mass (carriage) 0.063 kg, stage weight 0.39 kg, accuracy 45 um, repeatability < 6 um
%    microstep 0.1905 um (one motor turn 2.4384 mm = 200 step x 64), 2-phase stepper + 200 CPR encoder
%    RS-232 115200 bps, Zaber ASCII.  Thrust decreases as speed increases (Thrust-Speed figure)
%  CAD (AVT-3C with_pen_holder): X 축(아래, 긴 축)이 Y 축 스테이지 전체를 싣고, Y 축 캐리지에 펜 홀더
%  CAD (AVT-3C with_pen_holder): the X axis (lower, long axis) carries the whole Y-axis stage; the pen holder sits on the Y carriage
stage_travel = 101.6;         % 행정 [mm] / travel [mm]
stage_c      = stage_travel/2;% 중앙 [mm] / centre [mm]
stage_vmax   = 104;           % 최고속도 [mm/s] (maxspeed 기본값 = 데이터시트 최고속도) / maximum speed [mm/s] (maxspeed default = datasheet value)
stage_acc    = 500;           % 가감속 [mm/s^2] (accel 설정) / acceleration [mm/s^2] (accel setting)   [실측 필요: get accel]
stage_vcmd   = 0.8*stage_vmax;% 명령 변화율 제한 [mm/s] (PenCtrl) / command rate limit [mm/s] (PenCtrl)
stage_delay  = 0.01;          % 시리얼 명령 -> 동작 시작 [s] (115200 bps 한 줄 ~2 ms + 응답) [실측 필요]
                              % serial command -> motion start [s] (one line at 115200 bps ~2 ms + reply) [measurement needed]
stage_cmd_Ts = 0.04;          % 스테이지 노드가 move abs 를 보내는 주기 [s] (cmd_rate 25 Hz) / period at which the stage node sends move abs [s]
stage_Ts     = 0.05;          % /stage/pos 보고 주기 [s] (20 Hz) / /stage/pos report period [s] (20 Hz)
m_carr = 0.063;               % 캐리지 이동 질량 [kg] (데이터시트) / carriage moving mass [kg] (datasheet)
m_body = 0.39;                % 스테이지 한 대 무게 [kg] (데이터시트 X-LSM100B) / weight of one stage [kg] (datasheet X-LSM100B)
m_pen  = 0.10;                % 펜 홀더 + 마커 + 스프링 [kg] / pen holder + marker + spring [kg]           [실측 필요]
m_sx  = m_carr + m_body + m_pen;   % X 축이 미는 질량 = 캐리지 + Y 스테이지 + 펜 홀더 (0.55 kg)
                                   % mass moved by the X axis = carriage + Y stage + pen holder (0.55 kg)
m_sy  = m_carr + m_pen;            % Y 축이 미는 질량 = 캐리지 + 펜 홀더 (0.16 kg) / mass moved by the Y axis = carriage + pen holder (0.16 kg)
%  추력-속도 곡선 (데이터시트 X-LSMxxxB-E03 24 V 를 읽은 값)
%  Thrust-speed curve (values read from datasheet X-LSMxxxB-E03 at 24 V)
stage_Fv_v = [0  2  7  13  20  32  45  60  80  104];     % [mm/s]
stage_Fv_F = [32 32 29 26.5 25 22  20 17.5 14  9];       % [N]
F_back = 24;                  % 역구동력 [N] — 전원이 꺼져도 축 방향 하중이 이보다 작으면 밀리지 않는다
                              % back-drive force [N] — with power off, the axis does not move if the axial load is smaller than this
c_s   = 2.0;                  % 점성 마찰 [N s/m] / viscous friction [N s/m]                   [실측 필요]
wn_s  = 2*pi*20;  ze_s = 0.8; % 스테퍼 강성 + 펌웨어 위치 보정을 묶은 대역 (20 Hz)  [실측 필요]
                              % bandwidth lumping stepper stiffness + firmware position correction (20 Hz)  [measurement needed]
Kp_s  = [m_sx; m_sy]*wn_s^2;
Kd_s  = 2*ze_s*[m_sx; m_sy]*wn_s;
Ki_s  = Kp_s*wn_s/5;
stg_p = [m_sx; m_sy; c_s; Kp_s; Kd_s; Ki_s; g_acc];   % 순서: ZaberServo 안에서 풀어 쓴다 / order: unpacked inside ZaberServo
x_s0  = [stage_c; stage_c; 0; 0; 0; 0] .* [1e-3; 1e-3; 1; 1; 1; 1];
x_t0  = [stage_c; stage_c; 0; 0];                      % 펌웨어 궤적 상태 [mm; mm/s] / firmware trajectory state [mm; mm/s]
%  벽면 중력의 축 성분 점검: X 축 m_sx g = 5.4 N, Y 축 1.6 N  ->  연속 추력 25 N, 역구동력 24 N 보다 작다
%  Check of wall gravity along the axes: X axis m_sx g = 5.4 N, Y axis 1.6 N  ->  smaller than continuous thrust 25 N and back-drive force 24 N

%% 8. 센서 / Sensors
zk_Ts     = 0.05;             % ZeroKey 갱신 주기 [s] (INSMODE 2 ~20 Hz) / ZeroKey update period [s] (INSMODE 2 ~20 Hz)
zk_delay  = 0.05;             % ZeroKey 지연 [s] / ZeroKey delay [s]       [실측 필요]
zk_sigma  = 0.0010;           % 위치 잡음 1 sigma [m] / position noise 1 sigma [m]
zk_ywall  = 0.02;             % 벽 법선 오프셋 world y [m] (진단용) / wall-normal offset world y [m] (for diagnosis)
zk_xsign  = -1;               % ZeroKey world x 의 방향: -1 = 벽 정면에서 왼쪽 + (실기 plane_project 가정), +1 = 오른쪽 +  [현장 확인 필요]
                              % ZeroKey world x direction: -1 = left positive (as assumed by plane_project), +1 = right positive
imu_Ts    = 0.01;             % /imu/data 주기 [s] / /imu/data period [s]
imu_sigma = deg2rad(0.3);     % heading 잡음 1 sigma / heading noise 1 sigma
imu_drift = deg2rad(0.02);    % heading 드리프트 [rad/s] (철판 벽, 자력계 없음) / heading drift [rad/s] (steel wall, no magnetometer)
psi0      = pi/2;             % 출발 heading [rad] (천장 방향 = 90 deg). IMU 첫 요각을 이 값으로 맞춘다
                              % start heading [rad] (toward the ceiling = 90 deg). The first IMU yaw is aligned to this value
tau_pred  = 0;                % ZeroKey 지연 보상 시간 [s] (Nav, 0 이면 끔 — 비교: XY_compare)
                              % ZeroKey delay compensation time [s] (Nav, 0 disables it — comparison: XY_compare)

%% 9. 애니메이션 / Animation
anim_speed = 2.0;
anim_fps   = 25;
animate    = 1;               % Run 버튼으로 돌릴 때 실시간 화면 (0 이면 끔, 빨라진다) / live view when run with the Run button (0 disables it, faster)
anim_Ts    = 0.2;             % 실시간 화면 갱신 주기 [s] (Animate 입력의 Zero-Order Hold 주기) / live view update period [s] (Zero-Order Hold of the Animate input)
XY_batch   = 0;               % 1 이면 StopFcn (XY_stop) 이 그림을 그리지 않는다 — 스크립트가 세운다
                              % 1 = StopFcn (XY_stop) draws no figures — set by scripts
stl_file   = fullfile(here, '..', '4_CAD', 'AVT-3C.stl');

%% 10. 2 판 내부 PI (XY_1_pen_sim_v2, XY_2_pen_ros2_v2 만 쓴다 — 1 판 모델은 이 값을 읽지 않는다)
%      Version 2 inner PI (used only by XY_1_pen_sim_v2, XY_2_pen_ros2_v2 — version 1 models do not read these values)
%  실기 Tracked_UGV_ZeroKey_XsensIMU 의 PID Controller2 (속도) · 3 (요 각속도) 와 같은 값
%  v2 only: the inner PI loops of the real Xsens model (same gains, limits and feedback).
%  Same values as PID Controller2 (speed) and 3 (yaw rate) of the real Tracked_UGV_ZeroKey_XsensIMU
Kin_Pv = 0.15;  Kin_Iv = 0.8;     % 속도 PI:   e_v = v_cmd - u_body,             출력 V in [0, 0.5] / speed PI, output V in [0, 0.5]
Kin_Pw = 1.5;   Kin_Iw = 1.5;     % 요 각속도 PI: e_w = w_cmd - w_fb_gain * r,     출력 W in [-0.5, 0.5] / yaw-rate PI, output W in [-0.5, 0.5]
w_fb_gain = 1.0;                  % 자이로 z 되먹임 배율. 실기 모델 Multiply7 은 0.1443 (rad/s 명령과 단위가 안 맞아 시뮬레이션에서 ω 진동) [확인 필요]
                                  % gyro z feedback scale. The real model's Multiply7 uses 0.1443 (unit mismatch with the rad/s command, ω oscillates in simulation) [to be checked]
inner_reset = 1;                  % 1: 모드가 바뀔 때 PI 적분기 리셋 (실기 모델에는 없음). 0: 실기와 같음
                                  % 1: reset the PI integrators when the mode changes (not in the real model). 0: same as the real robot
imu_gyro_sigma = deg2rad(0.1);    % 자이로 z 잡음 1 sigma [rad/s] (Xsens MTi 급) / gyro z noise 1 sigma [rad/s] (Xsens MTi class)
imu_gyro_bias  = 0;               % 자이로 z 바이어스 [rad/s] / gyro z bias [rad/s]

fprintf('XY_setup 완료: 선 %.2f m, 레버암 %.3f m, ZeroKey %.0f Hz, ROS_DOMAIN_ID %d (가상 실험)\n', ...
        norm(pen_B - pen_A), norm(pen_d), 1/zk_Ts, ros_domain_sim);
