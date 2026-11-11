% XY_SETUP  AVT-3C 벽면 마킹 로봇 + XY 스테이지 시뮬레이션 파라미터
%
%   >> XY_setup        % 모델을 돌리기 전에 항상 먼저 실행
%
%   모델 두 개 (build_xy_models 가 만든다)
%     XY_1_pen_sim.slx   가상 실험: 운동모델·센서·스테이지가 ROS 2 토픽을 발행하고
%                        제어기는 그 토픽만 구독한다 (2_시뮬레이션_simulink/6WP_sim 과 같은 방식)
%     XY_2_pen_ros2.slx  실기: 같은 제어기 상자만 남긴 모델. 실제 로봇·스테이지 토픽에 붙는다
%
%   좌표 규약 (실기 모델 plane_project 와 같다)
%     p1 = world z : 높이, 천장 +
%     p2 = world x : 수평, 벽을 정면에서 볼 때 왼쪽 +
%     psi          : p1 축에서 p2 축으로 도는 각 (천장 0, 왼쪽 +90 deg)
%     바디 좌표     : x_B 전진, y_B 좌측.  R(psi) = [c -s; s c] 가 바디 -> 벽면
%
%   단위: 위치 m, 각도 rad, 스테이지 위치 mm (토픽 /stage/* 와 같음)
%   [실측 필요] 가 붙은 값은 가정값이다. 실측하면 이 파일만 고친다.

%% 0. 경로 · ROS 2 도메인
here = fileparts(mfilename('fullpath'));
addpath(here);
addpath(fullfile(here, '..', '_tools'));
%  가상 실험은 실기와 다른 도메인에서 돈다. 같은 망에 실제 로봇이 켜져 있어도
%  시뮬레이션이 발행한 /motor_pwm 이 실제 로봇을 움직이지 않게 하기 위해서다.
ros_domain_sim  = 77;          % XY_1_pen_sim (가상 실험)
ros_domain_real = 0;           % XY_2_pen_ros2 (실기 NUC 와 같은 값으로 맞출 것)
setenv('ROS_DOMAIN_ID', num2str(ros_domain_sim));

%% 1. 시뮬레이션
Ts_ctrl  = 0.01;         % 제어기 주기 = ROS 구독 주기 (100 Hz, 실기 모델과 같음)
Ts_plant = 0.005;        % 운동모델 적분 고정 스텝 (ode4)
T_run    = 70;           % 최대 시간 [s] (미션이 끝나면 자동 정지)
T_run_ros = 300;         % 실기 모델 최대 시간 [s]

%% 2. 그릴 직선 (펜 기준, 벽면 좌표 [p1; p2] m)
pen_A = [0.30;  0.40];   % 시작점
pen_B = [0.30; -0.20];   % 끝점 (0.6 m 수평선)
line_pre  = 0.06;        % 펜이 A 에 닿기 전 로봇이 미리 직진할 거리 [m]
line_post = 0.04;        % B 를 지나 더 갈 거리 [m]

%% 3. 로봇 초기 자세 (태그 위치)
q0 = [0.10; 0.55; 0.0];  % [p1; p2; psi]  천장 방향을 보고 출발

%% 4. 로봇 주행 제어 게인 — 6WP_sim / 실기 모델의 모드 함수와 같은 식
K_dist   = 1.0;   K_yaw = 2.0;          % Goto_Start / Turning (6WP 값)
v_ref    = 0.03;                        % 직선 구간 속도 [m/s] (실기 0.07, 긋는 동안 낮춤)
K_px     = 0.3;   K_py  = 5.0;   K_ptheta = 1.4;   % Line_Tracking (실기 ZeroKey 모델 값)
v_max    = 0.20;  w_max = deg2rad(45);  % 모드 상자 안 Saturation
K_c      = 0;     K_ci  = 0;        % 스테이지 중앙 유지 (비례, 적분 [1/s]) — 0 이면 끔 (비교: XY_compare)
d_reach  = 0.03;  th_tol = deg2rad(1);  % Stateflow 전이 조건 (실기 Chart 와 같음)

%% 5. 펜·태그 기하 (바디 좌표, m)   [실측 필요 — 레이저 트래커]
r_tag   = [-0.080;  0.060];   % 로봇 중심 기준 ZeroKey 태그 위치
r_pen0  = [ 0.000;  0.000];   % 로봇 중심 기준 펜 위치 (스테이지 중앙일 때, 배치 3안)
pen_d   = r_pen0 - r_tag;     % 태그 -> 펜 레버암

%% 6. 로봇 운동모델 — 6WP_sim Dynamics 의 식, AVT-3C 크기로   [실측 필요]
m_r    = 12.0;        % 로봇 질량 [kg] (스테이지 포함)                     [실측 필요]
l_trk  = 0.30;        % 궤도 접지 길이 [m]   (STL 길이 420 mm 에서 추정)
b_trk  = 0.28;        % 좌우 궤도 중심 간격 [m] (STL 폭 343 mm 에서 추정)
L_r    = sqrt((l_trk^2 + b_trk^2)/12);   % 6WP 와 같은 정의
mu_l   = 0.05;        % 구름 저항 계수 (6WP 0.6 = 흙, 여기는 강판)           [실측 필요]
mu_t   = 0.5;         % 횡 미끄럼 마찰 계수 (회전 저항)                     [실측 필요]
N_mag  = 400;         % 자석 흡착력 [N] — 6WP 의 W(=mg) 자리                 [실측 필요]
g_acc  = 9.81;
alpha_s = 0;          % ICR 이동각 (6WP compute_u 의 alpha). 0 이면 u = 0
dyn_p  = [m_r; l_trk; b_trk; mu_l; mu_t; N_mag; g_acc];

%  궤도 구동부: /motor_pwm -> ESC -> 모터 속도 루프 (6WP 의 PID Controller 두 개)
B_eff   = 0.30;               % 조향 계산용 궤도 유효 간격 [m]   [실측 필요]
pwm_dz  = 30;                 % ESC 데드존 [count]               [실측 필요]
v_full  = 0.20;               % PWM 400 count 일 때 궤도 속도 [m/s] [실측 필요]
k_pwm   = v_full/(400 - pwm_dz);
tau_trk = 0.15;               % ESC 램프 + 모터 시정수 [s]         [실측 필요]
trk_A = -eye(2)/tau_trk;  trk_B = eye(2)/tau_trk;  trk_C = eye(2);  trk_D = zeros(2);
Kp_v = 1500;  Ki_v = 15000;   % 속도 PID (감속기 모터는 벽에서 미끄러지지 않고 버틴다)
Kp_w = 1500;  Ki_w = 15000;
F_max = 200;                  % 궤도 한쪽 최대 구동력 [N]         [실측 필요]

%  실기 calc_pwm 상수
db_v = 47;   db_w = 20;
w_full  = 2*k_pwm*(400 - db_w)/B_eff;

%  벽면 외란 (영상·ZeroKey 로그에서 본 현상)                         [실측 필요]
slip_k   = 0.08;              % 이동 속도에 비례한 중력 방향 미끄러짐 (자석 궤도 크리프)
wob_amp  = deg2rad(1.5);      % 궤도 요철에 의한 heading 흔들림 진폭
wob_f    = 0.35;              % 그 주파수 [Hz]
dist_p   = [slip_k; wob_amp; wob_f];

%% 7. XY 스테이지 — Zaber X-LSM100B-E03 두 축 (X-LSM-E Datasheet · Manual + CAD)
%  데이터시트 (B 형 = medium pitch lead screw):
%    행정 101.6 mm, 최고속도 104 mm/s, 연속 추력 25 N, 피크 추력 25 N, 역구동력 24 N (+-30 %)
%    이동 질량(캐리지) 0.063 kg, 스테이지 무게 0.39 kg, 정확도 45 um, 반복정밀도 < 6 um
%    마이크로스텝 0.1905 um (모터 1 회전 2.4384 mm = 200 step x 64), 2 상 스테퍼 + 200 CPR 엔코더
%    RS-232 115200 bps, Zaber ASCII.  추력은 속도가 빠를수록 줄어든다 (Thrust-Speed 그림)
%  CAD (AVT-3C with_pen_holder): X 축(아래, 긴 축)이 Y 축 스테이지 전체를 싣고, Y 축 캐리지에 펜 홀더
stage_travel = 101.6;         % 행정 [mm]
stage_c      = stage_travel/2;% 중앙 [mm]
stage_vmax   = 104;           % 최고속도 [mm/s] (maxspeed 기본값 = 데이터시트 최고속도)
stage_acc    = 500;           % 가감속 [mm/s^2] (accel 설정)            [실측 필요: get accel]
stage_vcmd   = 0.8*stage_vmax;% 명령 변화율 제한 [mm/s] (PenCtrl)
stage_delay  = 0.01;          % 시리얼 명령 -> 동작 시작 [s] (115200 bps 한 줄 ~2 ms + 응답) [실측 필요]
stage_cmd_Ts = 0.04;          % 스테이지 노드가 move abs 를 보내는 주기 [s] (cmd_rate 25 Hz)
stage_Ts     = 0.05;          % /stage/pos 보고 주기 [s] (20 Hz)
m_carr = 0.063;               % 캐리지 이동 질량 [kg] (데이터시트)
m_body = 0.39;                % 스테이지 한 대 무게 [kg] (데이터시트 X-LSM100B)
m_pen  = 0.10;                % 펜 홀더 + 마커 + 스프링 [kg]           [실측 필요]
m_sx  = m_carr + m_body + m_pen;   % X 축이 미는 질량 = 캐리지 + Y 스테이지 + 펜 홀더 (0.55 kg)
m_sy  = m_carr + m_pen;            % Y 축이 미는 질량 = 캐리지 + 펜 홀더 (0.16 kg)
%  추력-속도 곡선 (데이터시트 X-LSMxxxB-E03 24 V 를 읽은 값)
stage_Fv_v = [0  2  7  13  20  32  45  60  80  104];     % [mm/s]
stage_Fv_F = [32 32 29 26.5 25 22  20 17.5 14  9];       % [N]
F_back = 24;                  % 역구동력 [N] — 전원이 꺼져도 축 방향 하중이 이보다 작으면 밀리지 않는다
c_s   = 2.0;                  % 점성 마찰 [N s/m]                   [실측 필요]
wn_s  = 2*pi*20;  ze_s = 0.8; % 스테퍼 강성 + 펌웨어 위치 보정을 묶은 대역 (20 Hz)  [실측 필요]
Kp_s  = [m_sx; m_sy]*wn_s^2;
Kd_s  = 2*ze_s*[m_sx; m_sy]*wn_s;
Ki_s  = Kp_s*wn_s/5;
stg_p = [m_sx; m_sy; c_s; Kp_s; Kd_s; Ki_s; g_acc];   % 순서: ZaberServo 안에서 풀어 쓴다
x_s0  = [stage_c; stage_c; 0; 0; 0; 0] .* [1e-3; 1e-3; 1; 1; 1; 1];
x_t0  = [stage_c; stage_c; 0; 0];                      % 펌웨어 궤적 상태 [mm; mm/s]
%  벽면 중력의 축 성분 점검: X 축 m_sx g = 5.4 N, Y 축 1.6 N  ->  연속 추력 25 N, 역구동력 24 N 보다 작다

%% 8. 센서
zk_Ts     = 0.05;             % ZeroKey 갱신 주기 [s] (INSMODE 2 ~20 Hz)
zk_delay  = 0.05;             % ZeroKey 지연 [s]       [실측 필요]
zk_sigma  = 0.0010;           % 위치 잡음 1 sigma [m]
zk_ywall  = 0.02;             % 벽 법선 오프셋 world y [m] (진단용)
imu_Ts    = 0.01;             % /imu/data 주기 [s]
imu_sigma = deg2rad(0.3);     % heading 잡음 1 sigma
imu_drift = deg2rad(0.02);    % heading 드리프트 [rad/s] (철판 벽, 자력계 없음)
psi0      = 0;                % 출발 heading (천장 방향)
tau_pred  = 0;                % ZeroKey 지연 보상 시간 [s] (Nav, 0 이면 끔 — 비교: XY_compare)

%% 9. 애니메이션
anim_speed = 2.0;
anim_fps   = 25;
animate    = 1;               % Run 버튼으로 돌릴 때 실시간 화면 (0 이면 끔, 빨라진다)
anim_Ts    = 0.1;             % 실시간 화면 갱신 주기 [s]
XY_batch   = 0;               % 1 이면 StopFcn (XY_stop) 이 그림을 그리지 않는다 — 스크립트가 세운다
stl_file   = fullfile(here, '..', '4_CAD', 'AVT-3C.stl');

fprintf('XY_setup 완료: 선 %.2f m, 레버암 %.3f m, ZeroKey %.0f Hz, ROS_DOMAIN_ID %d (가상 실험)\n', ...
        norm(pen_B - pen_A), norm(pen_d), 1/zk_Ts, ros_domain_sim);
