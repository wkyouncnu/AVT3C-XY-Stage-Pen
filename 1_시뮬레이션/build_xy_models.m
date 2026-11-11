function build_xy_models(which, ver)
% BUILD_XY_MODELS  XY 스테이지 펜 제어 Simulink 모델을 코드로 만든다.
%
%   >> XY_setup
%   >> build_xy_models          % 두 모델 모두
%   >> build_xy_models(1)       % 가상 실험 모델만
%
%     XY_1_pen_sim.slx    가상 실험. 2_시뮬레이션_simulink/6WP_sim 과 같은 방식:
%                         운동모델·센서·스테이지가 ROS 2 토픽을 **발행**하고,
%                         제어기는 그 토픽을 **구독**해서만 상태를 안다.
%     XY_2_pen_ros2.slx   실기. 같은 제어기 상자 (RosIn ~ RosOut) 만 남긴다.
%
%   토픽 (실기와 같은 이름 · 같은 형식)
%     운동모델 -> 제어기 : /zerokey/position  geometry_msgs/PointStamped
%                          /zerokey/vel       geometry_msgs/TwistStamped
%                          /imu/data          sensor_msgs/Imu
%     스테이지 -> 제어기 : /stage/pos         geometry_msgs/PointStamped  [mm]
%     제어기 -> 운동모델 : /motor_pwm         geometry_msgs/Point  (x 왼쪽, y 오른쪽 PWM)
%     제어기 -> 스테이지 : /stage/target_pos  geometry_msgs/Point  [mm]
%
%   윗줄 (제어기 = 실기 코드)   RosIn -> Nav -> Mission -> RobotCtrl -> PwmMap -> RosOut
%                                                         PenCtrl ----------------^
%   아랫줄 (가상 실험장)        PlantRosIn -> RobotPlant -> Sensors -> PlantRosOut
%                                          StagePlant ----------------^
%   PenTrace · Logging : 포트 없는 상자 (평가·기록)
%
%   계산 코드는 모두 모델 안 MATLAB Function 블록에 들어 있다 (외부 .m 함수 없음).
%   이 빌더가 그 코드를 블록에 써 넣는다 — 모델을 고칠 때는 이 파일을 고치고 다시 만든다.

%   >> build_xy_models([1 2], 2)   % 2 판: 실기 모델의 내부 PI (속도·요 각속도) 를 넣은 모델
%     XY_1_pen_sim_v2.slx, XY_2_pen_ros2_v2.slx  — 1 판 모델 (XY_1_pen_sim, XY_2_pen_ros2) 은 건드리지 않는다
%     2 판에서 바뀌는 것 / v2 adds (1 판은 그대로):
%       RosIn   /imu/data 에서 angular_velocity.z 도 읽는다 (imu_data = [쿼터니언; 자이로 z])
%       Nav     V_meas = [u_body; r] 를 낸다 (ZeroKey 속도를 heading 으로 바디 좌표에, IMU 자이로 z)
%       InnerPI 실기 Tracked_UGV_ZeroKey_XsensIMU 의 PID Controller2·3 과 같은 이산 PI 두 개
%       PwmMap  PI 출력 V, W (-0.5 ~ 0.5) 를 그대로 calc_pwm 에 넣는다 (속도 정규화 없음)
%       Sensors IMU 메시지에 자이로 z (참 각속도 + 잡음) 를 같이 실어 발행

    if nargin < 1, which = [1 2]; end
    if nargin < 2, ver = 1; end
    ver2(ver == 2);
    sfx = '';  if ver == 2, sfx = '_v2'; end
    here = fileparts(mfilename('fullpath'));
    cd(here);
    addpath(here);  addpath(fullfile(here, '..', '_tools'));
    if evalin('base', '~exist(''dyn_p'',''var'')')
        error('먼저 XY_setup 을 실행하십시오.');
    end
    if ~isfolder(fullfile(here,'img')), mkdir(fullfile(here,'img')); end

    if any(which == 1), build_sim(['XY_1_pen_sim' sfx]);   finish_model(['XY_1_pen_sim' sfx], here);  end
    if any(which == 2), build_real(['XY_2_pen_ros2' sfx]); finish_model(['XY_2_pen_ros2' sfx], here); end
    ver2(false);
    fprintf('\n완료.\n');
end

function v = ver2(set)
%  2 판 (내부 PI) 을 만드는 중인지 / true while building the v2 (inner PI) models
    persistent V
    if isempty(V), V = false; end
    if nargin, V = set; end
    v = V;
end

% =====================================================================
%  모델 두 개
% =====================================================================
function build_sim(m)
    fresh(m);
    set_param(m, 'SolverType','Fixed-step', 'SolverName','ode4', ...
                 'FixedStep','Ts_plant', 'StopTime','T_run', 'SimulationMode','normal');
    %  ROS 2 루프백: 토픽이 실제 DDS 를 지나므로 실시간보다 빨리 돌리지 않는다
    %  Run 버튼: To Workspace log 가 base 에 바로 생기고, 끝나면 XY_stop 이 결과 그림을 띄운다
    set_param(m, 'ReturnWorkspaceOutputs','off', 'StopFcn','XY_stop');
    try, set_param(m, 'EnablePacing','on', 'PacingRate', 4); catch, end
    L = layout();
    addController(m, L);
    addPlantRosIn(m,  L.prin);
    addRobotPlant(m,  L.rplant);
    addSensors(m,     L.sensors);
    addStagePlant(m,  L.splant);
    addPlantRosOut(m, L.prout);
    addPenTrace(m,    L.trace);
    addLogging(m,     L.log, log_signals(true));
    %  실시간 화면 (끄기: animate = 0). 그리는 일은 XY_animate.m
    %  Animate 입력마다 Zero-Order Hold (anim_Ts) 를 넣는다. 그러지 않으면 입력이 연속 신호라
    %  적분 스텝(0.5 ms)마다 XY_animate 가 불려 실행이 수십 배 느려진다
    %  Zero-order holds at anim_Ts on every input: otherwise the drawing call runs every solver step.
    add_animate_box(m, {'q_true','pen_true','pen_on','stage_pos','lineAB','e_pen','v_cmd','w_cmd','V_true','mode'}, 'XY_animate', '', [L.log(1)+330 L.log(2)], 'anim_Ts');
    hold_inputs([m '/Animate'], 'AnimateFcn', 'anim_Ts');
    wire_direct(m);
    note(m, 'n_title', [m ' : AVT-3C 벽면 로봇 + XY 스테이지 펜 직선 긋기 — 가상 실험 (ROS 2 루프백)' v2_line() newline ...
        '윗줄 = 제어기 (XY_2_pen_ros2 와 같은 상자).  아랫줄 = 가상 실험장 (6WP_sim 동역학 + 센서 + Zaber 모델)' newline ...
        '제어기와 가상 실험장은 선이 아니라 ROS 2 토픽으로만 이어진다 (RosOut → PlantRosIn,  PlantRosOut → RosIn).   실행 전 >> XY_setup'], 60, 10);
    note(m, 'n_plant', ['▼ 가상 실험장 — /motor_pwm · /stage/target_pos 를 구독하고' newline ...
        '   /zerokey/position · /zerokey/vel · /imu/data · /zerokey/orientation · /stage/pos 를 발행한다'], L.prin(1), L.prin(2) - 70);
    note(m, 'n_eval', '▼ 평가·기록 — 참값으로 계산한 펜 자국 (제어기는 모른다), To Workspace log', L.trace(1), L.trace(2) - 50);
    save_system(m);  close_system(m);
    fprintf('  %s 생성\n', m);
end

function build_real(m)
    fresh(m);
    set_param(m, 'SolverType','Fixed-step', 'SolverName','ode4', ...
                 'FixedStep','Ts_ctrl', 'StopTime','T_run_ros', 'SimulationMode','normal');
    set_param(m, 'ReturnWorkspaceOutputs','off', 'StopFcn','XY_stop');
    try, set_param(m, 'EnablePacing','on', 'PacingRate', 1); catch, end
    L = layout();
    addController(m, L);
    addLogging(m, L.log, log_signals(false));
    wire_direct(m);
    note(m, 'n_title', [m ' : 실기 — 가상 실험 모델의 윗줄(제어기)과 같은 상자' v2_line() newline ...
        '구독 /zerokey/position /zerokey/vel /imu/data /stage/pos    발행 /motor_pwm /stage/target_pos' newline ...
        '실행 전 >> XY_setup ; setenv(''ROS_DOMAIN_ID'', num2str(ros_domain_real))'], 60, 10);
    save_system(m);  close_system(m);
    fprintf('  %s 생성\n', m);
end

function s = v2_line()
%  2 판이면 제목 주석에 붙일 한 줄 / extra title line for the v2 models
    s = '';
    if ver2()
        s = '   [2 판: InnerPI = 실기 내부 PI, 되먹임 V_meas]';
    end
end

%  제어기 = 두 모델이 공유하는 상자들 (같은 함수로 만든다)
function addController(m, L)
    addRosIn(m,     L.rosin);
    addNav(m,       L.nav);
    addMission(m,   L.mission);
    addRobotCtrl(m, L.robot);
    if ver2(), addInnerPI(m, L.inner); end
    addPwmMap(m,    L.pwm);
    addPenCtrl(m,   L.pen);
    addRosOut(m,    L.rosout);
end

function finish_model(m, here)
%  강의 도구로 색·치수를 맞추고 검사·도면을 남긴다. 도구가 없어도 모델은 남는다.
    load_system(m);
    %  서브시스템 안: arrangeSystem 으로 층 배치 (선이 왼쪽 -> 오른쪽으로 흐른다).
    %  맨 위: 빌더 자리를 쓰고, 상자를 앞 상자의 포트 높이에 맞춘 뒤 선을 다시 긋는다.
    %  (tidy_model 은 쓰지 않는다: 되먹임 선을 다시 Goto/From 으로 바꾸고 출력 없는 ROS 상자를 구석으로 내린다)
    arrange_subs(m);
    try, paint_roles(m); catch e, warning('paint_roles: %s', e.message); end
    try, mss_style(m);   catch e, warning('mss_style: %s', e.message); end
    align_root(m);
    connect_root(m);
    fix_const_crossings(m);
    snap_jogs(m);
    save_system(m);
    try
        [n, ~, n2] = check_lines(m, false);
        fprintf('  %s check_lines 지적 %d, 꺾임2회 %d\n', m, n, n2);
    catch e, warning('check_lines: %s', e.message); end
    try, check_tags(m, true);   catch e, warning('check_tags: %s', e.message); end
    try, nf = check_flow(m); fprintf('  %s check_flow 어긋남 %d\n', m, nf); catch e, warning('check_flow: %s', e.message); end
    try
        export_diagram(m, fullfile(here,'img'));
        subs = find_system(m, 'SearchDepth',1, 'BlockType','SubSystem');
        for k = 1:numel(subs)
            export_diagram(m, fullfile(here,'img'), get_param(subs{k}, 'Name'));
        end
        export_diagram(m, fullfile(here,'img'), 'RobotCtrl/M3_Line');
    catch e
        warning('export_diagram: %s', e.message);
    end
    close_system(m, 0);
end

% =====================================================================
%  배치 — 상자 자리 [x y 폭 포트수]
% =====================================================================
function L = layout()
%  강의 규칙 (check_flow): 왼쪽 -> 오른쪽 = 항법 -> 미션 -> 제어 -> 배분 -> ROS 경계·플랜트 -> 기록
%    윗줄   제어기   RosIn -> Nav -> Mission -> RobotCtrl -> PwmMap -> RosOut   (PenCtrl 은 RobotCtrl 아래)
%    오른쪽 아래     가상 실험장  PlantRosIn -> RobotPlant -> Sensors -> PlantRosOut  (StagePlant 는 RobotPlant 아래)
%    왼쪽 아래       평가·기록 (PenTrace, Logging) — 사슬 전체보다 아래
%  상자 사이는 선으로 바로 잇는다 (Goto/From 은 기록용 탭만). 그래서 열 간격은 선 이름이 들어갈 만큼만
    W = 160;  Y1 = 140;
    col = @(k) 120 + (k-1)*330;
    L.rosin   = [col(1) Y1      W 4];
    L.nav     = [col(2) Y1      W 3];
    L.mission = [col(3) Y1      W 5];
    L.robot   = [col(4) Y1      W 6];
    L.pen     = [col(4) Y1+430  W 3];
    L.pwm     = [col(5) Y1      W 2];
    L.rosout  = [col(6) Y1      W 2];
    if ver2()   % 2 판: RobotCtrl 과 PwmMap 사이에 InnerPI 열 하나
        L.inner  = [col(5) Y1      W 3];
        L.pwm    = [col(6) Y1      W 2];
        L.rosout = [col(7) Y1      W 2];
    end
    Y2 = 480;  cp = @(k) col(5) + 60 + (k-1)*330;
    L.prin    = [cp(1) Y2       W 2];
    L.rplant  = [cp(2) Y2       W 2];
    L.splant  = [cp(2) Y2+200   W 2];
    L.sensors = [cp(3) Y2       W 4];
    L.prout   = [cp(4) Y2       W 5];
    L.trace   = [col(1) 900];
    L.log     = [col(2) 900];
end

function pos = boxpos(P, nports)
    h = max(80, 60*nports);
    pos = [P(1) P(2) P(1)+P(3) P(2)+h];
end

% =====================================================================
%  제어기 상자들 (윗줄) — XY_2_pen_ros2 와 공유
% =====================================================================
function addRosIn(m, P)
%  ROSIN — 실기 토픽 구독. 제어기는 이 네 신호로만 로봇·스테이지 상태를 안다
    imu = 'imu_quat';  fimu = {'orientation.w','orientation.x','orientation.y','orientation.z'};
    if ver2()   % 2 판: 요 각속도 PI 의 되먹임으로 자이로 z 도 읽는다 (실기 ang_vel_z)
        imu = 'imu_data';  fimu = [fimu, {'angular_velocity.z'}];
    end
    ss = add_subsys(m, 'RosIn', boxpos(P,4), {}, ...
                    {'zerokey_position','zerokey_vel',imu,'stage_pos_meas'}, gnc_colour('ros'));
    add_sub(ss, 'SubZk',   '/zerokey/position', 'geometry_msgs/PointStamped', {'point.x','point.y','point.z'}, 60,  'zerokey_position');
    add_sub(ss, 'SubZkV',  '/zerokey/vel',      'geometry_msgs/TwistStamped', {'twist.linear.x','twist.linear.y','twist.linear.z'}, 300, 'zerokey_vel');
    add_sub(ss, 'SubImu',  '/imu/data',         'sensor_msgs/Imu', fimu, 540, imu);
    add_sub(ss, 'SubStg',  '/stage/pos',        'geometry_msgs/PointStamped', {'point.x','point.y'}, 820, 'stage_pos_meas');
end

function addNav(m, P)
%  NAV — ZeroKey 월드 좌표 + IMU 쿼터니언 -> 측정 자세 pose = [p1; p2; psi]
    if ver2(), addNav2(m, P); return; end
    ss = add_subsys(m, 'Nav', boxpos(P,3), {'zerokey_position','zerokey_vel','imu_quat'}, {'pose'}, gnc_colour('ros'));
    code = {
        'function pose = NavFcn(zerokey_position, zerokey_vel, imu_quat, psi0, tau_pred)'
        '% NavFcn : 센서 토픽 -> 측정 자세 (항법)'
        '% NavFcn : sensor topics -> measured pose (navigation)'
        '% 입력 / Inputs'
        '%   zerokey_position [3x1, m]   /zerokey/position point.(x,y,z). 태그 월드 좌표: x 벽 수평(왼쪽 +), y 벽 법선, z 높이'
        '%                               tag world position: x horizontal (left +), y wall normal, z height'
        '%   zerokey_vel      [3x1, m/s] /zerokey/vel twist.linear.(x,y,z). 태그 월드 속도 / tag world velocity'
        '%   imu_quat         [4x1, -]   /imu/data orientation (w,x,y,z). Xsens 자세 쿼터니언 / Xsens attitude quaternion'
        '%   psi0             [1, rad]   출발 heading (천장 방향 = 0) / initial heading (0 = toward the ceiling)'
        '%   tau_pred         [1, s]     ZeroKey 지연 보상 시간 (0 = 끔) / ZeroKey latency compensation (0 = off)'
        '% 출력 / Output'
        '%   pose [3x1: m, m, rad]  [p1; p2; psi] = 태그 벽면 위치 (p1 높이, p2 수평) + heading'
        '%                          [p1; p2; psi] = tag position on the wall (p1 height, p2 horizontal) + heading'
        '% 원리 / Principle'
        '%   벽면 좌표는 p1 = z, p2 = x (실기 plane_project 와 같은 규약). 철판 벽에서는 자력계를 못 쓰므로'
        '%   heading 은 절대 방위가 아니라 첫 유효 샘플의 요각을 0 으로 잡은 상대 요각에 psi0 를 더한 값이다.'
        '%   Wall coordinates p1 = z, p2 = x (same convention as plane_project). The steel wall disturbs the'
        '%   magnetometer, so heading is the yaw relative to the first valid sample, plus psi0.'
        'persistent yaw0 ok'
        'if isempty(ok), ok = false; yaw0 = 0; end'
        'qw = imu_quat(1); qx = imu_quat(2); qy = imu_quat(3); qz = imu_quat(4);'
        'yaw = atan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz));'
        'if ~ok && (abs(qw) + abs(qx) + abs(qy) + abs(qz)) > 0.5'
        '    yaw0 = yaw;  ok = true;'
        'end'
        'a = yaw - yaw0 + psi0;'
        'pose = [zerokey_position(3); zerokey_position(1); atan2(sin(a), cos(a))];'
        '% ZeroKey 지연 보상 (tau_pred = 0 이면 끔): 측정 속도로 tau_pred 만큼 앞으로 내다본다'
        'pose(1:2) = pose(1:2) + tau_pred * [zerokey_vel(3); zerokey_vel(1)];'};
    fcn_box(ss, 'NavFcn', code, {'in:zerokey_position','in:zerokey_vel','in:imu_quat','const:psi0','const:tau_pred'}, {'pose'});
    set_disc(ss, 'NavFcn', 'Ts_ctrl');
    set_param([ss '/NavFcn'], 'Description', '이산: persistent (첫 요각 기억) — 구독 주기와 같다');
end

function addMission(m, P)
%  MISSION — 6WP_sim 과 같은 구조: Stateflow Chart 가 모드를 정하고, 모드에 따라 waypoint 를 준다
%    Guards     : 전이 조건에 쓸 거리·각 오차 (차트 입력)
%    ModeChart  : Goto_Start -> Turning -> Line_Tracking -> Stop_Destination -> Done
%    Waypoints  : 모드에 따라 로봇이 따라갈 선분 (6WP 의 waypoint_manager 역할)
    ss = add_subsys(m, 'Mission', boxpos(P,5), {'pose'}, ...
                    {'WP_prev','WP_target','mode','t_line','lineAB'}, gnc_colour('mission'));
    geo = {'const:pen_A','const:pen_B','const:pen_d','const:line_pre','const:line_post','const:r_tag'};
    line_geo = {
        'th = atan2(pen_B(2) - pen_A(2), pen_B(1) - pen_A(1));   % 펜 직선 방향'
        'u  = [cos(th); sin(th)];'
        'R  = [cos(th) -sin(th); sin(th) cos(th)];'
        '% 로봇(태그)이 따라갈 선 = 펜 직선을 R*pen_d 만큼 옮긴 것 (스테이지 중앙일 때 펜이 직선 위)'
        'rA = pen_A - R*pen_d - u*line_pre;    % 로봇 출발점 (펜이 A 에 닿기 line_pre 전)'
        'rB = pen_B - R*pen_d + u*line_post;   % 로봇 끝점'
        '% 제자리 회전은 중심 둘레로 돈다: 회전 뒤 태그가 rA 에 오려면 중심이 cA 에 있어야 한다'
        '% In-place turns pivot about the centre: for the tag to end at rA the centre must be at cA'
        'cA = rA - R*r_tag;'};
    code = [{
        'function [dist_start, th_err, along_rem] = Guards(pose, pen_A, pen_B, pen_d, line_pre, line_post, r_tag)'
        '% Guards : Stateflow 전이 조건에 쓸 거리·각 (실기 Chart 의 dist, th_err, dist_to_WP2 와 같은 뜻)'
        '% Guards : distances and angle used by the Stateflow transitions (dist, th_err, dist_to_WP2 of the real Chart)'
        '% 입력 / Inputs'
        '%   pose         [3x1: m, m, rad] 측정 태그 자세 [p1; p2; psi] / measured tag pose'
        '%   pen_A, pen_B [2x1, m]   펜이 그릴 직선의 시작·끝 (벽면) / start and end of the pen line (wall)'
        '%   pen_d        [2x1, m]   태그 -> 펜 레버암, 바디 좌표, 스테이지 중앙일 때 / tag-to-pen lever arm, body frame, stage centred'
        '%   line_pre, line_post [m] 직선 앞뒤로 로봇이 더 가는 거리 / run-in and run-out lengths of the robot segment'
        '%   r_tag        [2x1, m]   로봇 중심 -> 태그, 바디 좌표 / robot centre to tag, body frame'
        '% 출력 / Outputs'
        '%   dist_start [m]   추정 중심과 출발 중심 cA 의 거리 (모드 1 -> 2) / estimated centre to start centre cA (mode 1 -> 2)'
        '%   th_err     [rad] 직선 방향 - heading, [-pi, pi] (모드 2 -> 3) / line direction minus heading (mode 2 -> 3)'
        '%   along_rem  [m]   태그에서 로봇 끝점 rB 까지 남은 진행 거리 (모드 3 -> 4) / distance left to rB along the line (mode 3 -> 4)'}; line_geo; {
        'p = pose(1:2);'
        'Rp = [cos(pose(3)) -sin(pose(3)); sin(pose(3)) cos(pose(3))];'
        'dist_start = norm(cA - (p - Rp*r_tag));                        % 중심과 출발 중심 cA 의 거리'
        'th_err     = atan2(sin(th - pose(3)), cos(th - pose(3)));      % 직선 방향 오차'
        'along_rem  = dot(rB - p, u);                                   % 끝점까지 남은 진행 거리'}];
    fcn_box(ss, 'Guards', code, [{'in:pose'}, geo], {'tag:dist_start','tag:th_err','tag:along_rem'}, 330, 60);

    make_chart(ss, 'ModeChart', 1100, 60);

    code = [{
        'function [WP_prev, WP_target, lineAB] = Waypoints(mode, pen_A, pen_B, pen_d, line_pre, line_post, r_tag)'
        '% Waypoints : 모드에 따라 로봇이 따라갈 선분을 낸다 (6WP waypoint_manager 역할)'
        '% Waypoints : gives the segment the robot follows in each mode (role of the 6WP waypoint_manager)'
        '% 입력 / Inputs'
        '%   mode [-]  1 출발점 이동, 2 회전, 3 직선 추종, 4 정지, 5 종료 / 1 go to start, 2 turn, 3 line, 4 stop, 5 done'
        '%   pen_A, pen_B, pen_d, line_pre, line_post, r_tag : Guards 와 같음 / same as Guards'
        '% 출력 / Outputs'
        '%   WP_prev   [2x1, m] 태그 선분 시작 rA (펜이 A 에 닿기 line_pre 전) / tag segment start rA'
        '%   WP_target [2x1, m] 모드 1: 출발 중심 cA (중심 기준), 그 뒤: 태그 선분 끝 rB'
        '%                      mode 1: start centre cA (centre-referenced); afterwards: tag segment end rB'
        '%   lineAB    [4x1, m] [A; B] 펜 직선 (PenComp 로 간다) / pen line, sent to PenComp'
        '% 원리 / Principle'
        '%   스테이지가 중앙일 때 펜이 직선 위에 오도록, 태그가 갈 선분은 펜 직선을 R(theta_l) pen_d 만큼 옮긴 것이다.'
        '%   The tag segment is the pen line shifted by -R(theta_l) pen_d, so that the pen is on the line when the stage is centred.'}; line_geo; {
        'WP_prev = rA;'
        'if round(mode) <= 1'
        '    WP_target = cA;'
        'else'
        '    WP_target = rB;'
        'end'
        'lineAB = [pen_A; pen_B];'}];
    fcn_box(ss, 'Waypoints', code, [{'from:mode_c'}, geo], {'WP_prev','WP_target','lineAB'}, 330, 560);

    %  모드 출력: 차트의 mode_c 를 바깥 포트로
    q = port_xy(ss, 'Waypoints', 'Outport', 1);
    add_block('simulink/Signal Routing/From', [ss '/Fr_mode_out'], 'GotoTag','mode_c', ...
              'Position', [1500 q(2)+200 1580 q(2)+222]);
    set_param([ss '/mode'], 'Position', [1700 q(2)+201 1730 q(2)+221]);
    add_line(ss, 'Fr_mode_out/1', 'mode/1');
end

function make_chart(ss, name, X, Y0)
%  6WP_sim 의 Chart 와 같은 4 모드 (+ 끝 표시). 전이 조건은 실기 Tracked_UGV_ZeroKey_only 의 값
    add_block('sflib/Chart', [ss '/' name], 'Position', [X Y0 X+220 Y0+300]);
    rt = sfroot;
    ch = rt.find('-isa','Stateflow.Chart', 'Path', [ss '/' name]);
    ch.ActionLanguage = 'MATLAB';
    ch.ChartUpdate = 'DISCRETE';
    ch.SampleTime  = 'Ts_ctrl';
    ins = {'dist_start','th_err','along_rem','t'};
    for k = 1:numel(ins)
        d = Stateflow.Data(ch);  d.Name = ins{k};  d.Scope = 'Input';
    end
    outs = {'mode','t_line','done'};  init = {'1','0','0'};
    for k = 1:numel(outs)
        d = Stateflow.Data(ch);  d.Name = outs{k};  d.Scope = 'Output';
        d.DataType = 'double';   d.Props.InitialValue = init{k};
    end
    for p = {'d_reach','th_tol'}
        d = Stateflow.Data(ch);  d.Name = p{1};  d.Scope = 'Parameter';
    end
    names = {'Goto_Start','Turning','Line_Tracking','Stop_Destination','Done'};
    acts  = {'mode = 1;', 'mode = 2;', sprintf('mode = 3;\nt_line = t;'), 'mode = 4;', 'done = 1;'};
    S = cell(1,5);
    for k = 1:5
        S{k} = Stateflow.State(ch);
        S{k}.LabelString = sprintf('%s\nentry:\n%s', names{k}, acts{k});
        S{k}.Position = [30 + (k-1)*230, 80, 170, 110];
    end
    t0 = Stateflow.Transition(ch);
    t0.Destination = S{1};  t0.DestinationOClock = 0;
    t0.SourceEndpoint = [S{1}.Position(1) + 85, 40];
    t0.MidPoint = [S{1}.Position(1) + 85, 60];
    conds = {'[dist_start < d_reach]', ...
             'after(1.0,sec)[abs(th_err) < th_tol]', ...
             '[along_rem < d_reach]', ...
             'after(2.0,sec)'};
    for k = 1:4
        tr = Stateflow.Transition(ch);
        tr.Source = S{k};  tr.Destination = S{k+1};
        tr.SourceOClock = 3;  tr.DestinationOClock = 9;
        tr.LabelString = conds{k};
    end
    wire_box(ss, name, {'from:dist_start','from:th_err','from:along_rem','clock'}, ...
             {'tag:mode_c','t_line','stop:done'}, X, 220);
    set_param([ss '/' name], 'Description', ...
        '이산: 모드는 사건 상태 (6WP_sim Chart 와 같은 역할). 전이: 거리 < d_reach, 1 s 뒤 각 오차 < th_tol, 남은 거리 < d_reach, 정지 2 s');
end

function addRobotCtrl(m, P)
%  ROBOTCTRL — 6WP_sim 과 같은 구조: 모드별 Enabled Subsystem 4 개 + Merge
%    각 모드 상자 안 MATLAB Function 은 6WP / 실기 모델의 함수와 같은 식이다
    ss = add_subsys(m, 'RobotCtrl', boxpos(P,6), ...
                    {'WP_prev','WP_target','mode','t_line','pose','stage_pos_meas'}, ...
                    {'v_cmd','w_cmd','e_cross'}, gnc_colour('control'));
    fan_inports(ss);
    modes = {
      'M1_GotoStart', {'WP_target','pose'}, {'v','w'}, ...
        {'function [v, w] = Goto_Start_Position(WP_target, pose, K_dist, K_yaw, r_tag)'
         '% 모드 1 : 6WP_sim Goto_Start_Position, Method 2 (리아프노프형). 6WP 와 같이 로봇 중심을 목표로 보낸다'
         '% Mode 1 : 6WP_sim Goto_Start_Position, Method 2. Drives the robot centre (as in 6WP_sim) to the target'
         '% 입력 / Inputs'
         '%   WP_target [2x1, m]   출발 중심 cA (벽면) / start centre cA (wall)'
         '%   pose      [3x1: m, m, rad] 측정 태그 자세 / measured tag pose'
         '%   K_dist    [1/s]  거리 게인 / distance gain;   K_yaw [1/s]  방향 게인 / bearing gain'
         '%   r_tag     [2x1, m]  중심 -> 태그 (바디) / centre to tag (body)'
         '% 출력 / Outputs'
         '%   v [m/s]   전진 속도 명령 / forward speed command;   w [rad/s] 요 각속도 명령 (반시계 +) / yaw-rate command (CCW +)'
         '% 원리 / Principle'
         '%   v = K_dist rho max(0, cos a), w = K_yaw a + 0.5 sin a cos a. a = 목표 방위 - heading.'
         '%   방향이 90 도 넘게 틀리면 서서 돌기만 한다 / turns on the spot while the bearing error exceeds 90 deg'
         'c = pose(1:2) - [cos(pose(3)) -sin(pose(3)); sin(pose(3)) cos(pose(3))] * r_tag;   % 중심 추정'
         'err_x = WP_target(1) - c(1);  err_y = WP_target(2) - c(2);'
         'dist = hypot(err_x, err_y);'
         'target_th = atan2(err_y, err_x);'
         'a = atan2(sin(target_th - pose(3)), cos(target_th - pose(3)));'
         'v = K_dist * dist * max(0, cos(a));            % 방향이 맞을 때만 전진'
         'w = K_yaw * a + 0.5 * sin(a) * cos(a);         % 비선형 감쇠항'}, ...
        {'const:K_dist','const:K_yaw','const:r_tag'};
      'M2_Turn', {'WP_prev','WP_target','pose'}, {'v','w'}, ...
        {'function [v, w] = Turning_Straight_Mode(WP_prev, WP_target, pose, K_yaw)'
         '% 모드 2 : 6WP_sim Turning_Straight_Mode — 제자리 회전으로 heading 을 직선 방향에 맞춘다'
         '% Mode 2 : 6WP_sim Turning_Straight_Mode — turns on the spot until the heading matches the line'
         '% 입력 / Inputs'
         '%   WP_prev, WP_target [2x1, m] 태그 선분 rA, rB / tag segment;  pose [m, m, rad];  K_yaw [1/s]'
         '% 출력 / Outputs'
         '%   v = 0 [m/s],  w = K_yaw wrap(theta_l - psi) [rad/s]'
         'th_final = atan2(WP_target(2) - WP_prev(2), WP_target(1) - WP_prev(1));'
         'th_err = atan2(sin(th_final - pose(3)), cos(th_final - pose(3)));'
         'v = 0;'
         'w = K_yaw * th_err;'}, ...
        {'const:K_yaw'};
      'M3_Line', {'WP_prev','WP_target','t_line','pose','stage_pos_meas'}, {'v','w','e_cross'}, ...
        {'function [v, w, e_cross] = Line_Tracking_Mode(WP_prev, WP_target, t_line, pose, stage_pos_meas, t, v_ref, K_px, K_py, K_ptheta, K_c, K_ci, stage_c)'
         '% 모드 3 : 6WP_sim Line_Tracking_Mode — 시간에 따라 앞서 가는 기준점 + 바디 좌표 오차 (게인은 실기 ZeroKey 모델 값)'
         '% Mode 3 : 6WP_sim Line_Tracking_Mode — time-parameterised reference point + body-frame error (real ZeroKey model gains)'
         '% 입력 / Inputs'
         '%   WP_prev, WP_target [2x1, m]  태그 선분 rA -> rB / tag segment'
         '%   t_line [s]  모드 3 진입 시각 / time mode 3 started;   t [s] 현재 시각 / current time'
         '%   pose   [3x1: m, m, rad]  측정 태그 자세 / measured tag pose'
         '%   stage_pos_meas [2x1, mm] /stage/pos (K_c = K_ci = 0 이면 쓰지 않음 / unused when K_c = K_ci = 0)'
         '%   v_ref [m/s] 직선 속도 / line speed;  K_px [1/s];  K_py [rad/(m s)];  K_ptheta [1/s]'
         '%   K_c [-] 스테이지 중앙 유지 비례 / stage-centring P;  K_ci [1/s] 그 적분 / its integral;  stage_c [mm] 스테이지 중앙 / stage centre'
         '% 출력 / Outputs'
         '%   v [m/s], w [rad/s]  속도 명령 / speed commands;   e_cross [m]  바디 좌표 횡오차 e_y / body-frame cross error'
         '% 원리 / Principle'
         '%   x_r = rA + v_ref (t - t_line) u,  [e_x; e_y] = R(psi)^T (x_r - p),  e_th = theta_l - psi'
         '%   v = v_ref cos e_th + K_px e_x,  w = K_py e_y + K_ptheta sin e_th'
         '%   로봇은 옆으로 못 가므로 횡오차 e_y 는 회전(w)으로만 줄인다 / a tracked robot cannot slide sideways, so e_y is removed by turning'
         'th_final = atan2(WP_target(2) - WP_prev(2), WP_target(1) - WP_prev(1));'
         'dist_total = hypot(WP_target(1) - WP_prev(1), WP_target(2) - WP_prev(2));'
         'd_traj = min(v_ref * (t - t_line), dist_total);'
         'x_r = WP_prev(1) + d_traj * cos(th_final);'
         'y_r = WP_prev(2) + d_traj * sin(th_final);'
         'if d_traj < dist_total, vr = v_ref; else, vr = 0; end'
         'theta = pose(3);'
         'err_g = [x_r - pose(1); y_r - pose(2); th_final - theta];'
         'R_mat = [cos(theta) sin(theta) 0; -sin(theta) cos(theta) 0; 0 0 1];'
         'ep = R_mat * err_g;'
         'eth = atan2(sin(ep(3)), cos(ep(3)));'
         'e_cross = ep(2);'
         'v = vr * cos(eth) + K_px * ep(1);'
         '% 스테이지 중앙 유지 (K_c = 0 이면 끔): 스테이지 Y 가 중앙에서 벗어난 만큼을 횡오차에 더해'
         '% 로봇이 펜 쪽으로 다가가게 한다 -> 스테이지가 행정 가운데에서 일한다'
         '%   비례(K_c) + 적분(K_ci): 적분이 벽면 미끄러짐으로 생기는 정상 편차를 없앤다'
         'persistent I t_prev'
         'if isempty(I), I = 0; t_prev = t; end'
         'e_s = (stage_pos_meas(2) - stage_c) / 1000;   % 스테이지 Y 편차 [m]'
         'I = I + K_ci * e_s * max(t - t_prev, 0);  t_prev = t;'
         'e_c = ep(2) + K_c * e_s + I;'
         'w = K_py * e_c + K_ptheta * sin(eth);'}, ...
        {'dclock','const:v_ref','const:K_px','const:K_py','const:K_ptheta','const:K_c','const:K_ci','const:stage_c'};
      'M4_Stop', {'WP_target','pose'}, {'v','w'}, ...
        {'function [v, w] = Stop_Destination_Mode(WP_target, pose, K_dist, K_yaw)'
         '% 모드 4 : 6WP_sim Stop_Destination_Mode — 끝점(WP_target) 으로 Goto 식을 계속 돌려 그 자리에 머문다'
         '% Mode 4 : 6WP_sim Stop_Destination_Mode — keeps running the go-to law on the end point to hold position'
         '% 입력 / Inputs : WP_target = rB [2x1, m], pose [m, m, rad], K_dist [1/s], K_yaw [1/s]'
         '% 출력 / Outputs: v [m/s], w [rad/s]  (끝점 10 mm 안에서는 w = 0 / w = 0 within 10 mm of the end point)'
         'err_x = WP_target(1) - pose(1);  err_y = WP_target(2) - pose(2);'
         'dist = hypot(err_x, err_y);'
         'target_th = atan2(err_y, err_x);'
         'a = atan2(sin(target_th - pose(3)), cos(target_th - pose(3)));'
         'v = K_dist * dist * max(0, cos(a));'
         'w = K_yaw * a + 0.5 * sin(a) * cos(a);'
         'if dist < 0.01, w = 0; end                     % 끝점 위에서는 방향을 돌리지 않는다'}, ...
        {'const:K_dist','const:K_yaw'}};
    Ybase = 60;  X = 700;
    for k = 1:4
        nm = modes{k,1};  ins = modes{k,2};  outs = modes{k,3};
        h = max(120, 60*max(numel(ins), numel(outs)) + 20);
        sub = add_subsys(ss, nm, [X Ybase X+180 Ybase+h], ins, outs, gnc_colour('control'));
        add_block('simulink/Ports & Subsystems/Enable', [sub '/Enable'], 'Position', [400 10 420 30]);
        pins = [cellfun(@(s) ['in:' s], ins, 'uni', 0), modes{k,5}];
        po = outs;  po{1} = 'sat:v:v_max';  po{2} = 'sat:w:w_max';
        fcn_box(sub, strrep(nm, 'M', 'Fcn'), modes{k,4}, pins, po);
        %  입력: 로컬 태그로 받는다
        for j = 1:numel(ins)
            b = port_xy(ss, nm, 'Inport', j);
            add_block('simulink/Signal Routing/From', [ss '/Fr_' ins{j} '_' num2str(k)], ...
                      'GotoTag', ['i_' ins{j}], 'Position', [X-210 b(2)-11 X-60 b(2)+11]);
            add_line(ss, ['Fr_' ins{j} '_' num2str(k) '/1'], sprintf('%s/%d', nm, j));
        end
        %  Enable: mode == k  (상자 바로 위)
        e = get_param([ss '/' nm], 'PortHandles');  ep = get_param(e.Enable, 'Position');
        add_block('simulink/Logic and Bit Operations/Compare To Constant', [ss '/IsMode' num2str(k)], ...
                  'const', num2str(k), 'relop', '==', 'Position', [ep(1)-25 ep(2)-55 ep(1)+25 ep(2)-30]);
        add_block('simulink/Signal Routing/From', [ss '/Fr_mode_' num2str(k)], 'GotoTag', 'i_mode', ...
                  'Position', [ep(1)-150 ep(2)-53 ep(1)-70 ep(2)-32]);
        add_line(ss, ['Fr_mode_' num2str(k) '/1'], ['IsMode' num2str(k) '/1']);
        add_line(ss, ['IsMode' num2str(k) '/1'], [nm '/Enable']);
        %  출력: 로컬 태그로 내보낸다 -> Merge
        for j = 1:numel(outs)
            q = port_xy(ss, nm, 'Outport', j);
            add_block('simulink/Signal Routing/Goto', [ss '/Go_' outs{j} num2str(k)], ...
                      'GotoTag', [outs{j} num2str(k)], 'TagVisibility','local', ...
                      'Position', [q(1)+30 q(2)-11 q(1)+90 q(2)+11]);
            add_line(ss, sprintf('%s/%d', nm, j), ['Go_' outs{j} num2str(k) '/1']);
        end
        Ybase = Ybase + h + 110;
    end
    %  Merge (6WP_sim 의 Merge 와 같은 역할 — 켜진 모드 상자의 출력만 지나간다)
    Xm = 1250;
    for v = {'v','w'}
        if strcmp(v{1},'v'), y0 = 100; outp = 'v_cmd'; else, y0 = 420; outp = 'w_cmd'; end
        mg = ['Merge_' v{1}];
        add_block('simulink/Signal Routing/Merge', [ss '/' mg], 'Inputs','4', ...
                  'Position', [Xm y0 Xm+40 y0+240]);
        for k = 1:4
            b = port_xy(ss, mg, 'Inport', k);
            add_block('simulink/Signal Routing/From', [ss '/Fr_' v{1} num2str(k)], 'GotoTag', [v{1} num2str(k)], ...
                      'Position', [Xm-120 b(2)-11 Xm-60 b(2)+11]);
            add_line(ss, ['Fr_' v{1} num2str(k) '/1'], sprintf('%s/%d', mg, k));
        end
        q = port_xy(ss, mg, 'Outport', 1);
        set_param([ss '/' outp], 'Position', [Xm+150 q(2)-10 Xm+180 q(2)+10]);
        add_line(ss, [mg '/1'], [outp '/1']);
    end
    add_block('simulink/Signal Routing/From', [ss '/Fr_e_cross'], 'GotoTag','e_cross3', ...
              'Position', [Xm-120 789 Xm-40 811]);
    set_param([ss '/e_cross'], 'Position', [Xm+150 790 Xm+180 810]);
    add_line(ss, 'Fr_e_cross/1', 'e_cross/1');
end

function addNav2(m, P)
%  NAV (2 판) — pose 에 더해 내부 PI 되먹임 V_meas = [u_body; r] 를 낸다
%    u_body : ZeroKey 평면 속도를 heading 으로 바디 좌표에 옮긴 전진 속도 (실기 MATLAB Function14 의 u)
%    r      : IMU 자이로 z [rad/s] (실기 ang_vel_z)
%  Nav (v2): also outputs V_meas = [u_body; r] for the inner PI loops, as in the real Xsens model.
    ss = add_subsys(m, 'Nav', boxpos(P,3), {'zerokey_position','zerokey_vel','imu_data'}, {'pose','V_meas'}, gnc_colour('ros'));
    code = {
        'function [pose, V_meas] = NavFcn(zerokey_position, zerokey_vel, imu_data, psi0, tau_pred)'
        '% NavFcn (2 판) : 1 판 NavFcn 에 내부 PI 되먹임 V_meas 를 더한 것'
        '% NavFcn (v2)   : v1 NavFcn plus the inner-PI feedback V_meas'
        '% 입력 / Inputs'
        '%   zerokey_position [3x1, m], zerokey_vel [3x1, m/s], psi0 [rad], tau_pred [s] : 1 판과 같음 / as in v1'
        '%   imu_data [5x1]  [qw; qx; qy; qz (-); 자이로 z (rad/s)] = /imu/data orientation + angular_velocity.z'
        '% 출력 / Outputs'
        '%   pose   [3x1: m, m, rad]  [p1; p2; psi]'
        '%   V_meas [2x1: m/s, rad/s] [u_body; r] = ZeroKey 속도의 바디 전진 성분, 자이로 요 각속도'
        '%                            forward body speed from the ZeroKey velocity, gyro yaw rate'
        '% 원리 / Principle'
        '%   u_body = x_B^T p_dot = p1_dot cos psi + p2_dot sin psi (실기 transform_state 의 u)'
        'persistent yaw0 ok'
        'if isempty(ok), ok = false; yaw0 = 0; end'
        'qw = imu_data(1); qx = imu_data(2); qy = imu_data(3); qz = imu_data(4);'
        'yaw = atan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz));'
        'if ~ok && (abs(qw) + abs(qx) + abs(qy) + abs(qz)) > 0.5'
        '    yaw0 = yaw;  ok = true;'
        'end'
        'a = yaw - yaw0 + psi0;'
        'psi = atan2(sin(a), cos(a));'
        'pose = [zerokey_position(3); zerokey_position(1); psi];'
        'pose(1:2) = pose(1:2) + tau_pred * [zerokey_vel(3); zerokey_vel(1)];'
        '% 바디 전진 속도 (실기 transform_state 의 u 와 같은 식)'
        'p1d = zerokey_vel(3);  p2d = zerokey_vel(1);'
        'u_body = p1d * cos(psi) + p2d * sin(psi);'
        'V_meas = [u_body; imu_data(5)];'};
    fcn_box(ss, 'NavFcn', code, {'in:zerokey_position','in:zerokey_vel','in:imu_data','const:psi0','const:tau_pred'}, {'pose','V_meas'});
    set_disc(ss, 'NavFcn', 'Ts_ctrl');
    set_param([ss '/NavFcn'], 'Description', '이산: persistent (첫 요각 기억) — 구독 주기와 같다');
end

function addInnerPI(m, P)
%  INNERPI (2 판) — 실기 Tracked_UGV_ZeroKey_XsensIMU 의 PID Controller2 (속도) · 3 (요 각속도) 와 같은 PI
%    e_v = v_cmd - u_body           -> PI (Kin_Pv, Kin_Iv), 출력 [0, 0.5]   -> V
%    e_w = w_cmd - w_fb_gain * r    -> PI (Kin_Pw, Kin_Iw), 출력 [-0.5, 0.5] -> W
%    실기와 같이 이산 (Forward Euler, Ts_ctrl), 적분 clamping. 앞먹임 없음.
%    w_fb_gain : 실기 모델 Multiply7 의 값 0.1443 을 기본으로 둔다 (단위가 맞으려면 1). XY_setup 10 절
%  Inner PI (v2): the two discrete PI loops of the real Xsens model, same gains, limits and feedback.
%    inner_reset = 1 : 모드가 바뀌는 순간 두 적분기를 0 으로 (실기 모델에는 없음, XY 2 판에서 더함).
%      속도 PI 는 출력 하한이 0 이고 앞먹임이 없어, 모드 1 -> 2 에서 v_cmd 가 0 이 되어도 적분값이
%      남아 회전하는 동안 로봇이 계속 전진한다 (시뮬레이션: 0.05–0.1 m/s, 직선 시작점 100 mm 어긋남)
    ss = add_subsys(m, 'InnerPI', boxpos(P,4), {'v_cmd','w_cmd','V_meas','mode'}, {'V_bar','W_bar'}, gnc_colour('control'));
    fcn_box(ss, 'SpeedErr', {
        'function [e_v, e_w, rst] = SpeedErr(v_cmd, w_cmd, V_meas, mode, w_fb_gain, inner_reset)'
        '% SpeedErr (2 판 InnerPI) : 내부 PI 두 개의 오차와 적분기 리셋 신호'
        '% SpeedErr (v2 InnerPI)   : errors of the two inner PI loops and the integrator reset'
        '% 입력 / Inputs'
        '%   v_cmd [m/s], w_cmd [rad/s]  외부 루프 명령 / outer-loop commands'
        '%   V_meas [2x1: m/s, rad/s]    [u_body; r] (Nav)'
        '%   mode [-]  미션 모드 / mission mode;  w_fb_gain [-] 자이로 배율 (실기 0.1443, 여기 1) / gyro scale'
        '%   inner_reset [0/1]  1 이면 모드 전환 때 적분기 리셋 / 1 resets the integrators on a mode change'
        '% 출력 / Outputs'
        '%   e_v [m/s] = v_cmd - u_body,  e_w [rad/s] = w_cmd - w_fb_gain r,  rst [0/1]'
        '% 실기 Sum3 = V_cmd(1) - u,   Sum4 = V_cmd(2) - 0.1443 * ang_vel_z'
        'persistent m_prev'
        'if isempty(m_prev), m_prev = mode; end'
        'e_v = v_cmd - V_meas(1);'
        'e_w = w_cmd - w_fb_gain * V_meas(2);'
        '% 모드가 바뀐 샘플에서만 1 -> PI 적분기 리셋 (inner_reset = 0 이면 실기와 같이 리셋 없음)'
        'rst = double(inner_reset ~= 0 && mode ~= m_prev);'
        'm_prev = mode;'}, ...
        {'in:v_cmd','in:w_cmd','in:V_meas','in:mode','const:w_fb_gain','const:inner_reset'}, {'tag:e_v','tag:e_w','tag:rst'}, 330, 60);
    set_disc(ss, 'SpeedErr', 'Ts_ctrl');
    set_param([ss '/SpeedErr'], 'Description', '이산: persistent (이전 모드 기억) — 제어 주기와 같다');
    pid = {'TimeDomain','Discrete-time', 'SampleTime','Ts_ctrl', 'IntegratorMethod','Forward Euler', ...
           'Form','Parallel', 'D','0', 'LimitOutput','on', 'AntiWindupMode','clamping', 'ExternalReset','level'};
    blk_box(ss, 'simulink/Continuous/PID Controller', 'PI_v', ...
        [pid, {'P','Kin_Pv', 'I','Kin_Iv', 'UpperSaturationLimit','0.5', 'LowerSaturationLimit','0'}], ...
        {'from:e_v','from:rst'}, {'V_bar'}, 800, 60, 100, 60);
    blk_box(ss, 'simulink/Continuous/PID Controller', 'PI_w', ...
        [pid, {'P','Kin_Pw', 'I','Kin_Iw', 'UpperSaturationLimit','0.5', 'LowerSaturationLimit','-0.5'}], ...
        {'from:e_w','from:rst'}, {'W_bar'}, 800, 220, 100, 60);
    for b = {'PI_v','PI_w'}
        set_param([ss '/' b{1}], 'Description', '이산: 실기 모델의 PID Controller2·3 과 같은 이산 PI (Forward Euler, 0.01 s). 리셋 입력은 2 판에서 더함');
    end
end

function addPwmMap(m, P)
%  PWMMAP — (v, w) -> /motor_pwm [left; right]. 실기 calc_pwm 과 같은 식 (데드밴드 역보상 포함)
    if ver2(), addPwmMap2(m, P); return; end
    ss = add_subsys(m, 'PwmMap', boxpos(P,2), {'v_cmd','w_cmd'}, {'motor_pwm'}, gnc_colour('allocation'));
    code = {
        'function motor_pwm = calc_pwm(v_cmd, w_cmd, v_full, w_full, db_v, db_w)'
        '% calc_pwm : 차체 속도 명령 -> 좌우 모터 PWM (실기 MATLAB Function6 calc_pwm 과 같은 식)'
        '% calc_pwm : body speed command -> left/right motor PWM (same as the real MATLAB Function6 calc_pwm)'
        '% 입력 / Inputs'
        '%   v_cmd  [m/s]    전진 속도 명령 (RobotCtrl) / forward speed command'
        '%   w_cmd  [rad/s]  요 각속도 명령, 반시계 (왼쪽으로 돎) + / yaw-rate command, CCW (turning left) +'
        '%   v_full [m/s]    throttle 400 count 일 때 궤도 속도 (0.20) / track speed at 400 counts of throttle'
        '%   w_full [rad/s]  steer 400 count 일 때 요 각속도 (1.37) / yaw rate at 400 counts of steer'
        '%   db_v, db_w [count]  ESC 데드밴드 역보상 크기 (47, 20) / ESC dead-band pre-compensation'
        '% 출력 / Output'
        '%   motor_pwm [2x1, count]  [왼쪽; 오른쪽] 서보 펄스 폭 [us] 와 같은 수, 1500 = 정지, 1100-1900'
        '%                           [left; right] servo pulse width in microseconds, 1500 = stop, 1100-1900'
        '%                           -> /motor_pwm (geometry_msgs/Point x, y) -> Arduino Mega -> C620 ESC'
        '% 원리 / Principle'
        '%   1) 실기에서는 PID 출력 (-0.5 ~ 0.5, 단위 없음) 이 V, W 로 들어간다. 1 판에는 내부 PID 가 없으므로'
        '%      속도 명령을 같은 범위로 정규화: V = 0.5 v_cmd / v_full'
        '%      The real model feeds PID outputs (-0.5 to 0.5, unitless) as V, W. v1 has no inner PID, so the'
        '%      speed commands are normalised to the same range: V = 0.5 v_cmd / v_full'
        '%   2) throttle = db tanh(V/0.02) + (400 - db) V/0.5 : 0 근처에서 db 만큼 뛰어 ESC 데드존을 건너뛴다'
        '%      throttle jumps by db near zero to step over the ESC dead zone, then rises linearly to 400'
        '%   3) left = throttle - steer, right = throttle + steer (steer > 0 이면 오른쪽이 빨라 왼쪽으로 돈다)'
        '%   4) 오른쪽 모터는 거울 대칭으로 달려 있어 부호를 뒤집는다 / the right motor is mirrored, so its sign is inverted'
        'V = max(-0.5, min(0.5, 0.5 * v_cmd / v_full));'
        'W = max(-0.5, min(0.5, 0.5 * w_cmd / w_full));'
        'NEUTRAL = 1500;  MAX_OFFSET = 400;  V_EPS = 0.02;  W_EPS = 0.10;'
        'db_v_c = max(0, min(MAX_OFFSET, db_v));'
        'db_w_c = max(0, min(MAX_OFFSET, db_w));'
        'throttle = db_v_c * tanh(V / V_EPS) + (MAX_OFFSET - db_v_c) * (V / 0.5);'
        'steer    = db_w_c * tanh(W / W_EPS) + (MAX_OFFSET - db_w_c) * (W / 0.5);'
        'left_raw  = throttle - steer;'
        'right_raw = throttle + steer;'
        'right_inv = -right_raw;                         % 우측 모터 물리적 반전'
        'motor_pwm = [max(1100, min(1900, NEUTRAL + left_raw));'
        '             max(1100, min(1900, NEUTRAL + right_inv))];'};
    fcn_box(ss, 'calc_pwm', code, {'in:v_cmd','in:w_cmd','const:v_full','const:w_full','const:db_v','const:db_w'}, {'motor_pwm'});
end

function addPwmMap2(m, P)
%  PWMMAP (2 판) — PI 출력 V, W (-0.5 ~ 0.5) -> /motor_pwm. 실기 MATLAB Function6 calc_pwm 그대로
    ss = add_subsys(m, 'PwmMap', boxpos(P,2), {'V_bar','W_bar'}, {'motor_pwm'}, gnc_colour('allocation'));
    code = {
        'function motor_pwm = calc_pwm(V, W, db_v, db_w)'
        '% calc_pwm (2 판) : 실기 MATLAB Function6 calc_pwm 과 같은 식 (throttle_sign = steer_sign = 1)'
        '% calc_pwm (v2)   : identical to the real MATLAB Function6 calc_pwm'
        '% 입력 / Inputs'
        '%   V [-]  속도 PI 출력 0 ~ 0.5 (단위 없음, 0.5 = throttle 400 count) / speed PI output, unitless'
        '%   W [-]  요 각속도 PI 출력 -0.5 ~ 0.5 (0.5 = steer 400 count) / yaw-rate PI output, unitless'
        '%   db_v, db_w [count]  ESC 데드밴드 역보상 / ESC dead-band pre-compensation'
        '% 출력 / Output'
        '%   motor_pwm [2x1, count]  [왼쪽; 오른쪽], 1500 = 정지, 1100-1900 / [left; right], 1500 = stop'
        '% 원리는 1 판 calc_pwm 머리말과 같음 / principle as in the v1 calc_pwm header'
        'NEUTRAL = 1500;  MAX_OFFSET = 400;  V_EPS = 0.02;  W_EPS = 0.10;'
        'db_v_c = max(0, min(MAX_OFFSET, db_v));'
        'db_w_c = max(0, min(MAX_OFFSET, db_w));'
        'throttle = db_v_c * tanh(V / V_EPS) + (MAX_OFFSET - db_v_c) * (V / 0.5);'
        'steer    = db_w_c * tanh(W / W_EPS) + (MAX_OFFSET - db_w_c) * (W / 0.5);'
        'left_raw  = throttle - steer;'
        'right_raw = throttle + steer;'
        'right_inv = -right_raw;                         % 우측 모터 물리적 반전'
        'motor_pwm = [max(1100, min(1900, NEUTRAL + left_raw));'
        '             max(1100, min(1900, NEUTRAL + right_inv))];'};
    fcn_box(ss, 'calc_pwm', code, {'in:V_bar','in:W_bar','const:db_v','const:db_w'}, {'motor_pwm'});
end

function addPenCtrl(m, P)
%  PENCTRL — 펜 기구학 보상. s_cmd = s_c + 1000 (R(psi)' (p_des - p_T) - d)   [mm]
%  출력 = 토픽 /stage/target_pos. Zaber 내부 위치 루프가 따라가므로 우리는 위치만 준다
    ss = add_subsys(m, 'PenCtrl', boxpos(P,3), {'lineAB','mode','pose'}, ...
                    {'stage_target_pos','pen_on'}, gnc_colour('control'));
    code = {
        'function [s_cmd, pen_on, sigma] = PenComp(lineAB, mode, pose, pen_d, stage_c)'
        '% PenComp : 펜 기구학 보상. 펜이 직선 위에 오도록 스테이지 위치 명령을 계산한다'
        '% PenComp : pen kinematic compensation. Computes the stage position command that puts the pen on the line'
        '% 입력 / Inputs'
        '%   lineAB  [4x1, m]   [A; B] 펜이 그릴 직선, 벽면 좌표 (p1 높이, p2 수평) / pen line in wall coordinates'
        '%   mode    [-]        미션 모드. 3 (직선 추종) 일 때만 보상 / mission mode; compensates only in mode 3'
        '%   pose    [3x1: m, m, rad]  측정 태그 자세 [p1; p2; psi] (Nav) / measured tag pose'
        '%   pen_d   [2x1, m]   태그 -> 펜 레버암, 바디 좌표 [x_B; y_B], 스테이지 중앙일 때 (= d)'
        '%                      tag-to-pen lever arm in body axes, with the stage centred (= d)'
        '%   stage_c [mm]       스테이지 중앙 (50.8, 두 축 같음) / stage centre (50.8 on both axes)'
        '% 출력 / Outputs'
        '%   s_cmd  [2x1, mm]   스테이지 목표 [X; Y] (X = x_B 전진 축, Y = y_B 왼쪽 축), 0-101.6'
        '%                      stage target [X; Y] (X along x_B forward, Y along y_B left) -> /stage/target_pos'
        '%   pen_on [0/1]       1 = 펜 다운 (펜 목표가 A 와 B 사이) / 1 = pen down (pen target between A and B)'
        '%   sigma  [m]         직선 위 진행 거리 (A 에서 0, B 에서 L) / distance travelled along the line'
        '% 원리 / Principle'
        '%   펜 = 태그 + R(psi) (pen_d + (s - s_c)/1000) 을 s 에 대해 푼다. /1000 은 mm -> m.'
        '%   Solve  pen = tag + R(psi) (pen_d + (s - s_c)/1000)  for s.  The 1000 converts m to mm.'
        '%   p_des : 스테이지 중앙일 때 펜 위치 c0 를 직선에 내린 수선의 발'
        '%           foot of the perpendicular from c0 (pen with the stage centred) to the line'
        '%   진행 방향은 로봇이 맡고, 스테이지는 직선과의 어긋남(횡오차 + heading 오차)만 지운다'
        '%   The robot provides the motion along the line; the stage removes only the offset from the line.'
        'A = lineAB(1:2);  B = lineAB(3:4);'
        'L = norm(B - A);  u = (B - A) / max(L, 1e-9);'
        'pT = pose(1:2);'
        'R  = [cos(pose(3)) -sin(pose(3)); sin(pose(3)) cos(pose(3))];'
        'pc = pT + R * pen_d;                 % 스테이지 중앙일 때 펜 위치'
        'sigma = dot(u, pc - A);              % 직선 위 진행 거리'
        'pen_on = 0;'
        's_cmd = [stage_c; stage_c];'
        'if round(mode) == 3'
        '    % 직선 구간에 들어서면 펜을 내리기 전부터 스테이지를 미리 보낸다 (시작 과도 제거)'
        '    % p_des 는 직선 위 수선의 발 (A, B 로 자르지 않는다): 진행 방향은 로봇이 맡고'
        '    % 스테이지는 직선에서 벗어난 만큼만 지운다. A 로 자르면 펜이 A 앞에 있을 때'
        '    % 남은 진행 거리(line_pre = 60 mm)까지 스테이지가 메우려 해서 행정 끝에 닿는다'
        '    p_des = A + sigma * u;'
        '    e_b = R'' * (p_des - pT) - pen_d;   % 바디 좌표 펜 이동량 [m]'
        '    s_cmd = stage_c + 1000 * e_b;'
        '    pen_on = double(sigma >= 0 && sigma <= L);'
        'end'};
    fcn_box(ss, 'PenComp', code, {'in:lineAB','in:mode','in:pose','const:pen_d','const:stage_c'}, ...
            {'stagecmd:stage_target_pos','pen_on','term:sigma'});
end

function addRosOut(m, P)
%  ROSOUT — 실기 토픽 발행 /motor_pwm, /stage/target_pos
    ss = add_subsys(m, 'RosOut', boxpos(P,2), {'motor_pwm','stage_target_pos'}, {}, gnc_colour('ros'));
    add_pub(ss, 'PubPwm', '/motor_pwm',        'geometry_msgs/Point', {'x','y'}, 60,  'motor_pwm', '');
    add_pub(ss, 'PubStg', '/stage/target_pos', 'geometry_msgs/Point', {'x','y'}, 300, 'stage_target_pos', '');
end

% =====================================================================
%  가상 실험장 (아랫줄) — XY_1_pen_sim 에만
% =====================================================================
function addPlantRosIn(m, P)
%  PLANTROSIN — 운동모델·스테이지 쪽이 제어기 명령을 구독한다 (실기의 아두이노·Zaber 노드 자리)
    ss = add_subsys(m, 'PlantRosIn', boxpos(P,2), {}, {'motor_pwm_rx','stage_target_rx'}, gnc_colour('ros'));
    add_sub(ss, 'SubPwm', '/motor_pwm',        'geometry_msgs/Point', {'x','y'}, 60,  'motor_pwm_rx');
    add_sub(ss, 'SubStg', '/stage/target_pos', 'geometry_msgs/Point', {'x','y'}, 300, 'stage_target_rx');
end

function addRobotPlant(m, P)
%  ROBOTPLANT — 6WP_sim 의 운동모델 체인을 그대로 쓴다
%    /motor_pwm -> ESC 역변환 -> 모터 시정수 -> 속도 PID (6WP PID Controller 두 개)
%    -> Control allocation -> 구동력 한계 -> Dynamics -> 적분 -> Kinematics -> 적분 -> q_true
    ss = add_subsys(m, 'RobotPlant', boxpos(P,2), {'motor_pwm_rx'}, {'q_true','V_true'}, gnc_colour('plant'));
    fan_inports(ss);
    %  1) ESC: PWM count -> 궤도 속도 명령
    fcn_box(ss, 'EscInv', {
        'function v_trk_cmd = EscInv(motor_pwm, pwm_dz, k_pwm)'
        '% EscInv : 가상 ESC. PWM -> 궤도 속도 명령 (calc_pwm 의 역) / virtual ESC: PWM -> track speed command'
        '% 입력 / Inputs'
        '%   motor_pwm [2x1, count] /motor_pwm [왼쪽; 오른쪽] / [left; right]'
        '%   pwm_dz [count] ESC 데드존 (30) / ESC dead zone;   k_pwm [m/s per count] 기울기 / slope'
        '% 출력 / Output'
        '%   v_trk_cmd [2x1, m/s]  [vL; vR] 궤도 속도 명령 / track speed commands'
        '% 원리: 우측 반전 해제 -> 데드존 -> count 를 속도로 / undo the right-motor inversion, dead zone, scale'
        'pL =  (motor_pwm(1) - 1500);'
        'pR = -(motor_pwm(2) - 1500);'
        'v_trk_cmd = k_pwm * [sign(pL) * max(0, abs(pL) - pwm_dz);'
        '                     sign(pR) * max(0, abs(pR) - pwm_dz)];'}, ...
        {'from:i_motor_pwm_rx','const:pwm_dz','const:k_pwm'}, {'tag:v_trk_cmd'}, 520, 60);
    %  2) ESC 램프 + 모터 시정수
    blk_box(ss, 'simulink/Continuous/State-Space', 'TrackLag', ...
        {'A','trk_A','B','trk_B','C','trk_C','D','trk_D','X0','[0;0]'}, ...
        {'from:v_trk_cmd'}, {'tag:v_trk'}, 520, 320, 120, 60);
    %  3) 속도 오차 (6WP: V_cmd - v_actual, 둘째 성분은 L*omega)
    fcn_box(ss, 'SpeedErr', {
        'function [e_v, e_Lw] = SpeedErr(v_trk, V_act, B_eff, L_r)'
        '% SpeedErr (운동모델) : 궤도 속도 명령 -> 차체 속도 오차. 6WP 와 같이 둘째 성분은 L*omega 단위'
        '% SpeedErr (plant)    : track speed command -> body speed error; 2nd component in L*omega units as in 6WP'
        '% 입력 / Inputs'
        '%   v_trk [2x1, m/s]  지연을 거친 궤도 속도 명령 [vL; vR] / lagged track speed commands'
        '%   V_act [2x1: m/s, rad/s]  실제 차체 속도 [v; omega] / actual body speed'
        '%   B_eff [m]  조향 유효 궤도 간격 / effective track spacing;   L_r [m]  관성 반경 / radius of gyration'
        '% 출력 / Outputs'
        '%   e_v [m/s] = (vL+vR)/2 - v,   e_Lw [m/s] = L_r ((vR-vL)/B_eff - omega)'
        'v_c = 0.5 * (v_trk(1) + v_trk(2));'
        'w_c = (v_trk(2) - v_trk(1)) / B_eff;'
        'e_v  = v_c - V_act(1);'
        'e_Lw = L_r * (w_c - V_act(2));'}, ...
        {'from:v_trk','from:V_act','const:B_eff','const:L_r'}, {'tag:e_v','tag:e_Lw'}, 1150, 60);
    %  4) 속도 PID (6WP_sim 의 PID Controller, PID Controller1)
    blk_box(ss, 'simulink/Continuous/PID Controller', 'PID_v', {'P','Kp_v','I','Ki_v','D','0'}, ...
        {'from:e_v'}, {'tag:tau_X'}, 1150, 360, 110, 60);
    blk_box(ss, 'simulink/Continuous/PID Controller', 'PID_w', {'P','Kp_w','I','Ki_w','D','0'}, ...
        {'from:e_Lw'}, {'tag:tau_N'}, 1150, 500, 110, 60);
    %  5) Control allocation (6WP 와 같은 식) + 구동력 한계
    fcn_box(ss, 'Control_allocation', {
        'function [Fr, Fl] = Control_allocation(tau_X, tau_N, b_trk, L_r)'
        '% Control_allocation : 차체 힘·모멘트 -> 좌우 궤도 구동력 (6WP_sim 과 같은 식)'
        '% Control_allocation : body force and moment -> right/left track forces (as in 6WP_sim)'
        '% 입력 / Inputs : tau_X [N] 전진 힘 / surge force;  tau_N [N] 요 모멘트/L / yaw moment divided by L'
        '%                 b_trk [m] 궤도 간격 / track spacing;  L_r [m]'
        '% 출력 / Outputs: Fr, Fl [N] 오른쪽·왼쪽 궤도 구동력 / right and left track forces'
        '% 원리: [tau_X; tau_N] = [1 1; b/2L -b/2L] [Fr; Fl] 를 푼다 / solve this 2x2 system'
        'B_tilde = [1, 1; b_trk/(2*L_r), -b_trk/(2*L_r)];'
        'F = B_tilde \ [tau_X; tau_N];'
        'Fr = F(1);  Fl = F(2);'}, ...
        {'from:tau_X','from:tau_N','const:b_trk','const:L_r'}, {'sattag:Fr:F_max','sattag:Fl:F_max'}, 1750, 60);
    %  6) ICR 이동량 u (6WP compute_u) 와 그 미분
    fcn_box(ss, 'compute_u', {
        'function u = compute_u(V_act, alpha_s)'
        '% compute_u (6WP_sim) : 회전 중심(ICR)의 길이방향 이동량 / longitudinal shift of the ICR'
        '% 입력 / Inputs : V_act [m/s, rad/s];  alpha_s [rad] ICR 이동각 (0 이면 u = 0) / ICR shift angle'
        '% 출력 / Output : u [m]'
        'R_turn = abs(V_act(1) / max(0.01, abs(V_act(2))));'
        'u = -sign(V_act(2)) * R_turn * sin(alpha_s);'}, ...
        {'from:V_act','const:alpha_s'}, {'tag:u'}, 1750, 400);
    blk_box(ss, 'simulink/Continuous/Derivative', 'Derivative_u', {}, {'from:u'}, {'tag:u_dot'}, 1750, 560, 60, 40);
    %  7) Dynamics (6WP 식, 벽면판)
    fcn_box(ss, 'Dynamics', {
        'function V_dot = Dynamics(q, V_act, u, u_dot, Fr, Fl, dyn_p)'
        '% Dynamics : 궤도차량 동역학 (Zou 2018), 벽면판 / tracked-vehicle dynamics (Zou 2018), wall version'
        '% 입력 / Inputs'
        '%   q [3x1: m, m, rad]  로봇 중심 자세 / robot centre pose;  V_act [2x1: m/s, rad/s] [v; omega]'
        '%   u [m], u_dot [m/s]  ICR 이동량과 그 미분 / ICR shift and its rate'
        '%   Fr, Fl [N]  궤도 구동력 / track forces'
        '%   dyn_p = [m kg; l m; b m; mu_l -; mu_t -; N_mag N; g m/s^2]'
        '% 출력 / Output : V_dot [2x1: m/s^2, rad/s^2]'
        '% 원리 / Principle : M~ nu_dot + C~ nu + S^T (f1 + f2) = B~ tau + S^T f_g,  nu = [v; L omega]'
        '% 6WP_sim Dynamics 와 같은 식 (궤도차량 동역학, 논문 Table 1 모델). 벽면판에서 바꾼 점:'
        '%  (1) 바닥 하중 W -> 자석 흡착력 N_mag : 벽에서 마찰을 만드는 수직력'
        '%  (2) 중력 f_g = [-m g; 0; 0] 추가    : 벽면 좌표 p1 = 위'
        '%  (3) 회전 저항 Mr 는 회전을 막는 쪽으로 (l^2/4 - u^2). 6WP 식 (u^2 - l^2/4) 은'
        '%      u = 0 근처에서 회전을 돕는 부호가 되어 헤딩 떨림(stick-slip)을 만든다'
        '%  (4) sign() -> tanh(./eps) : 정지 근처 수치 떨림 방지'
        'm = dyn_p(1); l = dyn_p(2); b = dyn_p(3); mu_l = dyn_p(4); mu_t = dyn_p(5); N = dyn_p(6); g = dyn_p(7);'
        'L = sqrt((l^2 + b^2)/12);'
        'B_tilde = [1, 1; b/(2*L), -b/(2*L)];'
        'B_tau = B_tilde * [Fr; Fl];'
        'theta = q(3);  v = V_act(1);  omega = V_act(2);'
        'M_mat = m * eye(3);'
        'Rl = mu_l * N / 2;'
        'f1_vec = [2*Rl*cos(theta); 2*Rl*sin(theta); 0] * tanh(v/0.005);'
        'Fy = 2 * tanh(omega/0.01) * mu_t * u * (N / l);'
        'Mr = tanh(omega/0.01) * mu_t * (N / l) * (l^2/4 - u^2);'
        'f2_vec = [Fy*sin(theta); -Fy*cos(theta); Mr/L];'
        'f_g = [-m*g; 0; 0];'
        'S_mat = [cos(theta), (u/L)*sin(theta); sin(theta), -(u/L)*cos(theta); 0, 1];'
        'S_dot = [-omega*sin(theta), (u_dot/L)*sin(theta) + (u*omega/L)*cos(theta);'
        '          omega*cos(theta), -(u_dot/L)*cos(theta) + (u*omega/L)*sin(theta); 0, 0];'
        'M_tilde = S_mat'' * M_mat * S_mat;'
        'C_tilde = S_mat'' * M_mat * S_dot;'
        'V_dot = M_tilde \ (B_tau - C_tilde*[v; L*omega] - S_mat''*f1_vec - S_mat''*f2_vec + S_mat''*f_g);'
        'V_dot(2) = V_dot(2) / L;          % [v_dot; omega_dot]'}, ...
        {'from:q_act','from:V_act','from:u','from:u_dot','from:Fr','from:Fl','const:dyn_p'}, {'tag:V_dot'}, 2400, 60);
    blk_box(ss, 'simulink/Continuous/Integrator', 'Integ_V', {'InitialCondition','[0;0]'}, ...
        {'from:V_dot'}, {'both:V_act:V_true'}, 2400, 520, 50, 40);
    %  8) Kinematics (6WP: q_dot = S_mat * v) + 벽면 외란
    fcn_box(ss, 'Kinematics', {
        'function q_dot = Kinematics(V_act, q, u, t, dist_p)'
        '% Kinematics : 차체 속도 -> 중심 자세 변화율 (+ 벽면 외란) / body speed -> centre pose rate (+ wall disturbance)'
        '% 입력 / Inputs : V_act [m/s, rad/s];  q [m, m, rad] 중심 자세 / centre pose;  u [m];  t [s]'
        '%                 dist_p = [slip_k -; 요철 진폭 A rad / wobble amplitude; 주파수 f Hz / frequency]'
        '% 출력 / Output : q_dot [3x1: m/s, m/s, rad/s]'
        '% 6WP_sim Kinematics: q_dot = S_mat * [v; L*omega]  (여기서는 q(3) = psi 로 정리)'
        '% 벽면 외란 (실기 영상·ZeroKey 로그에서 본 현상, [실측 필요]):'
        '%   자석 궤도 크리프 : 이동 속도의 slip_k 배만큼 중력 방향(-p1) 으로 밀린다'
        '%   궤도 요철        : heading 이 두 주파수로 조금씩 흔들린다'
        'theta = q(3);  v = V_act(1);  omega = V_act(2);'
        'slip_k = dist_p(1);  A = dist_p(2);  f = dist_p(3);'
        'wob = A*2*pi*f*cos(2*pi*f*t) + 0.5*A*2*pi*(2.3*f)*cos(2*pi*(2.3*f)*t + 1.0);'
        'q_dot = [v*cos(theta) + u*omega*sin(theta) - slip_k*abs(v);'
        '         v*sin(theta) - u*omega*cos(theta);'
        '         omega + wob];'}, ...
        {'from:V_act','from:q_act','from:u','clock','const:dist_p'}, {'tag:q_dot'}, 3050, 60);
    %  9) 적분 상태 = 로봇 중심 c (6WP_sim 과 같음). 출력 q_true = ZeroKey 태그 점
    %     Integrated state = robot centre c (as in 6WP_sim). Output q_true = ZeroKey tag point
    blk_box(ss, 'simulink/Continuous/Integrator', 'Integ_q', {'InitialCondition','q0c'}, ...
        {'from:q_dot'}, {'tag:q_act'}, 3050, 420, 50, 40);
    fcn_box(ss, 'TagPoint', {
        'function q_tag = TagPoint(q_c, r_tag)'
        '% TagPoint : 입력 q_c [m, m, rad] 중심 자세, r_tag [2x1, m] -> 출력 q_tag [m, m, rad] 태그 자세 (= q_true)'
        '% TagPoint : input centre pose q_c and r_tag -> output tag pose q_tag (= q_true)'
        '% 중심 -> 태그: p = c + R(psi) r_tag. 회전하면 태그는 반경 |r_tag| 원호를 그린다'
        '% Centre -> tag: p = c + R(psi) r_tag. While turning the tag moves on an arc of radius |r_tag|'
        'R = [cos(q_c(3)) -sin(q_c(3)); sin(q_c(3)) cos(q_c(3))];'
        'q_tag = [q_c(1:2) + R * r_tag; q_c(3)];'}, ...
        {'from:q_act','const:r_tag'}, {'both:q_tag:q_true'}, 3050, 560);
end

function addSensors(m, P)
%  SENSORS — 운동모델 참값 -> 실기 센서가 내는 값 (ZeroKey 위치·속도, IMU 쿼터니언)
    imu = 'imu_quat_meas';  if ver2(), imu = 'imu_meas'; end   % 2 판: [쿼터니언; 자이로 z]
    ss = add_subsys(m, 'Sensors', boxpos(P,4), {'q_true','V_true'}, ...
                    {'zk_meas','zk_vel_meas',imu,'zk_quat_meas'}, gnc_colour('env'));
    fan_inports(ss);
    fcn_box(ss, 'ZkWorld', {
        'function zk = ZkWorld(q_true, zk_ywall, noise)'
        '% ZkWorld : 가상 ZeroKey 위치 / virtual ZeroKey position'
        '% 입력 / Inputs : q_true [m, m, rad] 참 태그 자세 / true tag pose;  zk_ywall [m] 벽 법선 오프셋 / wall-normal offset'
        '%                 noise [3x1, m] 백색 잡음 (sigma 1 mm) / white noise'
        '% 출력 / Output : zk [3x1, m] 월드 [x; y; z] -> 지연 50 ms -> 20 Hz -> /zerokey/position'
        '% 벽면 (p1, p2) -> ZeroKey 월드 좌표 (x = p2, y = 벽 법선 오프셋, z = p1) + 측정 잡음'
        'zk = [q_true(2); zk_ywall; q_true(1)] + noise(:);'}, ...
        {'from:i_q_true','const:zk_ywall','rand:ZkNoise:zk_sigma^2:zk_Ts:[11 12 13]'}, {'tag:zk_raw'}, 330, 60);
    blk_box(ss, 'simulink/Continuous/Transport Delay', 'ZkDelay', ...
        {'DelayTime','zk_delay','InitialOutput','[q0(2); zk_ywall; q0(1)]'}, {'from:zk_raw'}, {'tag:zk_del'}, 900, 60, 70, 40);
    blk_box(ss, 'simulink/Discrete/Zero-Order Hold', 'ZkHold', {'SampleTime','zk_Ts'}, ...
        {'from:zk_del'}, {'zk_meas'}, 1250, 60, 40, 40);
    fcn_box(ss, 'ZkVel', {
        'function zkv = ZkVel(q_true, V_true, r_tag)'
        '% ZkVel : 입력 q_true [m, m, rad], V_true [m/s, rad/s], r_tag [m] -> 출력 zkv [3x1, m/s] -> /zerokey/vel'
        '% ZkVel : inputs q_true, V_true, r_tag -> output zkv [3x1, m/s] world velocity -> /zerokey/vel'
        '% ZeroKey 속도 (월드): x = p2 방향, z = p1 방향. 태그 속도 = 중심 속도 + omega J R r_tag'
        '% ZeroKey velocity (world): x = p2, z = p1. Tag velocity = centre velocity + omega J R r_tag'
        'psi = q_true(3);  R = [cos(psi) -sin(psi); sin(psi) cos(psi)];'
        'pd = V_true(1) * [cos(psi); sin(psi)] + V_true(2) * [0 -1; 1 0] * R * r_tag;'
        'zkv = [pd(2); 0; pd(1)];'}, {'from:i_q_true','from:i_V_true','const:r_tag'}, {'tag:zkv_raw'}, 330, 360);
    blk_box(ss, 'simulink/Discrete/Zero-Order Hold', 'ZkVHold', {'SampleTime','zk_Ts'}, ...
        {'from:zkv_raw'}, {'zk_vel_meas'}, 1250, 360, 40, 40);
    if ver2()
        fcn_box(ss, 'ImuQuat', {
            'function imu = ImuQuat(q_true, V_true, t, imu_drift, noise, gnoise, imu_gyro_bias)'
            '% ImuQuat (2 판) : 입력 q_true, V_true, t [s], imu_drift [rad/s], noise [rad], gnoise [rad/s], imu_gyro_bias [rad/s]'
            '%                  출력 imu [5x1] = [qw; qx; qy; qz; 자이로 z rad/s] -> /imu/data'
            '% ImuQuat (v2)   : output imu = [quaternion; gyro z] -> /imu/data'
            '% IMU 요각 -> 쿼터니언 (1 판과 같은 식) + 자이로 z = 참 요 각속도 + 바이어스 + 잡음'
            'psi = q_true(3) + imu_drift * t + noise;'
            'imu = [cos(psi/2); 0; 0; sin(psi/2); V_true(2) + imu_gyro_bias + gnoise];'}, ...
            {'from:i_q_true','from:i_V_true','clock','const:imu_drift','rand:ImuNoise:imu_sigma^2:imu_Ts:21', ...
             'rand:GyroNoise:imu_gyro_sigma^2:imu_Ts:22','const:imu_gyro_bias'}, {'tag:imu_raw'}, 330, 600);
    else
        fcn_box(ss, 'ImuQuat', {
            'function quat = ImuQuat(q_true, t, imu_drift, noise)'
            '% ImuQuat : 입력 q_true [m, m, rad], t [s], imu_drift [rad/s], noise [rad] -> 출력 quat [4x1] (w,x,y,z) -> /imu/data'
            '% ImuQuat : inputs true pose, time, drift rate, noise -> output attitude quaternion -> /imu/data'
            '% IMU 요각 = 참 heading + 드리프트 + 잡음 -> 쿼터니언 (6WP_sim Euler2Quat 와 같은 식)'
            'psi = q_true(3) + imu_drift * t + noise;'
            'quat = [cos(psi/2); 0; 0; sin(psi/2)];'}, ...
            {'from:i_q_true','clock','const:imu_drift','rand:ImuNoise:imu_sigma^2:imu_Ts:21'}, {'tag:imu_raw'}, 330, 600);
    end
    blk_box(ss, 'simulink/Discrete/Zero-Order Hold', 'ImuHold', {'SampleTime','imu_Ts'}, ...
        {'from:imu_raw'}, {imu}, 1250, 600, 40, 40);
    %  ZeroKey INS 자세 (/zerokey/orientation) — 실기 모델 Tracked_UGV_ZeroKey_* 가 구독한다.
    %  이 시뮬레이션의 제어기는 IMU 요각을 쓰지만, 실기 모델을 그대로 붙여도 돌도록 같이 발행한다
    fcn_box(ss, 'ZkQuat', {
        'function quat = ZkQuat(q_true, noise)'
        '% ZkQuat : 입력 q_true, noise [rad] -> 출력 quat [4x1] -> /zerokey/orientation (이 제어기는 쓰지 않음)'
        '% ZkQuat : output quaternion -> /zerokey/orientation (not used by this controller, kept for the real models)'
        '% ZeroKey INS 요각 = 참 heading + 잡음 (드리프트 없음, 대신 IMU 보다 잡음이 크다) -> 쿼터니언'
        'psi = q_true(3) + noise;'
        'quat = [cos(psi/2); 0; 0; sin(psi/2)];'}, ...
        {'from:i_q_true','rand:ZkQNoise:(3*imu_sigma)^2:zk_Ts:31'}, {'tag:zkq_raw'}, 330, 900);
    blk_box(ss, 'simulink/Discrete/Zero-Order Hold', 'ZkQHold', {'SampleTime','zk_Ts'}, ...
        {'from:zkq_raw'}, {'zk_quat_meas'}, 1250, 900, 40, 40);
    for b = {'ZkHold','ZkVHold','ImuHold','ZkQHold'}
        set_param([ss '/' b{1}], 'Description', '이산: 센서 갱신 주기 (ROS 발행 주기)');
    end
end

function addStagePlant(m, P)
%  STAGEPLANT — Zaber X-LSM100B 두 축. 펌웨어 = 궤적 생성(최고속도) + 위치 PID + 추력 한계
%  이동 질량은 CAD·데이터시트 추정, 벽면이라 중력이 heading 에 따라 축 방향으로 걸린다
    ss = add_subsys(m, 'StagePlant', boxpos(P,2), {'stage_target_rx','q_true'}, {'stage_pos'}, gnc_colour('plant'));
    fan_inports(ss);
    %  1) 스테이지 노드가 시리얼로 move abs 를 보내는 주기 (zaber_stage_node.py 의 cmd_rate) + 시리얼 지연
    blk_box(ss, 'simulink/Discrete/Zero-Order Hold', 'CmdHold', {'SampleTime','stage_cmd_Ts'}, ...
        {'from:i_stage_target_rx'}, {'tag:s_hold'}, 520, 60, 40, 40);
    set_param([ss '/CmdHold'], 'Description', '이산: 스테이지 노드가 move abs 를 보내는 주기 (cmd_rate)');
    blk_box(ss, 'simulink/Continuous/Transport Delay', 'CmdDelay', ...
        {'DelayTime','stage_delay','InitialOutput','[stage_c; stage_c]'}, {'from:s_hold'}, {'tag:s_goal'}, 520, 200, 70, 40);
    %  2) 펌웨어 궤적 생성 (move abs : maxspeed · accel 로 사다리꼴 속도)
    fcn_box(ss, 'ZaberTraj', {
        'function x_t_dot = ZaberTraj(s_goal, x_t, stage_vmax, stage_acc, stage_travel)'
        '% ZaberTraj : Zaber 펌웨어 궤적 생성기의 모델 (우리가 바꿀 수 없는 상용 부분)'
        '% ZaberTraj : model of the Zaber firmware trajectory generator (commercial, not ours to change)'
        '% 입력 / Inputs : s_goal [2x1, mm] move abs 목표 / move abs target;  x_t [4x1: mm, mm, mm/s, mm/s]'
        '%                 stage_vmax [mm/s] maxspeed;  stage_acc [mm/s^2] accel;  stage_travel [mm] 행정 / travel'
        '% 출력 / Output : x_t_dot [4x1: mm/s, mm/s, mm/s^2, mm/s^2]'
        '% Zaber 펌웨어의 move abs 궤적 (두 축 따로).  상태 x_t = [s_t(2) mm; v_t(2) mm/s]'
        '%   목표까지 남은 거리에서 멈출 수 있는 속도 sqrt(2 a |e|) 와 maxspeed 중 작은 값으로 간다'
        '%   (가속·감속 한계 = accel 설정).  새 move abs 가 오면 그 자리에서 목표만 바뀐다'
        'goal = min(max(s_goal(:), 0), stage_travel);'
        's = x_t(1:2);  v = x_t(3:4);'
        'e = goal - s;'
        'v_des = sign(e) .* min(stage_vmax, sqrt(2*stage_acc*abs(e)));'
        'a = min(max(50*(v_des - v), -stage_acc), stage_acc);'
        'x_t_dot = [v; a];'}, ...
        {'from:s_goal','from:x_t','const:stage_vmax','const:stage_acc','const:stage_travel'}, {'tag:x_t_dot'}, 900, 60);
    blk_box(ss, 'simulink/Continuous/Integrator', 'Integ_t', {'InitialCondition','x_t0'}, ...
        {'from:x_t_dot'}, {'tag:x_t'}, 900, 420, 50, 40);
    %  3) 스테퍼 + 이동 질량 (데이터시트: 이동 질량 0.063 kg, 스테이지 0.39 kg, 추력-속도 곡선)
    fcn_box(ss, 'ZaberServo', {
        'function x_dot = ZaberServo(x_t, x_s, q_true, stg_p, Fv_v, Fv_F)'
        '% ZaberServo : Zaber 내부 위치 서보 + 캐리지 질량의 모델 (상용 펌웨어, 명령만 줄 수 있음)'
        '% ZaberServo : model of the internal Zaber position servo and carriage mass (commercial; command only)'
        '% 입력 / Inputs : x_t [mm, mm/s] 궤적 / trajectory;  x_s [6x1: m, m, m/s, m/s, m s, m s] 캐리지 상태 / carriage state'
        '%                 q_true [m, m, rad] (중력 방향 계산 / for the gravity direction)'
        '%                 stg_p = [m_X kg; m_Y kg; c N s/m; Kp N/m (2); Kd N s/m (2); Ki N/(m s) (2); g m/s^2]'
        '%                 Fv_v [mm/s], Fv_F [N]  추력-속도 곡선 / thrust-speed curve'
        '% 출력 / Output : x_dot [6x1: m/s, m/s, m/s^2, m/s^2, m, m]'
        '% Zaber X-LSM100B 두 축 (X = 아래 축, Y = 위 축).  상태 x_s = [s(2) m; v(2) m/s; 적분(2)]'
        '%   스테퍼는 궤적을 단단한 스프링처럼 따라간다 -> 높은 대역의 위치 루프로 나타낸다'
        '%   낼 수 있는 힘은 속도에 따라 줄어든다 (데이터시트 Thrust-Speed, B 형 24 V 곡선 Fv_v -> Fv_F)'
        '%   X 축은 Y 축 스테이지 전체 + 펜 홀더를 민다 (m_sx > m_sy)'
        '%   벽면 중력 [-g; 0] 을 바디 좌표로 돌려 각 축에 m g 성분을 건다'
        'm  = stg_p(1:2);  c = stg_p(3);'
        'Kp = stg_p(4:5);  Kd = stg_p(6:7);  Ki = stg_p(8:9);  g = stg_p(10);'
        's = x_s(1:2);  v = x_s(3:4);  ie = x_s(5:6);'
        's_t = x_t(1:2)/1000;  v_t = x_t(3:4)/1000;'
        'e = s_t - s;'
        'F_av = interp1(Fv_v, Fv_F, min(abs(v)*1000, Fv_v(end)));   % 지금 속도에서 낼 수 있는 추력 [N]'
        'F = min(max(Kp.*e + Ki.*ie + Kd.*(v_t - v), -F_av), F_av);'
        'psi = q_true(3);'
        'g_b = [cos(psi) sin(psi); -sin(psi) cos(psi)] * [-g; 0];'
        'a = (F + m.*g_b - c*v) ./ m;'
        'x_dot = [v; a; e];'}, ...
        {'from:x_t','from:x_s','from:i_q_true','const:stg_p','const:stage_Fv_v','const:stage_Fv_F'}, {'tag:x_s_dot'}, 1500, 60);
    blk_box(ss, 'simulink/Continuous/Integrator', 'Integ_s', {'InitialCondition','x_s0'}, ...
        {'from:x_s_dot'}, {'tag:x_s'}, 1500, 480, 50, 40);
    fcn_box(ss, 'Encoder', {
        'function stage_pos = Encoder(x_s, stage_travel)'
        '% Encoder : 입력 x_s [m, m/s, m s], stage_travel [mm] -> 출력 stage_pos [2x1, mm] -> /stage/pos (20 Hz)'
        '% Encoder : output = position reported by get pos [mm], clipped at the travel ends'
        '% 내장 엔코더 위치 [mm] (get pos 가 돌려주는 값), 행정 끝에서 막힌다'
        'stage_pos = min(max(1000 * x_s(1:2), 0), stage_travel);'}, ...
        {'from:x_s','const:stage_travel'}, {'stage_pos'}, 2100, 60);
end

function addPlantRosOut(m, P)
%  PLANTROSOUT — 가상 센서·스테이지가 실기와 같은 토픽을 발행한다
    imu = 'imu_quat_meas';  fimu = {'orientation.w','orientation.x','orientation.y','orientation.z'};
    if ver2(), imu = 'imu_meas';  fimu = [fimu, {'angular_velocity.z'}]; end   % 2 판: 한 메시지에 자세 + 자이로
    ss = add_subsys(m, 'PlantRosOut', boxpos(P,5), {'zk_meas','zk_vel_meas',imu,'stage_pos','zk_quat_meas'}, {}, gnc_colour('ros'));
    add_pub(ss, 'PubZk',  '/zerokey/position', 'geometry_msgs/PointStamped', {'point.x','point.y','point.z'}, 60,  'zk_meas', '');
    add_pub(ss, 'PubZkV', '/zerokey/vel',      'geometry_msgs/TwistStamped', {'twist.linear.x','twist.linear.y','twist.linear.z'}, 300, 'zk_vel_meas', '');
    add_pub(ss, 'PubImu', '/imu/data',         'sensor_msgs/Imu', fimu, 540, imu, '');
    add_pub(ss, 'PubStg', '/stage/pos',        'geometry_msgs/PointStamped', {'point.x','point.y'}, 820, 'stage_pos', 'stage_Ts');
    add_pub(ss, 'PubZkQ', '/zerokey/orientation', 'geometry_msgs/QuaternionStamped', ...
            {'quaternion.w','quaternion.x','quaternion.y','quaternion.z'}, 1060, 'zk_quat_meas', '');
end

function addPenTrace(m, P)
%  PENTRACE — 참값으로 계산한 실제 펜 위치 (평가용, 제어기는 이 값을 모른다)
    ss = add_subsys(m, 'PenTrace', [P(1) P(2) P(1)+150 P(2)+80], {}, {}, gnc_colour('measurement'));
    fcn_box(ss, 'PenTraceFcn', {
        'function [pen_true, pen_nocomp, e_pen, e_nocomp] = PenTraceFcn(q_true, stage_pos, lineAB, pen_d, stage_c)'
        '% PenTraceFcn : 참값으로 계산한 펜 위치와 횡오차 (평가 전용, 제어기는 모름)'
        '% PenTraceFcn : true pen position and cross error from true values (evaluation only)'
        '% 입력 / Inputs : q_true [m, m, rad] 참 태그 자세;  stage_pos [2x1, mm] 엔코더;  lineAB [4x1, m];'
        '%                 pen_d [2x1, m];  stage_c [mm]'
        '% 출력 / Outputs: pen_true, pen_nocomp [2x1, m];  e_pen, e_nocomp [m]'
        '% pen_true   : 스테이지가 실제로 움직인 결과의 펜 위치 [p1; p2]'
        '% pen_nocomp : 스테이지를 중앙에 고정했을 때 펜 위치 (보상 없음 비교)'
        '% e_*        : 직선에 대한 부호 있는 횡오차 [m] (직선 진행방향의 왼쪽 +)'
        'A = lineAB(1:2);  B = lineAB(3:4);'
        'u = (B - A) / max(norm(B - A), 1e-9);  n = [-u(2); u(1)];'
        'pT = q_true(1:2);'
        'R  = [cos(q_true(3)) -sin(q_true(3)); sin(q_true(3)) cos(q_true(3))];'
        'pen_true   = pT + R * (pen_d + (stage_pos - stage_c) / 1000);'
        'pen_nocomp = pT + R * pen_d;'
        'e_pen    = dot(n, pen_true - A);'
        'e_nocomp = dot(n, pen_nocomp - A);'}, ...
        {'from:q_true','from:stage_pos','from:lineAB','const:pen_d','const:stage_c'}, ...
        {'gtag:pen_true','gtag:pen_nocomp','gtag:e_pen','gtag:e_nocomp'}, 330, 60);
end

function addLogging(m, P, sig)
%  LOGGING — To Workspace 하나. Bus Creator 로 묶어 out.log.<신호이름> 으로 꺼낸다
    ss = add_subsys(m, 'Logging', [P(1) P(2) P(1)+150 P(2)+80], {}, {}, gnc_colour('measurement'));
    n = numel(sig);
    add_block('simulink/Signal Routing/Bus Creator', [ss '/LogBus'], ...
              'Position', [300 40 310 40+40*n], 'Inputs', num2str(n));
    for k = 1:n
        b = port_xy(ss,'LogBus','Inport',k);
        add_block('simulink/Signal Routing/From', [ss '/Fr_' sig{k}], 'GotoTag', sig{k}, ...
                  'Position', [150 b(2)-11 250 b(2)+11]);
        h = add_line(ss, ['Fr_' sig{k} '/1'], sprintf('LogBus/%d', k));
        set_param(h, 'Name', sig{k});
    end
    q = port_xy(ss,'LogBus','Outport',1);
    %  운동모델 참값(연속)과 제어기 신호(Ts_ctrl)가 섞여 있으므로 한 번 Ts_ctrl 로 잡아서 기록한다
    add_block('simulink/Discrete/Zero-Order Hold', [ss '/LogHold'], 'SampleTime','Ts_ctrl', ...
              'Position', [370 q(2)-15 400 q(2)+15]);
    set_param([ss '/LogHold'], 'Description', '이산: 기록 주기 (제어기 주기와 같다)');
    add_block('simulink/Sinks/To Workspace', [ss '/log'], 'Position', [460 q(2)-15 540 q(2)+15], ...
              'VariableName','log', 'SaveFormat','Timeseries', 'SampleTime','Ts_ctrl');
    add_line(ss, 'LogBus/1', 'LogHold/1');
    add_line(ss, 'LogHold/1', 'log/1');
end

function s = log_signals(sim)
    s = {'pose','mode','pen_on','v_cmd','w_cmd','e_cross','motor_pwm','stage_target_pos', ...
         'stage_pos_meas','zerokey_position','zerokey_vel','imu_quat','lineAB'};
    if ver2()   % 2 판: IMU 신호 이름이 바뀌고 내부 PI 신호를 같이 기록
        s{strcmp(s, 'imu_quat')} = 'imu_data';
        s = [s, {'V_meas','V_bar','W_bar'}];
    end
    if sim
        s = [s, {'q_true','V_true','stage_pos','pen_true','pen_nocomp','e_pen','e_nocomp'}];
    end
end

% =====================================================================
%  ROS 2 블록 관용구 — 선이 전부 수평이 되도록 포트 높이를 맞춘다
% =====================================================================
function add_sub(ss, bn, topic, mtype, fields, y, outname)
%  Subscribe -> Bus Selector -> (Mux) -> Outport
    n = numel(fields);  H = max(60, 40*n);
    add_block('ros2lib/Subscribe', [ss '/' bn], 'Position', [60 y 200 y+60]);
    set_param([ss '/' bn], 'topicSource','Specify your own', 'topic', topic, ...
              'messageType', mtype, 'sampleTime','Ts_ctrl');
    set_param([ss '/' bn], 'Description', '이산: ROS Subscribe 는 구독 주기로만 돈다');
    m2 = port_xy(ss, bn, 'Outport', 2);
    sel = ['Sel_' bn];
    add_block('simulink/Signal Routing/Bus Selector', [ss '/' sel], 'Position', [300 m2(2)-H/2 305 m2(2)+H/2]);
    set_param([ss '/' sel], 'OutputSignals', strjoin(fields, ','));
    b = port_xy(ss, sel, 'Inport', 1);
    set_param([ss '/' bn], 'Position', get_param([ss '/' bn], 'Position') + [0 1 0 1]*(b(2) - m2(2)));
    add_line(ss, [bn '/2'], [sel '/1']);
    m1 = port_xy(ss, bn, 'Outport', 1);
    add_block('simulink/Sinks/Terminator', [ss '/IsNew_' bn], 'Position', [240 m1(2)-8 255 m1(2)+8]);
    add_line(ss, [bn '/1'], ['IsNew_' bn '/1']);
    p = get_param([ss '/' sel], 'Position');
    if n > 1
        mx = ['Mux_' bn];
        add_block('simulink/Signal Routing/Mux', [ss '/' mx], 'Inputs', num2str(n), ...
                  'Position', [420 p(2) 425 p(4)]);
        for j = 1:n, add_line(ss, sprintf('%s/%d', sel, j), sprintf('%s/%d', mx, j)); end
        src = mx;
    else
        src = sel;
    end
    q = port_xy(ss, src, 'Outport', 1);
    set_param([ss '/' outname], 'Position', [560 q(2)-10 590 q(2)+10]);
    add_line(ss, [src '/1'], [outname '/1']);
end

function add_pub(ss, bn, topic, mtype, fields, y, inname, zoh)
%  Inport -> (ZOH) -> Demux -> Bus Assignment <- Blank Message ; -> Publish
    n = numel(fields);  pitch = 40;  H = pitch*(n+1);
    asg = ['Asg_' bn];  dmx = ['Dmx_' bn];  blank = ['Blank_' bn];
    add_block('simulink/Signal Routing/Bus Assignment', [ss '/' asg], 'Position', [520 y 525 y+H]);
    set_param([ss '/' asg], 'AssignedSignals', strjoin(fields, ','));
    a1 = port_xy(ss, asg, 'Inport', 1);  a2 = port_xy(ss, asg, 'Inport', 2);  an = port_xy(ss, asg, 'Inport', n+1);
    add_block('ros2lib/Blank Message', [ss '/' blank], 'Position', [300 a1(2)-20 440 a1(2)+20]);
    set_param([ss '/' blank], 'entityType', mtype, 'messageType', mtype);
    bo = port_xy(ss, blank, 'Outport', 1);
    set_param([ss '/' blank], 'Position', get_param([ss '/' blank], 'Position') + [0 1 0 1]*(a1(2) - bo(2)));
    add_line(ss, [blank '/1'], [asg '/1']);
    if n > 1
        add_block('simulink/Signal Routing/Demux', [ss '/' dmx], 'Outputs', num2str(n), ...
                  'Position', [440 a2(2)-pitch/2 445 an(2)+pitch/2]);
        %  Demux 출력 포트를 Bus Assignment 2..n+1 입력 높이에 맞춘다
        d1 = port_xy(ss, dmx, 'Outport', 1);  dn = port_xy(ss, dmx, 'Outport', n);
        p = get_param([ss '/' dmx], 'Position');
        sc = (an(2) - a2(2)) / max(dn(2) - d1(2), 1);
        set_param([ss '/' dmx], 'Position', [p(1) a2(2) - (d1(2)-p(2))*sc p(3) an(2) + (p(4)-dn(2))*sc]);
        for j = 1:n, add_line(ss, sprintf('%s/%d', dmx, j), sprintf('%s/%d', asg, j+1)); end
        din = port_xy(ss, dmx, 'Inport', 1);
        first = dmx;
    else
        din = a2;  first = asg;
    end
    if ~isempty(zoh)
        zn = ['Hold_' bn];
        add_block('simulink/Discrete/Zero-Order Hold', [ss '/' zn], 'SampleTime', zoh, ...
                  'Position', [330 din(2)-15 360 din(2)+15]);
        set_param([ss '/' zn], 'Description', '이산: 발행 주기');
        if n > 1, add_line(ss, [zn '/1'], [first '/1']); else, add_line(ss, [zn '/1'], [asg '/2']); end
        set_param([ss '/' inname], 'Position', [200 din(2)-10 230 din(2)+10]);
        add_line(ss, [inname '/1'], [zn '/1']);
    else
        set_param([ss '/' inname], 'Position', [200 din(2)-10 230 din(2)+10]);
        if n > 1, add_line(ss, [inname '/1'], [first '/1']); else, add_line(ss, [inname '/1'], [asg '/2']); end
    end
    o = port_xy(ss, asg, 'Outport', 1);
    add_block('ros2lib/Publish', [ss '/' bn], 'Position', [620 o(2)-30 760 o(2)+30]);
    set_param([ss '/' bn], 'topicSource','Specify your own', 'topic', topic, 'messageType', mtype);
    pi1 = port_xy(ss, bn, 'Inport', 1);
    set_param([ss '/' bn], 'Position', get_param([ss '/' bn], 'Position') + [0 1 0 1]*(o(2) - pi1(2)));
    add_line(ss, [asg '/1'], [bn '/1']);
end

% =====================================================================
%  상자 채우기 관용구
%    ins  : 'in:<포트>' | 'from:<태그>' | 'const:<변수>' | 'clock' | 'rand:<이름>:<분산>:<Ts>:<seed>'
%    outs : '<포트>' | 'tag:<로컬태그>' | 'gtag:<전역태그>' | 'both:<로컬태그>:<포트>'
%           'sat:<포트>:<한계>' | 'sattag:<태그>:<한계>' | 'stagecmd:<포트>' | 'term:<이름>' | 'stop:<이름>'
%  소스는 자기 포트 높이로, 출력도 포트 높이로 놓는다 -> 선이 전부 수평 직선
% =====================================================================
function fcn_box(ss, fname, code, ins, outs, X, Y0)
    if nargin < 6, X = 330; end
    if nargin < 7, Y0 = 60; end
    if iscell(code), code = strjoin(code, newline); end
    nIn = numel(ins);  nOut = numel(outs);
    H = max(100, 60*max(nIn, nOut));
    add_block('simulink/User-Defined Functions/MATLAB Function', [ss '/' fname], ...
              'Position', [X Y0 X+200 Y0+H]);
    setFcn(ss, fname, code);
    wire_box(ss, fname, ins, outs, X, 200);
end

function blk_box(ss, lib, name, params, ins, outs, X, Y0, W, H)
    add_block(lib, [ss '/' name], 'Position', [X Y0 X+W Y0+H]);
    if ~isempty(params), set_param([ss '/' name], params{:}); end
    wire_box(ss, name, ins, outs, X, W);
end

function wire_box(ss, bn, ins, outs, X, W)
    for k = 1:numel(ins)
        tk = strsplit(ins{k}, ':');
        b = port_xy(ss, bn, 'Inport', k);
        dst = sprintf('%s/%d', bn, k);
        switch tk{1}
            case 'in'
                set_param([ss '/' tk{2}], 'Position', [X-200 b(2)-10 X-170 b(2)+10]);
                add_line(ss, [tk{2} '/1'], dst);
            case 'from'
                w = max(70, 7*numel(tk{2}) + 20);
                nm = uniq(ss, ['Fr_' tk{2}]);
                add_block('simulink/Signal Routing/From', [ss '/' nm], 'GotoTag', tk{2}, ...
                          'Position', [X-50-w b(2)-11 X-50 b(2)+11]);
                add_line(ss, [nm '/1'], dst);
            case 'const'
                w = max(60, 7*numel(tk{2}) + 16);
                nm = uniq(ss, ['C_' tk{2}]);
                add_block('simulink/Sources/Constant', [ss '/' nm], 'Value', tk{2}, ...
                          'Position', [X-50-w b(2)-15 X-50 b(2)+15], 'ShowName','off');
                add_line(ss, [nm '/1'], dst);
            case 'clock'
                nm = uniq(ss, 'Clock');
                add_block('simulink/Sources/Clock', [ss '/' nm], 'Position', [X-80 b(2)-10 X-60 b(2)+10]);
                add_line(ss, [nm '/1'], dst);
            case 'dclock'
                %  제어기 안의 시간 — 연속 Clock 을 쓰면 Merge 앞 모드 상자들의 샘플 시간이 어긋난다
                nm = uniq(ss, 'DClock');
                add_block('simulink/Sources/Digital Clock', [ss '/' nm], 'SampleTime','Ts_ctrl', ...
                          'Position', [X-90 b(2)-12 X-60 b(2)+12]);
                set_param([ss '/' nm], 'Description', '이산: 제어기 주기의 시간 (Merge 앞 모드 상자 샘플 시간 통일)');
                add_line(ss, [nm '/1'], dst);
            case 'rand'
                add_block('simulink/Sources/Random Number', [ss '/' tk{2}], ...
                          'Position', [X-100 b(2)-15 X-60 b(2)+15], 'Mean','0', ...
                          'Variance', tk{3}, 'SampleTime', tk{4}, 'Seed', tk{5});
                set_param([ss '/' tk{2}], 'Description', '이산: 센서 잡음은 센서 갱신 주기로 바뀐다');
                add_line(ss, [tk{2} '/1'], dst);
        end
    end
    xo = X + W;
    for j = 1:numel(outs)
        tk = strsplit(outs{j}, ':');
        q = port_xy(ss, bn, 'Outport', j);
        src = sprintf('%s/%d', bn, j);
        switch tk{1}
            case {'tag','gtag'}
                if strcmp(tk{1},'tag'), vis = 'local'; else, vis = 'global'; end
                w = max(70, 7*numel(tk{2}) + 20);
                add_block('simulink/Signal Routing/Goto', [ss '/Go_' tk{2}], 'GotoTag', tk{2}, ...
                          'TagVisibility', vis, 'Position', [xo+40 q(2)-11 xo+40+w q(2)+11]);
                add_line(ss, src, ['Go_' tk{2} '/1']);
            case 'both'
                w = max(70, 7*numel(tk{2}) + 20);
                %  선을 가지치지 않는다: 같은 높이에 로컬 Goto, 그 아래 From -> 출력 포트 (꺾임 0)
                add_block('simulink/Signal Routing/Goto', [ss '/Go_' tk{2}], 'GotoTag', tk{2}, ...
                          'TagVisibility','local', 'Position', [xo+40 q(2)-11 xo+40+w q(2)+11]);
                add_line(ss, src, ['Go_' tk{2} '/1']);
                nm = uniq(ss, ['Fr_' tk{2}]);
                add_block('simulink/Signal Routing/From', [ss '/' nm], 'GotoTag', tk{2}, ...
                          'Position', [xo+40 q(2)+50 xo+40+w q(2)+72]);
                set_param([ss '/' tk{3}], 'Position', [xo+w+120 q(2)+51 xo+w+150 q(2)+71]);
                add_line(ss, [nm '/1'], [tk{3} '/1']);
            case 'sat'
                sn = ['Sat_' tk{2}];
                add_block('simulink/Discontinuities/Saturation', [ss '/' sn], ...
                          'Position', [xo+60 q(2)-15 xo+100 q(2)+15], ...
                          'UpperLimit', tk{3}, 'LowerLimit', ['-(' tk{3} ')']);
                add_line(ss, src, [sn '/1']);
                set_param([ss '/' tk{2}], 'Position', [xo+200 q(2)-10 xo+230 q(2)+10]);
                add_line(ss, [sn '/1'], [tk{2} '/1']);
            case 'sattag'
                sn = ['Sat_' tk{2}];
                add_block('simulink/Discontinuities/Saturation', [ss '/' sn], ...
                          'Position', [xo+60 q(2)-15 xo+100 q(2)+15], ...
                          'UpperLimit', tk{3}, 'LowerLimit', ['-(' tk{3} ')']);
                add_line(ss, src, [sn '/1']);
                add_block('simulink/Signal Routing/Goto', [ss '/Go_' tk{2}], 'GotoTag', tk{2}, ...
                          'TagVisibility','local', 'Position', [xo+150 q(2)-11 xo+220 q(2)+11]);
                add_line(ss, [sn '/1'], ['Go_' tk{2} '/1']);
            case 'stagecmd'
                add_block('simulink/Discontinuities/Saturation', [ss '/Travel'], ...
                          'Position', [xo+60 q(2)-15 xo+100 q(2)+15], ...
                          'UpperLimit','stage_travel', 'LowerLimit','0');
                add_block('simulink/Discontinuities/Rate Limiter', [ss '/CmdRate'], ...
                          'Position', [xo+150 q(2)-15 xo+190 q(2)+15], ...
                          'RisingSlewLimit','stage_vcmd', 'FallingSlewLimit','-stage_vcmd');
                try, set_param([ss '/CmdRate'], 'InitialCondition','stage_c'); catch, end
                add_line(ss, src, 'Travel/1');  add_line(ss, 'Travel/1', 'CmdRate/1');
                set_param([ss '/' tk{2}], 'Position', [xo+280 q(2)-10 xo+310 q(2)+10]);
                add_line(ss, 'CmdRate/1', [tk{2} '/1']);
            case 'term'
                add_block('simulink/Sinks/Terminator', [ss '/T_' tk{2}], ...
                          'Position', [xo+60 q(2)-10 xo+80 q(2)+10]);
                add_line(ss, src, ['T_' tk{2} '/1']);
            case 'stop'
                add_block('simulink/Sinks/Stop Simulation', [ss '/Stop'], ...
                          'Position', [xo+60 q(2)-15 xo+90 q(2)+15]);
                add_line(ss, src, 'Stop/1');
            otherwise
                set_param([ss '/' tk{1}], 'Position', [xo+200 q(2)-10 xo+230 q(2)+10]);
                add_line(ss, src, [tk{1} '/1']);
        end
    end
end

function nm = uniq(ss, base)
    nm = base;  j = 1;
    while ~isempty(find_system(ss, 'SearchDepth',1, 'Name', nm))
        j = j + 1;  nm = sprintf('%s_%d', base, j);
    end
end

%  서브시스템 입력마다 바로 옆에 로컬 Goto 'i_<이름>' — 여러 블록이 같은 입력을 쓸 때
function fan_inports(ss)
    ins = find_system(ss, 'SearchDepth',1, 'BlockType','Inport');
    for k = 1:numel(ins)
        nm = get_param(ins{k}, 'Name');
        p = get_param(ins{k}, 'Position');
        w = max(70, 7*numel(nm) + 30);
        add_block('simulink/Signal Routing/Goto', [ss '/Go_i_' nm], 'GotoTag', ['i_' nm], ...
                  'TagVisibility','local', 'Position', [p(3)+40 (p(2)+p(4))/2-11 p(3)+40+w (p(2)+p(4))/2+11]);
        add_line(ss, [nm '/1'], ['Go_i_' nm '/1']);
    end
end

% =====================================================================
%  공통 헬퍼
% =====================================================================
function fresh(m)
    try, close_system(m, 0); catch, end
    if exist([m '.slx'],'file'), delete([m '.slx']); end
    new_system(m);
end

function setFcn(sys, path, code)
    S = sfroot;
    fpos = get_param([sys '/' path], 'Position');
    ch = S.find('-isa','Stateflow.EMChart','Path',[sys '/' path]);
    ch.Script = code;
    set_param([sys '/' path], 'Position', fpos);
end

%  persistent 변수를 쓰는 MATLAB Function 은 연속 샘플 시간이 금지 -> 이산으로
function set_disc(sys, path, Ts)
    S = sfroot;
    ch = S.find('-isa','Stateflow.EMChart','Path',[sys '/' path]);
    ch.ChartUpdate = 'DISCRETE';
    ch.SampleTime  = Ts;
end

%  맨 위 상자들을 이름 있는 포트끼리 잇는다 — 출력 포트 옆에 전역 Goto, 입력 포트 옆에 From.
%  태그 이름 = 포트 이름. 그래서 PenTrace·Logging 같은 포트 없는 상자도 같은 이름으로 꺼내 쓴다.
%  (가상 실험장과 제어기 사이는 이름이 달라 이어지지 않는다 — 둘은 ROS 2 토픽으로만 이어진다)
function wire_tags(m)
    subs = find_system(m, 'SearchDepth',1, 'BlockType','SubSystem');
    for k = 1:numel(subs)
        bn = get_param(subs{k}, 'Name');
        ph = get_param(subs{k}, 'PortHandles');
        ins  = find_system(subs{k}, 'SearchDepth',1, 'BlockType','Inport');
        outs = find_system(subs{k}, 'SearchDepth',1, 'BlockType','Outport');
        for j = 1:numel(ins)
            nm = get_param(ins{j}, 'Name');
            pn = str2double(get_param(ins{j}, 'Port'));
            p  = get_param(ph.Inport(pn), 'Position');
            w  = max(80, 7*numel(nm) + 24);
            fr = sprintf('%s/Fr_%s_%s', m, bn, nm);
            add_block('simulink/Signal Routing/From', fr, 'GotoTag', nm, ...
                      'Position', [p(1)-40-w p(2)-11 p(1)-40 p(2)+11]);
            add_line(m, sprintf('Fr_%s_%s/1', bn, nm), sprintf('%s/%d', bn, pn));
        end
        for j = 1:numel(outs)
            nm = get_param(outs{j}, 'Name');
            pn = str2double(get_param(outs{j}, 'Port'));
            p  = get_param(ph.Outport(pn), 'Position');
            w  = max(80, 7*numel(nm) + 24);
            go = sprintf('%s/Go_%s', m, nm);
            add_block('simulink/Signal Routing/Goto', go, 'GotoTag', nm, 'TagVisibility','global', ...
                      'Position', [p(1)+40 p(2)-11 p(1)+40+w p(2)+11]);
            add_line(m, sprintf('%s/%d', bn, pn), sprintf('Go_%s/1', nm));
        end
    end
end


function fix_const_crossings(m)
%  서브시스템 안에서 남의 선이 지나가는 Constant 블록을 왼쪽으로 민다 (최대 4 번, 50 px 씩)
%  Shift Constant blocks left while an unrelated line crosses them (arrangeSystem leftovers).
    cs = find_system(m, 'LookUnderMasks','all', 'BlockType','Constant');
    for k = 1:numel(cs)
        sys = get_param(cs{k}, 'Parent');
        if strcmp(sys, m), continue; end
        for it = 1:4
            if ~const_crossed(sys, cs{k}), break; end
            set_param(cs{k}, 'Position', get_param(cs{k}, 'Position') - [50 0 50 0]);
        end
    end
end

function tf = const_crossed(sys, c)
    r = get_param(c, 'Position');  hc = get_param(c, 'Handle');  tf = false;
    ls = find_system(sys, 'SearchDepth',1, 'FindAll','on', 'Type','line');
    for j = 1:numel(ls)
        if get_param(ls(j), 'SrcBlockHandle') == hc, continue; end
        pts = get_param(ls(j), 'Points');
        for i = 1:size(pts,1)-1
            a = pts(i,:);  b = pts(i+1,:);
            if a(2) == b(2)          % 수평 선분
                if a(2) > r(2) && a(2) < r(4) && max(a(1),b(1)) > r(1) && min(a(1),b(1)) < r(3), tf = true; return; end
            elseif a(1) == b(1)      % 수직 선분
                if a(1) > r(1) && a(1) < r(3) && max(a(2),b(2)) > r(2) && min(a(2),b(2)) < r(4), tf = true; return; end
            end
        end
    end
end

function note(m, tag, txt, x, y)
    add_block('built-in/Note', [m '/' tag], 'Position',[x y x y], 'Text', txt);
end

% =====================================================================
%  선으로 잇기 — 가독성: 신호 흐름이 선으로 보이게, Goto/From 은 기록용 탭만
% =====================================================================
%  1) 맨 위 상자들을 같은 이름의 포트끼리 선으로 바로 잇는다 (선에 신호 이름)
%  2) 모든 서브시스템 안의 로컬 Goto/From 쌍을 선으로 바꾼다
%  3) Logging · PenTrace 가 쓰는 신호만 만든 상자 안에 전역 Goto (탭) 를 하나씩 둔다
%  (가상 실험장과 제어기 사이는 포트 이름이 달라 이어지지 않는다 — 둘은 ROS 2 토픽으로만 이어진다)
function wire_direct(m)
    connect_root(m);
    collapse_all(m);
    shift_ctrl_inputs(m);
    tap_gotos(m);
end

function shift_ctrl_inputs(m)
%  RobotCtrl: 입력 포트·Goto 를 From 열 가까이로 (빈 띠를 줄인다)
    ss = [m '/RobotCtrl'];
    if getSimulinkBlockHandle(ss) < 0, return; end
    bl = [find_system(ss, 'SearchDepth',1, 'BlockType','Inport'); ...
          find_system(ss, 'SearchDepth',1, 'BlockType','Goto', 'TagVisibility','local')];
    for k = 1:numel(bl)
        p = get_param(bl{k}, 'Position');  set_param(bl{k}, 'Position', p + [180 0 180 0]);
    end
end

function connect_root(m)
%  같은 이름의 출력 -> 입력 포트를 선으로 잇는다 (이미 있는 맨 위 선은 지우고 다시)
    old = find_system(m, 'SearchDepth',1, 'FindAll','on', 'Type','line');
    for k = 1:numel(old), try, delete_line(old(k)); catch, end, end
    subs = find_system(m, 'SearchDepth',1, 'BlockType','SubSystem');
    src = containers.Map();
    for k = 1:numel(subs)
        ph = get_param(subs{k}, 'PortHandles');
        outs = find_system(subs{k}, 'SearchDepth',1, 'BlockType','Outport');
        for j = 1:numel(outs)
            src(get_param(outs{j}, 'Name')) = ph.Outport(str2double(get_param(outs{j}, 'Port')));
        end
    end
    for k = 1:numel(subs)
        ph = get_param(subs{k}, 'PortHandles');
        ins = find_system(subs{k}, 'SearchDepth',1, 'BlockType','Inport');
        for j = 1:numel(ins)
            nm = get_param(ins{j}, 'Name');
            if ~isKey(src, nm), continue; end
            h = add_line(m, src(nm), ph.Inport(str2double(get_param(ins{j}, 'Port'))), 'autorouting','smart');
            set_param(h, 'Name', nm);
        end
    end
end

function collapse_all(m)
    subs = find_system(m, 'LookUnderMasks','all', 'FollowLinks','on', 'BlockType','SubSystem');
    for k = 1:numel(subs)
        if ~strcmp(get_param(subs{k}, 'SFBlockType'), 'NONE'), continue; end   % MATLAB Function · Chart 제외
        collapse_tags(subs{k});
    end
end

function collapse_tags(sys)
    gos = find_system(sys, 'SearchDepth',1, 'BlockType','Goto', 'TagVisibility','local');
    keep_in = endsWith(sys, '/RobotCtrl');   % 모드 상자 네 개가 같은 입력을 나눠 쓴다 -> 입력은 태그가 더 읽기 쉽다
    for k = 1:numel(gos)
        tag = get_param(gos{k}, 'GotoTag');
        if keep_in && startsWith(tag, 'i_'), continue; end
        frs = find_system(sys, 'SearchDepth',1, 'BlockType','From', 'GotoTag', tag);
        lg = get_param(gos{k}, 'LineHandles');
        if isempty(frs) || lg.Inport < 0, continue; end
        sp = get_param(lg.Inport, 'SrcPortHandle');
        dst = [];
        for j = 1:numel(frs)
            lf = get_param(frs{j}, 'LineHandles');
            if lf.Outport > 0
                dst = [dst; get_param(lf.Outport, 'DstPortHandle')]; %#ok<AGROW>
                delete_line(lf.Outport);
            end
            delete_block(frs{j});
        end
        delete_line(lg.Inport);  delete_block(gos{k});
        for j = 1:numel(dst)
            if dst(j) > 0, add_line(sys, sp, dst(j), 'autorouting','smart'); end
        end
    end
end

function tap_gotos(m)
%  기록 상자가 From 으로 꺼내 쓰는 신호를, 그 신호를 내보내는 상자 안 출력 포트 옆에 전역 Goto 로 건다
    need = {};
    for b = {'Logging','PenTrace','Animate'}
        if getSimulinkBlockHandle([m '/' b{1}]) < 0, continue; end
        fr = find_system([m '/' b{1}], 'BlockType','From');
        need = [need; cellstr(get_param(fr, 'GotoTag'))]; %#ok<AGROW>
    end
    need = unique(need);
    hg = find_system(m, 'LookUnderMasks','all', 'BlockType','Goto', 'TagVisibility','global');
    have = {};  if ~isempty(hg), have = cellstr(get_param(hg, 'GotoTag')); end
    subs = find_system(m, 'SearchDepth',1, 'BlockType','SubSystem');
    for k = 1:numel(subs)
        outs = find_system(subs{k}, 'SearchDepth',1, 'BlockType','Outport');
        for j = 1:numel(outs)
            nm = get_param(outs{j}, 'Name');
            if ~ismember(nm, need) || ismember(nm, have), continue; end
            lo = get_param(outs{j}, 'LineHandles');
            if lo.Inport < 0, continue; end
            sp = get_param(lo.Inport, 'SrcPortHandle');
            p = get_param(outs{j}, 'Position');
            w = max(70, 7*numel(nm) + 20);
            g = [subs{k} '/Tap_' nm];
            add_block('simulink/Signal Routing/Goto', g, 'GotoTag', nm, 'TagVisibility','global', ...
                      'Position', [p(1)-w-10 p(4)+20 p(1)-10 p(4)+42]);
            gp = get_param(g, 'PortHandles');
            add_line(subs{k}, sp, gp.Inport, 'autorouting','smart');
            have{end+1} = nm; %#ok<AGROW>
        end
    end
end

function arrange_subs(m)
    ws = warning('off', 'all');
    subs = find_system(m, 'LookUnderMasks','all', 'BlockType','SubSystem');
    for k = 1:numel(subs)
        if ~strcmp(get_param(subs{k}, 'SFBlockType'), 'NONE'), continue; end
        if endsWith(subs{k}, '/RobotCtrl'), continue; end   % M1 -> M4 순서를 빌더 자리 그대로
        try, Simulink.BlockDiagram.arrangeSystem(subs{k}); catch, end
    end
    %  층 배치 뒤 compute_u -> Dynamics 선이 dist_p 상수의 오른쪽 끝을 지나므로 상수를 왼쪽으로 민다
    %  After arrangeSystem the compute_u -> Dynamics line clips the dist_p constant; shift it left.
    c = [m '/RobotPlant/C_dist_p'];
    if getSimulinkBlockHandle(c) > 0
        set_param(c, 'Position', get_param(c, 'Position') - [50 0 50 0]);
    end
    warning(ws);
end

function align_root(m)
%  상자를 앞 상자의 출력 포트 높이에 맞춰 위아래로만 옮긴다 -> 맨 위 선이 수평 직선이 된다
    P = {'RosIn','zerokey_position','Nav','zerokey_position';
         'Nav','pose','Mission','pose';
         'Mission','WP_prev','RobotCtrl','WP_prev';
         'RobotCtrl','v_cmd','PwmMap','v_cmd';
         'RobotCtrl','v_cmd','InnerPI','v_cmd';      % 2 판
         'InnerPI','V_bar','PwmMap','V_bar';         % 2 판
         'PwmMap','motor_pwm','RosOut','motor_pwm';
         'PlantRosIn','motor_pwm_rx','RobotPlant','motor_pwm_rx';
         'RobotPlant','q_true','Sensors','q_true';
         'Sensors','zk_meas','PlantRosOut','zk_meas'};
    for k = 1:size(P,1)
        a = [m '/' P{k,1}];  b = [m '/' P{k,3}];
        if getSimulinkBlockHandle(a) < 0 || getSimulinkBlockHandle(b) < 0, continue; end
        ya = port_y(a, 'Outport', P{k,2});  yb = port_y(b, 'Inport', P{k,4});
        if isnan(ya) || isnan(yb), continue; end
        p = get_param(b, 'Position');  d = ya - yb;
        set_param(b, 'Position', p + [0 d 0 d]);
    end
    %  아래에 놓인 상자는 위 상자 바로 밑으로
    below(m, 'RobotCtrl', 'PenCtrl', 70);
    below(m, 'RobotPlant', 'StagePlant', 80);
    %  평가·기록은 사슬 전체 바로 아래 (check_flow: 종착 상자는 사슬보다 아래면 x 를 따지지 않는다)
    subs = find_system(m, 'SearchDepth',1, 'BlockType','SubSystem');
    yb = 0;
    for k = 1:numel(subs)
        if any(strcmp(get_param(subs{k},'Name'), {'PenTrace','Logging','Animate'})), continue; end
        p = get_param(subs{k}, 'Position');  yb = max(yb, p(4));
    end
    for b = {'PenTrace','Logging','Animate','n_eval'}
        h = getSimulinkBlockHandle([m '/' b{1}]);
        if h < 0, continue; end
        p = get_param(h, 'Position');
        if strcmp(b{1}, 'n_eval'), y0 = yb + 40; else, y0 = yb + 110; end
        set_param(h, 'Position', [p(1) y0 p(3) y0 + p(4) - p(2)]);
    end
end

function below(m, top, bot, gap)
    a = [m '/' top];  b = [m '/' bot];
    if getSimulinkBlockHandle(a) < 0 || getSimulinkBlockHandle(b) < 0, return; end
    pa = get_param(a, 'Position');  pb = get_param(b, 'Position');
    set_param(b, 'Position', [pb(1) pa(4)+gap pb(3) pa(4)+gap+pb(4)-pb(2)]);
end

function y = port_y(blk, kind, name)
    y = NaN;
    ports = find_system(blk, 'SearchDepth',1, 'BlockType', kind, 'Name', name);
    if isempty(ports), return; end
    ph = get_param(blk, 'PortHandles');  n = str2double(get_param(ports{1}, 'Port'));
    q = get_param(ph.(kind)(n), 'Position');  y = q(2);
end

function hold_inputs(ss, fcn, ts)
%  HOLD_INPUTS — From -> fcn 선마다 Zero-Order Hold 를 끼운다 (샘플 시간 ts)
%    MATLAB Function 이 연속 신호를 받으면 적분 스텝마다 실행된다. 입력을 모두 이산으로 만들면
%    블록이 ts 를 상속해 그 주기에만 실행된다.
%  Insert a zero-order hold on each From -> fcn line so the function block inherits ts.
    ph = get_param([ss '/' fcn], 'PortHandles');
    for k = 1:numel(ph.Inport)
        ln = get_param(ph.Inport(k), 'Line');
        if ln < 0, continue; end
        src = get_param(ln, 'SrcBlockHandle');
        if ~strcmp(get_param(src, 'BlockType'), 'From'), continue; end
        sp = get_param(src, 'Position');  q = port_xy(ss, fcn, 'Inport', k);
        delete_line(ln);
        set_param(src, 'Position', [sp(1)-60 sp(2) sp(3)-60 sp(4)]);
        zb = sprintf('ZOH_%d', k);
        add_block('simulink/Discrete/Zero-Order Hold', [ss '/' zb], 'SampleTime', ts, ...
                  'Position', [sp(3)-20 q(2)-12 sp(3)+20 q(2)+12], 'ShowName','off');
        add_line(ss, [get_param(src,'Name') '/1'], [zb '/1']);
        add_line(ss, [zb '/1'], sprintf('%s/%d', fcn, k));
    end
end

function snap_jogs(m)
%  자동 배선이 남긴 1–3 px 비틀림(사선)을 곧게 편다. 중간 꺾임점만 옮긴다
%  Straighten 1-3 px diagonal jogs left by auto-routing. Only interior bend points move.
    ln = find_system(m, 'FindAll','on', 'Type','line');
    for k = 1:numel(ln)
        p = get_param(ln(k), 'Points');  n = size(p, 1);  ch = false;
        for j = 2:n-1
            dx = p(j,1) - p(j-1,1);  dy = p(j,2) - p(j-1,2);
            if dx ~= 0 && abs(dx) <= 3 && dy ~= 0 && p(j+1,2) == p(j,2)
                p(j,1) = p(j-1,1);  ch = true;
            end
        end
        if ch, try, set_param(ln(k), 'Points', p); catch, end, end
    end
end
