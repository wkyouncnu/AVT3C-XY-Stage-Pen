% XY_COMPARE  설계 선택 비교 — 주행 속도 · 스테이지 대역 · 지연 보상 · 스테이지 중앙 유지
%
%   >> XY_compare            % 모든 경우를 돌린다 (한 경우 30–60 s, ROS 2 루프백 실시간)
%   >> XY_compare_only = [1 4];  XY_compare    % 일부만
%
%   경우마다 XY_setup 기본값에서 한두 값만 바꾸고 XY_1_pen_sim 을 돌린다.
%   결과: logs/compare.mat (표 C, 시계열 S), img/XY_compare_*.png  (그림은 XY_compare_plot)
%
%   Compare design choices: robot speed, stage bandwidth, ZeroKey delay prediction, stage centering.
%   Each case changes one or two values from XY_setup and runs XY_1_pen_sim (ROS 2 loopback, real time).

here = fileparts(mfilename('fullpath'));  cd(here);
XY_setup;
if ~isfile('XY_1_pen_sim.slx'), build_xy_models(1); end
load_system('XY_1_pen_sim');

% 이름, 설명, 바꿀 값 {변수, 값, ...}
cases = {
  'base',     '기준 (30 mm/s, 스테이지 20 Hz, 명령 25 Hz)', {};
  'v15',      '속도 15 mm/s',                              {'v_ref',0.015};
  'v60',      '속도 60 mm/s',                              {'v_ref',0.060};
  'v60pred',  '속도 60 mm/s + 지연 보상',                  {'v_ref',0.060,'tau_pred',zk_delay};
  'pred',     '지연 보상 (tau = ZeroKey 지연 50 ms)',      {'tau_pred',zk_delay};
  'loose',    '스테이지 느슨 (대역 2 Hz, 명령 5 Hz)',     {'wn_s',2*pi*2,'stage_cmd_Ts',0.2};
  'center',   '스테이지 중앙 유지 (K_c 0.5, K_ci 0.3)',   {'K_c',0.5,'K_ci',0.3};
  'all',      '지연 보상 + 중앙 유지',                    {'tau_pred',zk_delay,'K_c',0.5,'K_ci',0.3};
  'zk05',     'ZeroKey 잡음 절반 (0.5 mm)',                {'zk_sigma',0.0005};
  'ideal',    '센서 잡음·드리프트 0 (동역학만 남김)',      {'zk_sigma',0,'imu_sigma',0,'imu_drift',0};
};
if ~exist('XY_compare_only','var') || isempty(XY_compare_only), XY_compare_only = 1:size(cases,1); end

f = fullfile(here,'logs','compare.mat');
if ~isfolder(fileparts(f)), mkdir(fileparts(f)); end
if isfile(f), load(f, 'C', 'S'); else, C = struct([]); S = struct([]); end

for k = XY_compare_only
    XY_setup;                                   % 매 경우 기본값으로 되돌린 뒤
    kv = cases{k,3};
    for j = 1:2:numel(kv), eval(sprintf('%s = kv{j+1};', kv{j})); end
    %  스테이지 대역을 바꾸면 서보 게인을 다시 계산한다 (XY_setup 7 절과 같은 식)
    Kp_s = [m_sx; m_sy]*wn_s^2;  Kd_s = 2*ze_s*[m_sx; m_sy]*wn_s;  Ki_s = Kp_s*wn_s/5;
    stg_p = [m_sx; m_sy; c_s; Kp_s; Kd_s; Ki_s; g_acc];
    T_run = 40 + 0.7/v_ref;
    fprintf('\n[%d/%d] %s — %s\n', k, size(cases,1), cases{k,1}, cases{k,2});
    out = sim('XY_1_pen_sim');
    lg = out.log;
    [t, ep] = xy_getd(lg, 'e_pen');   [~, en] = xy_getd(lg, 'e_nocomp');
    [~, on] = xy_getd(lg, 'pen_on');  [~, sp] = xy_getd(lg, 'stage_pos');
    on = on > 0.5;
    r.name = cases{k,1};  r.desc = cases{k,2};
    r.e_rms = 1000*sqrt(mean(ep(on).^2));   r.e_max = 1000*max(abs(ep(on)));
    r.n_rms = 1000*sqrt(mean(en(on).^2));   r.n_max = 1000*max(abs(en(on)));
    r.sy_mean = mean(sp(on,2)) - stage_c;   r.sy_rng = [min(sp(on,2)) max(sp(on,2))];
    r.sx_rng  = [min(sp(on,1)) max(sp(on,1))];
    r.pen_time = sum(on)*mean(diff(t));
    fprintf('    펜 RMS %.2f mm (최대 %.2f) | 보상 없음 RMS %.2f | 스테이지 y %.1f–%.1f mm | 펜 시간 %.1f s\n', ...
            r.e_rms, r.e_max, r.n_rms, r.sy_rng, r.pen_time);
    if isempty(C), C = r; else
        i = find(strcmp({C.name}, r.name));  if isempty(i), i = numel(C) + 1; end
        C(i) = r;
    end
    s.name = r.name;  t0 = t(find(on,1));
    s.t = t(on) - t0;  s.e = 1000*ep(on);  s.en = 1000*en(on);  s.sy = sp(on,2);
    if isempty(S), S = s; else
        i = find(strcmp({S.name}, s.name));  if isempty(i), i = numel(S) + 1; end
        S(i) = s;
    end
    save(f, 'C', 'S');
end
XY_setup;                                       % 작업 공간을 기본값으로 되돌린다
clear XY_compare_only
fprintf('\n저장: %s\n', f);

%% 지역 함수
function [t, D] = xy_getd(log, name)
%  out.log.<name> (timeseries) -> [시간 N x 1, 값 N x k]
ts = log.(name);
t  = ts.Time(:);
D  = squeeze(ts.Data);
if size(D,1) ~= numel(t), D = D.'; end
end
