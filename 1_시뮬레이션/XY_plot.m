function M = XY_plot(out, imgdir)
%XY_PLOT  XY_1_pen_sim 결과 그림 두 장 + 핵심 수치
%
%   M = XY_plot(out)            % 그림을 띄우고 img/ 에 저장, 수치를 돌려준다
%
%   그림 1  벽면: 목표 직선 · 로봇(태그) 경로 · 실제 펜 자국(보상) · 보상 없을 때 펜 자국
%   그림 2  시간: 펜 횡오차(보상 / 없음) · 스테이지 위치 · 미션 모드 · 펜 상태
%   벽면 그림은 벽을 정면에서 본 모습이다: 가로축 = -p2 (오른쪽 +), 세로축 = p1 (위 +)

if nargin < 2, imgdir = fullfile(fileparts(mfilename('fullpath')), 'img'); end
if ~isfolder(imgdir), mkdir(imgdir); end
lg = out.log;

[t, q]   = xy_getd(lg, 'q_true');
[~, pt]  = xy_getd(lg, 'pen_true');
[~, pn]  = xy_getd(lg, 'pen_nocomp');
[~, ep]  = xy_getd(lg, 'e_pen');
[~, en]  = xy_getd(lg, 'e_nocomp');
[~, on]  = xy_getd(lg, 'pen_on');
[~, md]  = xy_getd(lg, 'mode');
[~, sc]  = xy_getd(lg, 'stage_target_pos');
[~, sp]  = xy_getd(lg, 'stage_pos');
[~, ab]  = xy_getd(lg, 'lineAB');
A = ab(end,1:2);  B = ab(end,3:4);
on = on > 0.5;

%% 수치 — 펜이 내려가 있던 동안만
M.e_rms_mm    = 1000*sqrt(mean(ep(on).^2));
M.e_max_mm    = 1000*max(abs(ep(on)));
M.enc_rms_mm  = 1000*sqrt(mean(en(on).^2));
M.enc_max_mm  = 1000*max(abs(en(on)));
M.pen_time_s  = sum(on) * mean(diff(t));
M.t_end_s     = t(end);
M.stage_x_mm  = [min(sp(on,1)) max(sp(on,1))];
M.stage_y_mm  = [min(sp(on,2)) max(sp(on,2))];
fprintf('\n===== XY_1_pen_sim 결과 =====\n');
fprintf('  미션 종료 시각            %.1f s,  펜 내린 시간 %.1f s\n', M.t_end_s, M.pen_time_s);
fprintf('  펜 횡오차 (스테이지 보상)  RMS %.2f mm,  최대 %.2f mm\n', M.e_rms_mm, M.e_max_mm);
fprintf('  펜 횡오차 (보상 없음)      RMS %.2f mm,  최대 %.2f mm\n', M.enc_rms_mm, M.enc_max_mm);
fprintf('  스테이지 사용 범위  x %.1f ~ %.1f mm,  y %.1f ~ %.1f mm (행정 0 ~ 101.6)\n', ...
        M.stage_x_mm, M.stage_y_mm);

%% 그림 1 — 벽면
f1 = figure('Color','w','Position',[80 80 1100 560], 'Name','벽면 궤적');
ax = axes(f1); hold(ax,'on'); grid(ax,'on'); axis(ax,'equal');
plot(ax, -[A(2) B(2)], [A(1) B(1)], '-', 'Color',[0.6 0.75 1], 'LineWidth',6, 'DisplayName','목표 직선');
plot(ax, -q(:,2), q(:,1), 'k:', 'LineWidth',1.0, 'DisplayName','로봇 태그 경로');
pnn = pn; pnn(~on,:) = NaN;  ptt = pt; ptt(~on,:) = NaN;
plot(ax, -pnn(:,2), pnn(:,1), '-', 'Color',[0.55 0.55 0.55], 'LineWidth',1.5, 'DisplayName','펜 자국 — 보상 없음');
plot(ax, -ptt(:,2), ptt(:,1), 'r-', 'LineWidth',2.0, 'DisplayName','펜 자국 — 스테이지 보상');
plot(ax, -q(1,2), q(1,1), 'bo', 'MarkerFaceColor','b', 'DisplayName','출발');
xlabel(ax,'수평 -p_2 [m]  (벽 정면에서 오른쪽 +)'); ylabel(ax,'높이 p_1 [m]');
title(ax, sprintf('펜 횡오차 RMS  보상 %.2f mm  /  보상 없음 %.2f mm', M.e_rms_mm, M.enc_rms_mm));
legend(ax,'Location','southoutside','Orientation','horizontal');
exportgraphics(f1, fullfile(imgdir,'XY_1_result_wall.png'), 'Resolution',150);

%% 그림 2 — 시간 응답
f2 = figure('Color','w','Position',[120 60 1100 760], 'Name','시간 응답');
tl = tiledlayout(f2, 3, 1, 'TileSpacing','compact');
a1 = nexttile(tl); hold(a1,'on'); grid(a1,'on');
e1 = 1000*ep; e1(~on) = NaN;  e2 = 1000*en; e2(~on) = NaN;
plot(a1, t, e2, 'Color',[0.55 0.55 0.55], 'LineWidth',1.2, 'DisplayName','보상 없음');
plot(a1, t, e1, 'r', 'LineWidth',1.5, 'DisplayName','스테이지 보상');
ylabel(a1,'펜 횡오차 [mm]'); legend(a1,'Location','best'); title(a1,'펜이 내려가 있는 동안의 직선 횡오차');
a2 = nexttile(tl); hold(a2,'on'); grid(a2,'on');
plot(a2, t, sc(:,1), 'b--', t, sc(:,2), 'm--', 'LineWidth',1.0);
plot(a2, t, sp(:,1), 'b', t, sp(:,2), 'm', 'LineWidth',1.5);
yline(a2, [0 101.6], 'k:');
ylabel(a2,'스테이지 [mm]'); legend(a2, {'s_x 명령','s_y 명령','s_x 실제','s_y 실제'}, 'Location','best');
title(a2,'/stage/target_pos (점선) 와 /stage/pos (실선)', 'Interpreter','none');
a3 = nexttile(tl); hold(a3,'on'); grid(a3,'on');
stairs(a3, t, md, 'k', 'LineWidth',1.5);  stairs(a3, t, 4.5 + 0.4*on, 'r', 'LineWidth',1.5);
yticks(a3, [1 2 3 4 4.5 4.9]);
yticklabels(a3, {'1 Goto','2 Turn','3 Line','4 Stop','펜 업','펜 다운'});
xlabel(a3,'시간 [s]'); title(a3,'미션 모드 · 펜 상태');
exportgraphics(f2, fullfile(imgdir,'XY_1_result_time.png'), 'Resolution',150);
end

% ---------------------------------------------------------------------
function [t, D] = xy_getd(log, name)
%  out.log.<name> (timeseries) -> [시간 N x 1, 값 N x k]
ts = log.(name);
t  = ts.Time(:);
D  = squeeze(ts.Data);
if size(D,1) ~= numel(t), D = D.'; end
end
