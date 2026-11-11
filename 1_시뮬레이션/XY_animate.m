function XY_animate(q_true, pen_true, pen_on, stage_pos, lineAB, t)
%XY_ANIMATE  XY_1_pen_sim 의 Animate 상자가 0.1 s 마다 부르는 실시간 화면
%   Live view called every 0.1 s by the Animate box of XY_1_pen_sim.
%
%   벽을 정면에서 본 모습 (가로 = -p2 오른쪽 +, 세로 = p1 위 +)
%     파랑 굵은 선  그릴 직선 A -> B
%     회색 사각형   로봇 외형 420 x 343 mm,  주황 화살표 = heading
%     초록 점선     XY 스테이지 행정 (펜이 움직일 수 있는 범위)
%     빨강          실제 펜 자국 (펜 다운일 때만)
%   끄기: base workspace 에서  animate = 0  (시뮬레이션이 빨라진다)
%   Front view of the wall. Turn off with animate = 0 in the base workspace.

persistent S
A = lineAB(1:2);  B = lineAB(3:4);
if isempty(S) || ~isvalid(S.fig) || t < S.t_last
    S = init_view(A, B, q_true);
end
S.t_last = t;

R = [cos(q_true(3)) -sin(q_true(3)); sin(q_true(3)) cos(q_true(3))];
c = q_true(1:2) - R * S.r_tag;                          % 로봇 중심 (태그 -> 중심)
set(S.body,  'XData', -(c(2) + R(2,:)*S.box), 'YData', c(1) + R(1,:)*S.box);
hd = c + R * [0.25; 0];
set(S.head,  'XData', -[c(2) hd(2)], 'YData', [c(1) hd(1)]);
pc = q_true(1:2) + R * S.pen_d;                         % 스테이지 중앙일 때 펜
set(S.stage, 'XData', -(pc(2) + R(2,:)*S.sbox), 'YData', pc(1) + R(1,:)*S.sbox);
set(S.pen,   'XData', -pen_true(2), 'YData', pen_true(1));
if pen_on > 0.5
    addpoints(S.trace, -pen_true(2), pen_true(1));
end
u = (B - A) / max(norm(B - A), 1e-9);  n = [-u(2); u(1)];
e = 1000 * dot(n, pen_true(:) - A);
if pen_on > 0.5, pst = '펜 다운'; else, pst = '펜 업'; end
set(S.ttl, 'String', sprintf('t = %5.1f s   |   %s   |   직선 오차 %+.2f mm   |   스테이지 (%.1f, %.1f) mm', ...
                             t, pst, e, stage_pos(1), stage_pos(2)));
drawnow limitrate;
end

function S = init_view(A, B, q0)
S.fig = findobj('Type','figure', 'Tag','XY_live');
if isempty(S.fig)
    S.fig = figure('Tag','XY_live', 'Name','XY_1_pen_sim 실시간', 'Color','w', 'NumberTitle','off', ...
                   'Position', [80 120 900 620]);
end
clf(S.fig);
ax = axes(S.fig);  hold(ax, 'on');  grid(ax, 'on');  axis(ax, 'equal');
P = [A B q0(1:2)];
pad = 0.30;
xlim(ax, sort(-[min(P(2,:)) - pad, max(P(2,:)) + pad]));
ylim(ax, [min(P(1,:)) - pad, max(P(1,:)) + pad]);
xlabel(ax, '수평 -p_2 [m]  (벽 정면에서 오른쪽 +)');  ylabel(ax, '높이 p_1 [m]');
plot(ax, -[A(2) B(2)], [A(1) B(1)], '-', 'Color', [0.6 0.75 1], 'LineWidth', 6);
text(ax, -A(2), A(1) - 0.04, 'A', 'FontWeight','bold');
text(ax, -B(2), B(1) - 0.04, 'B', 'FontWeight','bold');
S.r_tag = base_or('r_tag', [-0.08; 0.06]);
S.pen_d = base_or('pen_d', [0.08; -0.06]);
L = 0.420;  W = 0.343;  s = base_or('stage_travel', 101.6) / 2000;
S.box  = [ L/2  L/2 -L/2 -L/2  L/2;  W/2 -W/2 -W/2  W/2  W/2];
S.sbox = [ s  s -s -s  s;  s -s -s  s  s];
S.body  = plot(ax, nan, nan, '-', 'Color', [0.35 0.35 0.35], 'LineWidth', 1.5);
S.head  = plot(ax, nan, nan, '-', 'Color', [0.9 0.5 0.1], 'LineWidth', 2);
S.stage = plot(ax, nan, nan, '--', 'Color', [0.2 0.6 0.2], 'LineWidth', 1);
S.trace = animatedline(ax, 'Color', 'r', 'LineWidth', 2);
S.pen   = plot(ax, nan, nan, 'o', 'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'k', 'MarkerSize', 6);
S.ttl   = title(ax, '', 'FontWeight', 'normal');
S.t_last = -inf;
end

function v = base_or(name, default)
try
    v = evalin('base', name);
catch
    v = default;
end
end
