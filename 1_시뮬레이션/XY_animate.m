function XY_animate(q_true, pen_true, pen_on, stage_pos, lineAB, e_pen, v_cmd, w_cmd, V_true, mode, t)
%XY_ANIMATE  XY_1_pen_sim 의 Animate 상자가 anim_Ts 마다 부르는 실시간 화면 (튜닝용 대시보드)
%   Live tuning dashboard called every anim_Ts seconds by the Animate box of XY_1_pen_sim.
%
%   화면 구성 (글자는 영어) / layout (English labels)
%     위        제목 + 시각 · 모드 · 펜 상태
%     왼쪽      강판 벽면 3D (AVT-3C STL, X·Y 캐리지, 펜 자국) / 펜 확대 3D
%     가운데    v 명령 vs 실제 [m/s] · omega 명령 vs 실제 [deg/s] · 펜 횡오차 e [mm]
%     오른쪽    스테이지 캐리지 위치 [mm] · 현재 게인 표
%   명령 = RobotCtrl 출력 (v_cmd, w_cmd), 실제 = 운동모델 참값 V_true = [v; omega].
%   두 선이 겹치면 안쪽 속도 루프 (PwmMap -> ESC -> 속도 PI -> 동역학) 가 명령을 따라가는 것이다.
%   v, omega: RobotCtrl command vs plant truth -> shows whether the inner speed loop tracks.
%
%   끄기: animate = 0.  갱신 주기: anim_Ts (기본 0.5 s, 작게 하면 부드럽고 느리다)
%   녹화: XY_record_live — 끝난 뒤 기록(out.log)을 이 함수에 다시 넣어 동영상을 만든다 (실행을 늦추지 않는다).
%   Turn off with animate = 0; update period anim_Ts; video: XY_record_live replays out.log afterwards.

persistent S
if isempty(S) || ~isvalid(S.fig) || t < S.t_last
    S = init_view(lineAB(1:2), lineAB(3:4), q_true);
end
S.t_last = t;

S.scene.update(q_true, stage_pos, pen_true, pen_on > 0.5);

%  안쪽 속도 루프: 명령 vs 실제 / inner speed loop: command vs actual
addpoints(S.hv(1), t, v_cmd);           addpoints(S.hv(2), t, V_true(1));
addpoints(S.hw(1), t, rad2deg(w_cmd));  addpoints(S.hw(2), t, rad2deg(V_true(2)));
set(S.tv, 'String', sprintf('Linear speed v:  cmd %+6.3f,  act %+6.3f  m/s', v_cmd, V_true(1)));
set(S.tw, 'String', sprintf('Yaw rate \\omega:  cmd %+6.2f,  act %+6.2f  deg/s', rad2deg(w_cmd), rad2deg(V_true(2))));

%  펜 횡오차 (펜 다운일 때만) / pen cross-track error, pen-down only
if pen_on > 0.5
    e = 1000*e_pen;
    addpoints(S.he, t, e);
    S.esum = S.esum + [e^2 1];
    S.emax = max(S.emax, abs(e));
    set(S.te, 'String', sprintf('Pen cross-track error e:  now %+5.2f,  RMS %4.2f,  max %4.2f  mm', ...
        e, sqrt(S.esum(1)/S.esum(2)), S.emax));
    pst = 'PEN DOWN';
else
    pst = 'pen up';
end
set([S.axv S.axw S.axe], 'XLim', [max(0, t-S.win), max(S.win, t)]);

%  스테이지 캐리지 / stage carriages
set(S.hs, 'XData', stage_pos(1), 'YData', stage_pos(2));
addpoints(S.hst, stage_pos(1), stage_pos(2));
set(S.tsg, 'String', {'XY stage carriages', sprintf('s_x = %5.1f mm,  s_y = %5.1f mm', stage_pos(1), stage_pos(2))});

mn = {'1  go to start', '2  align heading', '3  draw line', '4  stop'};
set(S.sub, 'String', sprintf('t = %5.1f s     |     mode %s     |     %s', ...
    t, mn{max(1, min(4, round(mode)))}, pst));
drawnow limitrate;
end

% =====================================================================
function S = init_view(A, B, q0)
S.fig = findobj('Type','figure', 'Tag','XY_live');
if isempty(S.fig)
    S.fig = figure('Tag','XY_live', 'Name','XY_1_pen_sim live dashboard', 'Color','w', 'NumberTitle','off', ...
                   'Position', [40 60 1600 900]);
end
clf(S.fig);

P.r_tag     = base_or('r_tag',  [-0.08; 0.06]);
P.r_pen0    = base_or('r_pen0', [0; 0]);
P.pen_d     = base_or('pen_d',  P.r_pen0 - P.r_tag);
P.stage_c   = base_or('stage_c', 50.8);
P.line_pre  = base_or('line_pre', 0.06);
P.line_post = base_or('line_post', 0.04);
P.stl_file  = base_or('stl_file', '');
trv         = base_or('stage_travel', 101.6);
S.win       = 60;                                % 시간축 창 [s]
navy = [0.12 0.23 0.37];

%  제목 띠 / title banner
annotation(S.fig, 'rectangle', [0 0.915 1 0.085], 'FaceColor',navy, 'EdgeColor','none');
annotation(S.fig, 'textbox', [0 0.952 1 0.045], 'EdgeColor','none', 'Color','w', ...
    'HorizontalAlignment','center', 'FontSize',16, 'FontWeight','bold', 'Interpreter','none', ...
    'String','Trajectory Tracking Control of a Magnetic-Adhesion Wall-Climbing Tracked Robot');
annotation(S.fig, 'textbox', [0 0.922 1 0.034], 'EdgeColor','none', 'Color',[0.85 0.90 0.97], ...
    'HorizontalAlignment','center', 'FontSize',12, 'FontAngle','italic', 'Interpreter','none', ...
    'String','Macro-Micro Pen Drawing on a Vertical Steel Wall with an XY Linear Stage   (AVT-3C + Zaber X-LSM100B)');
S.sub = annotation(S.fig, 'textbox', [0 0.882 0.46 0.032], 'EdgeColor','none', ...
    'HorizontalAlignment','center', 'FontSize',12, 'FontWeight','bold', 'Color',navy, 'String','');

%  왼쪽: 3D / left: 3D scene
left = uipanel(S.fig, 'Position',[0.00 0.00 0.46 0.88], 'BackgroundColor','w', 'BorderType','none');
S.scene = live_scene(left, P, A(:).', B(:).', q0(1:2).');

%  가운데: 시간 그래프 / centre: time histories
ck = [0 0 0];  cv = [0.85 0.33 0.10];  cw = [0.00 0.45 0.74];  ce = [0.80 0.10 0.10];
[S.axv, S.tv] = tplot(S.fig, [0.505 0.665 0.255 0.18], 'v [m/s]');
S.hv = [animatedline(S.axv, 'Color',ck, 'LineStyle','--', 'LineWidth',1.3), ...
        animatedline(S.axv, 'Color',cv, 'LineWidth',2)];
legend(S.axv, S.hv, {'command v_c', 'actual v'}, 'Location','northwest', 'FontSize',8);
[S.axw, S.tw] = tplot(S.fig, [0.505 0.375 0.255 0.18], '\omega [deg/s]');
S.hw = [animatedline(S.axw, 'Color',ck, 'LineStyle','--', 'LineWidth',1.3), ...
        animatedline(S.axw, 'Color',cw, 'LineWidth',2)];
legend(S.axw, S.hw, {'command \omega_c', 'actual \omega'}, 'Location','northwest', 'FontSize',8);
[S.axe, S.te] = tplot(S.fig, [0.505 0.075 0.255 0.20], 'e [mm]');
yline(S.axe, 0, 'k-');
S.he = animatedline(S.axe, 'Color',ce, 'LineWidth',1.8);
ylim(S.axe, [-5 5]);  xlabel(S.axe, 'time t [s]');
set(S.te, 'String', 'Pen cross-track error e  (waiting for pen down)');
S.esum = [0 0];  S.emax = 0;

%  오른쪽 위: 스테이지 / right top: stage carriages
S.axs = axes(S.fig, 'Position',[0.815 0.52 0.165 0.30]);
hold(S.axs,'on'); grid(S.axs,'on'); box(S.axs,'on'); axis(S.axs,'equal');
patch(S.axs, [0 trv trv 0], [0 0 trv trv], [0.93 0.97 0.93], 'EdgeColor',[0.3 0.68 0.32], 'LineWidth',1.5);
plot(S.axs, P.stage_c(1), P.stage_c(end), '+', 'Color',[0.3 0.68 0.32], 'MarkerSize',12, 'LineWidth',1.5);
S.hst = animatedline(S.axs, 'Color',[0.98 0.62 0.15], 'LineWidth',1.2);
S.hs  = plot(S.axs, NaN, NaN, 's', 'MarkerSize',11, 'MarkerFaceColor',[0.98 0.62 0.15], 'MarkerEdgeColor','k');
xlim(S.axs, [-5 trv+5]);  ylim(S.axs, [-5 trv+5]);
xlabel(S.axs, 's_x [mm] (along line)', 'FontSize',9);  ylabel(S.axs, 's_y [mm] (across line)', 'FontSize',9);
S.tsg = title(S.axs, 'XY stage carriages', 'FontWeight','bold', 'FontSize',10);

%  오른쪽 아래: 현재 게인 / right bottom: current gains
%  각 줄: {LaTeX 기호, 값, 단위, 머리줄 여부}. 기호는 README 4장 식과 같다 / symbols as in README ch. 4
G = {
  'Outer loop, line tracking (mode 3)', [], '', 1
  'k_x',               base_or('K_px',NaN),           's^{-1}', 0
  'k_y',               base_or('K_py',NaN),           'rad\,m^{-1}s^{-1}', 0
  'k_\psi',            base_or('K_ptheta',NaN),       's^{-1}', 0
  'v_r',               base_or('v_ref',NaN),          'm\,s^{-1}', 0
  'Outer loop, approach (modes 1, 4)', [], '', 1
  'k_\rho',            base_or('K_dist',NaN),         's^{-1}', 0
  'k_\alpha',          base_or('K_yaw',NaN),          's^{-1}', 0
  'Inner loop, speed PI (track force)', [], '', 1
  'K_{P,v}',           base_or('Kp_v',NaN),           'N\,s\,m^{-1}', 0
  'K_{I,v}',           base_or('Ki_v',NaN),           'N\,m^{-1}', 0
  'K_{P,\omega}',      base_or('Kp_w',NaN),           'N\,s\,m^{-1}', 0
  'K_{I,\omega}',      base_or('Ki_w',NaN),           'N\,m^{-1}', 0
  'Micro, XY stage servo', [], '', 1
  '\omega_{n,s}/2\pi', base_or('wn_s',NaN)/(2*pi),    'Hz', 0
  '\zeta_s',           base_or('ze_s',NaN),           '-', 0
  'k_c,\;k_{ci}',      [base_or('K_c',NaN) base_or('K_ci',NaN)], '-,\;s^{-1}', 0};
x0 = 0.785;  y = 0.405;  dy = 0.0215;
annotation(S.fig, 'rectangle', [0.775 0.025 0.22 0.40], 'Color',navy, 'LineWidth',1, 'FaceColor',[0.96 0.97 0.99]);
for k = 1:size(G,1)
    if G{k,4}
        y = y - 0.004;
        annotation(S.fig, 'textbox', [x0 y-dy 0.20 dy], 'String',G{k,1}, 'Interpreter','none', ...
            'EdgeColor','none', 'FontWeight','bold', 'FontSize',9, 'Color',navy, 'VerticalAlignment','middle');
    else
        v = G{k,2};
        if isscalar(v), vs = num2str(v, '%g'); else, vs = sprintf('%g,\\;%g', v); end
        annotation(S.fig, 'textbox', [x0+0.008 y-dy 0.20 dy], 'Interpreter','latex', 'FontSize',11, ...
            'EdgeColor','none', 'VerticalAlignment','middle', ...
            'String', sprintf('$%s = %s\\;\\;[\\mathrm{%s}]$', G{k,1}, vs, G{k,3}));
    end
    y = y - dy;
end
annotation(S.fig, 'textbox', [0.775 0.43 0.22 0.03], 'String','Current gains  (edit in XY_setup.m)', ...
    'Interpreter','none', 'EdgeColor','none', 'FontWeight','bold', 'FontSize',10, 'Color',navy);
S.t_last = -inf;
end

function [ax, tt] = tplot(fig, pos, yl)
ax = axes(fig, 'Position',pos);
hold(ax,'on'); grid(ax,'on'); box(ax,'on');
ylabel(ax, yl);  set(ax, 'FontSize',9);
tt = title(ax, '', 'FontWeight','bold', 'FontSize',10);
end

% =====================================================================
function S = live_scene(parent, P, A, B, pts)
%  벽면 + 로봇(STL) + XY 스테이지 + 펜 3D 장면. 화면 X = -p2, Y = p1, Z = 벽 바깥.
%  3D 장면 그림 규칙 (화면 X = -p2, Y = p1, Z = 벽 바깥).
W = @(p1, p2) [-p2(:), p1(:)];

th = atan2(B(2)-A(2), B(1)-A(1));  u = [cos(th) sin(th)];
Rl = [cos(th) -sin(th); sin(th) cos(th)];
rA = A - (Rl*P.pen_d).' - u*P.line_pre;
rB = B - (Rl*P.pen_d).' + u*P.line_post;
xy = W([pts(:,1); A(1); B(1); rA(1); rB(1)], [pts(:,2); A(2); B(2); rA(2); rB(2)]);
xl = [min(xy(:,1))-0.25, max(xy(:,1))+0.25];
yl = [min(xy(:,2))-0.25, max(xy(:,2))+0.25];

pn1 = uipanel(parent, 'Position',[0 0.42 1 0.58], 'BackgroundColor','w', 'Title','Vertical steel wall (3D): gravity points down, magnetic tracks hold the robot', 'FontSize',11);
pn2 = uipanel(parent, 'Position',[0 0.00 1 0.42], 'BackgroundColor','w', ...
              'Title','Pen close-up: green = XY stage carriages (moving), orange = pen holder', 'FontSize',11);
ax1 = axes(pn1, 'Position',[0 0 1 1]);
ax2 = axes(pn2, 'Position',[0 0 1 1]);
for ax = [ax1 ax2]
    hold(ax,'on'); axis(ax,'equal'); axis(ax,'off');
    set(ax, 'Projection','perspective', 'CameraUpVector',[0 1 0]);
    patch(ax, xl([1 2 2 1]), yl([1 1 2 2]), [0 0 0 0], [0.93 0.89 0.82], 'EdgeColor','none', 'FaceLighting','none');
    for g = ceil(xl(1)*10)/10 : 0.1 : xl(2), plot3(ax, [g g], yl, [1 1]*1e-4, 'Color',[0.85 0.80 0.72]); end
    for g = ceil(yl(1)*10)/10 : 0.1 : yl(2), plot3(ax, xl, [g g], [1 1]*1e-4, 'Color',[0.85 0.80 0.72]); end
end

a = W(A(1),A(2));  b = W(B(1),B(2));
cA = [0.05 0.60 0.15];  cB = [0.85 0.10 0.10];
T = {'Clipping','off', 'HorizontalAlignment','center', 'FontWeight','bold', 'BackgroundColor','w', 'Margin',1};
for ax = [ax1 ax2]
    z = 2.5e-3;
    plot3(ax, [a(1) b(1)], [a(2) b(2)], [2e-4 2e-4], '--', 'Color',[0.3 0.5 1], 'LineWidth',1.2);
    plot3(ax, a(1), a(2), z, 'o', 'MarkerSize',11, 'LineWidth',2.5, 'MarkerEdgeColor',cA, 'MarkerFaceColor','w');
    plot3(ax, b(1), b(2), z, 's', 'MarkerSize',11, 'LineWidth',2.5, 'MarkerEdgeColor',cB, 'MarkerFaceColor','w');
end
text(ax1, a(1), a(2)-0.09, 0.25, sprintf('A (%.2f, %.2f)', A(1), A(2)), 'Color',cA, 'FontSize',11, T{:});
text(ax1, b(1), b(2)-0.09, 0.25, sprintf('B (%.2f, %.2f)', B(1), B(2)), 'Color',cB, 'FontSize',11, T{:});
text(ax2, a(1), a(2)-0.03, 0.01, 'A', 'Color',cA, 'FontSize',13, T{:});
text(ax2, b(1), b(2)-0.03, 0.01, 'B', 'Color',cB, 'FontSize',13, T{:});

[F, Vb] = robot_mesh(P.stl_file);
g1 = make_robot(ax1, F, Vb, P);
g2 = make_robot(ax2, F, Vb, P);
tr1 = plot3(ax1, NaN, NaN, NaN, 'r-', 'LineWidth',2.5);
tr2 = plot3(ax2, NaN, NaN, NaN, 'r-', 'LineWidth',3.0);
for ax = [ax1 ax2], camlight(ax,'headlight'); lighting(ax,'gouraud'); end

cx = mean(xl); cy = mean(yl);
camtarget(ax1, [cx cy 0]);  campos(ax1, [cx+0.20, cy-0.55, 2.6]);
camva(ax1, 2*atand(0.36*diff(xl)/2.7));
camva(ax2, 24);

trail = nan(0,2);
S.update = @update;

    function update(q, sp, pen, on)
        q = q(:);  psi = q(3);
        R = [cos(psi) -sin(psi); sin(psi) cos(psi)];
        ctr = q(1:2) - R*P.r_tag(:);
        M  = makehgtform('translate',[W(ctr(1), ctr(2)) 0]) * makehgtform('zrotate', psi + pi/2);
        sp = sp(:).';  c0 = P.stage_c(:).';  if isscalar(c0), c0 = [c0 c0]; end
        ds = (sp - c0) / 1000;
        for g = [g1 g2]
            set(g.body, 'Matrix', M);
            set(g.sx,   'Matrix', makehgtform('translate',[ds(1) 0 0]));
            set(g.sy,   'Matrix', makehgtform('translate',[0 ds(2) 0]));
        end
        if on, trail(end+1,:) = W(pen(1), pen(2)); else, trail(end+1,:) = [NaN NaN]; end
        n = size(trail,1);
        set([tr1 tr2], 'XData',trail(:,1), 'YData',trail(:,2), 'ZData',1.5e-3*ones(n,1));
        pp = W(pen(1), pen(2));
        camtarget(ax2, [pp 0.05]);  campos(ax2, [pp(1)+0.12, pp(2)-0.30, 0.85]);
    end
end

function [F, Vb] = robot_mesh(f)
%  STL 이 있으면 줄여서 쓰고, 없으면 420 x 343 x 120 mm 상자
if ~isempty(f) && isfile(f)
    TR = stlread(f);
    [F, V] = reducepatch(TR.ConnectivityList, TR.Points, 0.15);
    c  = [mean([min(V(:,1)) max(V(:,1))]), min(V(:,2)), mean([min(V(:,3)) max(V(:,3))])];
    Vb = [ (V(:,1)-c(1)), -(V(:,3)-c(3)), (V(:,2)-c(2)) ] / 1000;
else
    s = [0.343 0.420 0.100];
    Vb = [s(1)/2*[-1 1 1 -1 -1 1 1 -1]', s(2)/2*[-1 -1 1 1 -1 -1 1 1]', s(3)*[0 0 0 0 1 1 1 1]'];
    F  = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
end
end

function g = make_robot(ax, F, Vb, P)
%  hgtransform 사슬: 차체 -> X 캐리지 (sx) -> Y 캐리지 (sy) -> 펜
g.body = hgtransform('Parent', ax);
patch('Parent',g.body, 'Faces',F, 'Vertices',Vb, 'FaceColor',[0.55 0.57 0.60], ...
      'EdgeColor','none', 'FaceLighting','gouraud', 'AmbientStrength',0.45);
px = P.r_pen0(1);  py = P.r_pen0(2);
green = [0.30 0.68 0.32];  orange = [0.98 0.62 0.15];
box3(g.body, [px py-0.090 0.112], [0.210 0.045 0.021], green*0.85);
g.sx = hgtransform('Parent', g.body);
box3(g.sx, [px py-0.090 0.127], [0.060 0.045 0.010], green);
box3(g.sx, [px py-0.020 0.143], [0.045 0.210 0.021], green*0.85);
g.sy = hgtransform('Parent', g.sx);
box3(g.sy, [px py 0.159], [0.045 0.060 0.010], green);
box3(g.sy, [px py 0.170], [0.050 0.050 0.008], orange);
[xc, yc, zc] = cylinder(0.006, 16);
surf(xc+px, yc+py, zc*0.20, 'Parent',g.sy, 'FaceColor',[0.1 0.1 0.1], 'EdgeColor','none');
end

function box3(parent, c, s, col)
x = c(1) + s(1)/2*[-1 1 1 -1 -1 1 1 -1];
y = c(2) + s(2)/2*[-1 -1 1 1 -1 -1 1 1];
z = c(3) + s(3)/2*[-1 -1 -1 -1 1 1 1 1];
f = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
patch('Parent',parent, 'Faces',f, 'Vertices',[x' y' z'], 'FaceColor',col, ...
      'EdgeColor',[0.2 0.3 0.2], 'FaceLighting','gouraud');
end

function v = base_or(name, default)
try
    v = evalin('base', name);
catch
    v = default;
end
end
