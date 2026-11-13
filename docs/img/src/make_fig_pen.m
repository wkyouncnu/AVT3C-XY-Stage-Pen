function make_fig_pen()
%MAKE_FIG_PEN  펜 기구학 보상(PenComp) 설명 그림 네 장. 모든 길이 mm, 실제 축척
%   Four figures that explain the pen compensation (PenComp). All lengths in mm, to scale.
%     ../fig_pen_body.png      바디 좌표만: pen_d, R'(p_des - pT), e_b 삼각형
%                              body frame only: the triangle pen_d + e_b = R'(p_des - pT)
%     ../fig_pen_wall.png      벽면 좌표만: A, B, u, n, sigma, c0(pc), p_des, e0
%                              wall frame only: A, B, u, n, sigma, c0 (pc), p_des, e0
%     ../fig_pen_combined.png  같은 삼각형을 벽면과 바디에서 나란히 (R' 가 하는 일)
%                              the same triangle in the wall and body frames (what R' does)
%     ../fig_pen_stage.png     e_b -> s_cmd: x 1000 과 + stage_c
%                              e_b -> s_cmd: the factor 1000 and the offset stage_c
%   수치는 이론 6.4 절의 예와 같다: p_hat = (-0.200, 0.345) m, psi = 3 deg, A = (-0.70, 0.30), B = (0.50, 0.30)
%   Numbers follow the worked example of section 6.4.
%   화면 밖 렌더링(Visible off) 만 쓴다 / offscreen rendering only.

here = fileparts(mfilename('fullpath'));
E = example();
body(E,     fullfile(here, '..', 'fig_pen_body.png'));
wall(E,     fullfile(here, '..', 'fig_pen_wall.png'));
combined(E, fullfile(here, '..', 'fig_pen_combined.png'));
stage(E,    fullfile(here, '..', 'fig_pen_stage.png'));
end

% =====================================================================
function E = example()
%  예제 값 [mm] / example values [mm]
E.A = [-700; 300];  E.B = [500; 300];
E.p = [-200; 345];  E.ps = deg2rad(3);
E.d = [80; -60];    E.rtag = [-80; 60];  E.sc = 50.8;  E.h = 50.8;
E.R = [cos(E.ps) -sin(E.ps); sin(E.ps) cos(E.ps)];
E.L = norm(E.B - E.A);  E.u = (E.B - E.A)/E.L;  E.n = [-E.u(2); E.u(1)];
E.c0 = E.p + E.R*E.d;                    % pc  : 스테이지 가운데일 때 펜 / pen with the stage centred
E.sg = E.u'*(E.c0 - E.A);                % sigma
E.e0 = E.n'*(E.c0 - E.A);                % 보상 없는 횡오차 / cross-track error without compensation
E.pst = E.A + E.sg*E.u;                  % p_des
E.g = E.pst - E.p;                       % p_des - pT (벽면 / wall)
E.gB = E.R'*E.g;                         % R'(p_des - pT) (바디 / body)
E.eb = E.gB - E.d;                       % e_b
E.s = E.sc + E.eb;                       % s_cmd [mm] (eb 가 이미 mm)
E.c = E.p - E.R*E.rtag;                  % 로봇 중심 / robot centre
E.col.navy = [0.12 0.23 0.37];  E.col.gry = [0.45 0.45 0.45];  E.col.red = [0.80 0.10 0.10];
E.col.grn = [0.15 0.55 0.25];   E.col.pur = [0.55 0.25 0.65];   E.col.org = [0.90 0.45 0.05];
E.col.blu = [0.35 0.55 0.90];   E.col.stg = [0.10 0.55 0.60];
end

% =====================================================================
function body(E, f)
%  (a) 로봇 전체 (바디 좌표), (b) 스테이지 가운데 확대
C = E.col;
fig = figure('Color','w', 'Position',[50 50 1600 760], 'Visible','off');
% ---- (a)
ax = axes(fig, 'Position',[0.01 0.03 0.55 0.92]);  hold(ax,'on');  axis(ax,'equal');  axis(ax,'off');
T = @(x,y,s,varargin) text(ax, x, y, s, 'Interpreter','latex', 'FontSize',14, varargin{:});
robot_body(ax, [0;0], eye(2), C);
plot(ax, E.h*[-1 1 1 -1 -1], E.h*[-1 -1 1 1 -1], '-', 'Color',C.stg, 'LineWidth',1.6);
T(-E.h+2, -E.h-12, 'stage travel $\pm 50.8$ mm', 'Color',C.stg, 'FontSize',12);
arw(ax, [0 0], [250 0], 'k', 1.4, 9);  T(255, -2, '$x_B$ (forward)');
arw(ax, [0 0], [0 205], 'k', 1.4, 9);  T(-20, 218, '$y_B$ (left)');
t = E.rtag;  tg = t + E.gB;
plot(ax, t(1), t(2), 's', 'MarkerSize',12, 'MarkerFaceColor',[0.2 0.2 0.2], 'MarkerEdgeColor','k');
T(t(1)-150, t(2)-22, '$\hat p$ (tag), body $(-80.0,\ 60.0)$');
arw(ax, t', [0 0], C.pur, 2.0, 9);
T(-215, 10, sprintf('$d = (%.1f,\\ %.1f)$', E.d), 'Color',C.pur, 'FontSize',15);
arw(ax, t', tg', C.grn, 2.0, 9);
T(45, 100, sprintf('$R^{\\top}(\\hat\\psi)\\,(p^{\\star} - \\hat p) = (%.1f,\\ %.1f)$', E.gB), 'Color',C.grn, 'FontSize',14);
% 성분 (점선) / components (dotted)
plot(ax, [t(1) t(1)+E.d(1)], [t(2) t(2)], ':', 'Color',C.pur, 'LineWidth',1.2);
plot(ax, [t(1) t(1)+E.gB(1)], [t(2)+4 t(2)+4], ':', 'Color',C.grn, 'LineWidth',1.2);
T(t(1)-10, t(2)+16, '$x_B$ parts: 80.0 (purple $d$), 80.6 (green)', 'FontSize',11);
plot(ax, [0 0], [t(2) 0], ':', 'Color',C.pur, 'LineWidth',1.2);
T(60, 40, '$y_B$ parts: $-60.0$ (purple $d$), $-49.3$ (green)', 'FontSize',11);
plot(ax, 0, 0, 'o', 'MarkerSize',9, 'MarkerFaceColor','w', 'MarkerEdgeColor','k', 'LineWidth',1.2);
T(6, -16, 'stage centre ($s = s_c$): pen at $c_0$', 'FontSize',12);
plot(ax, tg(1), tg(2), 'd', 'MarkerSize',9, 'MarkerFaceColor',C.grn, 'MarkerEdgeColor','k');
rectangle(ax, 'Position',[-18 -10 36 34], 'EdgeColor',C.gry, 'LineStyle','--');
T(-215, -205, 'body frame: the picture is the same whatever way the robot faces on the wall', 'FontSize',12);
T(-230, 245, '(a) body frame $\mathcal{B}$, to scale [mm]', 'FontSize',15);
xlim(ax, [-235 330]);  ylim(ax, [-215 255]);
% ---- (b) 확대 / zoom
ax = axes(fig, 'Position',[0.60 0.10 0.38 0.82]);  hold(ax,'on');  axis(ax,'equal');  box(ax,'on');
T = @(x,y,s,varargin) text(ax, x, y, s, 'Interpreter','latex', 'FontSize',14, varargin{:});
k = 18 / norm(E.d);                     % 확대 그림에 보이는 pen_d 끝 부분 / last part of pen_d
plot(ax, [-E.d(1)*k 0], [-E.d(2)*k 0], '-', 'Color',C.pur, 'LineWidth',2.0);
arw(ax, (-E.d*k)', [0 0], C.pur, 2.0, 1.1);
a0 = tg - E.gB*(22/norm(E.gB));
arw(ax, a0', tg', C.grn, 2.0, 1.1);
arw(ax, [0 0], E.eb', C.org, 2.6, 1.1);
plot(ax, [0 E.eb(1)], [0 0], ':', 'Color',C.org);  plot(ax, [E.eb(1) E.eb(1)], [0 E.eb(2)], ':', 'Color',C.org);
plot(ax, 0, 0, 'o', 'MarkerSize',10, 'MarkerFaceColor','w', 'MarkerEdgeColor','k', 'LineWidth',1.2);
plot(ax, tg(1), tg(2), 'd', 'MarkerSize',11, 'MarkerFaceColor',C.grn, 'MarkerEdgeColor','k');
T(-15.5, 5, '$d$ (from $\hat p$)', 'Color',C.pur, 'FontSize',13);
T(-7.5, 20, '$R^{\top}(\hat\psi)(p^{\star} - \hat p)$ (from $\hat p$)', 'Color',C.grn, 'FontSize',13);
T(1.5, 5.5, sprintf('$\\Delta s = (%.2f,\\ %.2f)$ mm', E.eb), 'Color',C.org, 'FontSize',15);
T(1.2, -1.6, 'stage centre ($c_0$)', 'FontSize',12);
T(tg(1)+1.0, tg(2)+0.6, '$p^{\star}$: where the pen must go', 'Color',C.grn, 'FontSize',12);
T(-16.5, -7.5, '$d + \Delta s = R^{\top}(\hat\psi)\,(p^{\star} - \hat p)$', 'FontSize',15);
xlabel(ax, '$x_B$ [mm] (from the stage centre)', 'Interpreter','latex', 'FontSize',13);
ylabel(ax, '$y_B$ [mm]', 'Interpreter','latex', 'FontSize',13);
set(ax, 'TickLabelInterpreter','latex', 'FontSize',11);
xlim(ax, [-18 16]);  ylim(ax, [-10 24]);
title(ax, '(b) zoom of the dashed box', 'Interpreter','latex', 'FontSize',15);
exportgraphics(fig, f, 'Resolution',200);  close(fig);
end

% =====================================================================
function wall(E, f)
%  (a) 직선 전체와 sigma, (b) 펜 근처 확대 (e0, p_des)
C = E.col;
fig = figure('Color','w', 'Position',[50 50 1700 760], 'Visible','off');
% ---- (a)
ax = axes(fig, 'Position',[0.01 0.05 0.58 0.90]);  hold(ax,'on');  axis(ax,'equal');  axis(ax,'off');
T = @(x,y,s,varargin) text(ax, x, y, s, 'Interpreter','latex', 'FontSize',14, varargin{:});
robot_body(ax, E.c, E.R, C);
plot(ax, [E.A(1) E.B(1)], [E.A(2) E.B(2)], '-', 'Color',C.blu, 'LineWidth',4);
plot(ax, E.A(1), E.A(2), 'o', 'MarkerSize',9, 'MarkerFaceColor',C.navy, 'MarkerEdgeColor','k');
plot(ax, E.B(1), E.B(2), 'o', 'MarkerSize',9, 'MarkerFaceColor',C.navy, 'MarkerEdgeColor','k');
T(E.A(1)-25, E.A(2)+28, '$A\ (-700,\ 300)$', 'Color',C.navy);  T(E.B(1)-90, E.B(2)+28, '$B\ (500,\ 300)$', 'Color',C.navy);
arw(ax, E.A', (E.A + 150*E.u)', C.navy, 2.0, 16);  T(E.A(1)+70, E.A(2)-28, '$u = (1,\ 0)$', 'Color',C.navy);
arw(ax, E.A', (E.A + 120*E.n)', C.navy, 2.0, 16);  T(E.A(1)+8, E.A(2)+128, '$n = (0,\ 1)$', 'Color',C.navy);
plot(ax, E.p(1), E.p(2), 's', 'MarkerSize',11, 'MarkerFaceColor',[0.2 0.2 0.2], 'MarkerEdgeColor','k');
plot(ax, E.c0(1), E.c0(2), 'o', 'MarkerSize',8, 'MarkerFaceColor','w', 'MarkerEdgeColor','k', 'LineWidth',1.2);
plot(ax, E.pst(1), E.pst(2), 'd', 'MarkerSize',9, 'MarkerFaceColor',C.grn, 'MarkerEdgeColor','k');
% sigma 치수선 / dimension line
yd = 70;
plot(ax, [E.A(1) E.A(1)], [E.A(2)-15 yd-10], ':', 'Color',C.gry);
plot(ax, [E.pst(1) E.pst(1)], [E.c0(2)-15 yd-10], ':', 'Color',C.gry);
arw(ax, [E.A(1) yd], [E.pst(1) yd], 'k', 1.3, 16);  arw(ax, [E.pst(1) yd], [E.A(1) yd], 'k', 1.3, 16);
T(-640, yd-28, sprintf('$\\sigma = u^{\\top}(c_0 - A) = %.1f$ mm  (how far along the line)', E.sg));
plot(ax, [E.pst(1) E.B(1)], [yd yd], ':', 'Color',C.gry);  plot(ax, [E.B(1) E.B(1)], [yd-10 E.B(2)-15], ':', 'Color',C.gry);
T(-20, yd-28, sprintf('$L - \\sigma = %.0f$ mm to go', E.L - E.sg), 'FontSize',12, 'Color',C.gry);
T(-60, 520, '$\hat\psi = 3^{\circ}$: nose $3^{\circ}$ above the line', 'FontSize',13);
arw(ax, [-760 470], [-760 400], C.gry, 1.6, 14);  T(-748, 440, '$g$', 'Color',C.gry);
rectangle(ax, 'Position',[-230 265 140 95], 'EdgeColor',C.gry, 'LineStyle','--');
arw(ax, [330 450], [430 450], 'k', 1.3, 14);  T(440, 450, '$x$ (right)', 'FontSize',12);
arw(ax, [330 450], [330 540], 'k', 1.3, 14);  T(340, 545, '$y$ (up)', 'FontSize',12);
T(-770, 580, '(a) wall frame $\mathcal{W}$, seen from the front, to scale [mm]', 'FontSize',15);
xlim(ax, [-780 560]);  ylim(ax, [10 600]);
% ---- (b) 확대 / zoom
ax = axes(fig, 'Position',[0.63 0.10 0.36 0.82]);  hold(ax,'on');  axis(ax,'equal');  box(ax,'on');
T = @(x,y,s,varargin) text(ax, x, y, s, 'Interpreter','latex', 'FontSize',14, varargin{:});
plot(ax, [-235 -10], [300 300], '-', 'Color',C.blu, 'LineWidth',4);
T(-232, 304, 'line $y = 300$', 'Color',C.navy, 'FontSize',12);
plot(ax, E.p(1), E.p(2), 's', 'MarkerSize',12, 'MarkerFaceColor',[0.2 0.2 0.2], 'MarkerEdgeColor','k');
T(E.p(1)-30, E.p(2)+8, '$\hat p = (-200.0,\ 345.0)$ (tag)', 'FontSize',13);
arw(ax, E.p', E.c0', C.pur, 2.0, 3.5);
T(-200, 291, sprintf('$R(\\hat\\psi)\\,d = (%.1f,\\ %.1f)$', E.R*E.d), 'Color',C.pur, 'FontSize',13);
arw(ax, E.p', E.pst', C.grn, 2.0, 3.5);
T(-158, 333, sprintf('$p^{\\star} - \\hat p = (%.1f,\\ %.1f)$', E.g), 'Color',C.grn, 'FontSize',13);
arw(ax, E.c0', E.pst', C.org, 2.4, 3.5);
plot(ax, E.c0(1), E.c0(2), 'o', 'MarkerSize',10, 'MarkerFaceColor','w', 'MarkerEdgeColor','k', 'LineWidth',1.3);
T(E.c0(1)+3, E.c0(2)-5, sprintf('$c_0 = (%.1f,\\ %.1f)$', E.c0), 'FontSize',13);
plot(ax, E.pst(1), E.pst(2), 'd', 'MarkerSize',11, 'MarkerFaceColor',C.grn, 'MarkerEdgeColor','k');
T(E.pst(1)+3, E.pst(2)+6, sprintf('$p^{\\star} = A + \\sigma u = (%.1f,\\ %.1f)$', E.pst), 'Color',C.grn, 'FontSize',13);
T(E.c0(1)+3, 294.5, sprintf('$R(\\hat\\psi)\\Delta s = -e_0 n$, $e_0 = %.2f$ mm', E.e0), 'Color',C.org, 'FontSize',13);
xb = E.c0 + 22*E.R(:,1);  yb = E.c0 + 22*E.R(:,2);
arw(ax, E.c0', xb', 'k', 1.0, 2.5);  T(xb(1)+1, xb(2)-1, '$x_B$', 'FontSize',12);
arw(ax, E.c0', yb', 'k', 1.0, 2.5);  T(yb(1)-9, yb(2)+3, '$y_B$', 'FontSize',12);
xlabel(ax, '$x$ [mm]', 'Interpreter','latex', 'FontSize',13);  ylabel(ax, '$y$ [mm]', 'Interpreter','latex', 'FontSize',13);
set(ax, 'TickLabelInterpreter','latex', 'FontSize',11);
xlim(ax, [-235 -10]);  ylim(ax, [272 358]);
title(ax, '(b) zoom of the dashed box', 'Interpreter','latex', 'FontSize',15);
exportgraphics(fig, f, 'Resolution',200);  close(fig);
end

% =====================================================================
function combined(E, f)
%  같은 화살표 삼각형: (a) 벽면에서 (스테이지·로봇 축 포함), (b) R' 로 돌린 바디에서
C = E.col;
fig = figure('Color','w', 'Position',[50 50 1700 800], 'Visible','off');
for pnl = 1:2
    if pnl == 1
        ax = axes(fig, 'Position',[0.02 0.08 0.46 0.84]);  M = eye(2);  o = E.p;
        ttl = '(a) on the wall: $\mathcal{W}$ components';
        lg = sprintf('$p^{\\star} - \\hat p = (%.1f,\\ %.1f)$', E.g);
        ld = sprintf('$R(\\hat\\psi)\\,d = (%.1f,\\ %.1f)$', E.R*E.d);
        le = sprintf('$R(\\hat\\psi)\\,\\Delta s = (%.2f,\\ %.2f)$', E.R*E.eb);
        Rb = E.R;
    else
        ax = axes(fig, 'Position',[0.52 0.08 0.46 0.84]);  M = E.R';  o = E.p;
        ttl = '(b) multiplied by $R^{\top}(\hat\psi)$: $\mathcal{B}$ components';
        lg = sprintf('$R^{\\top}(\\hat\\psi)\\,(p^{\\star} - \\hat p) = (%.1f,\\ %.1f)$', E.gB);
        ld = sprintf('$d = (%.1f,\\ %.1f)$', E.d);
        le = sprintf('$\\Delta s = (%.2f,\\ %.2f)$', E.eb);
        Rb = eye(2);
    end
    hold(ax,'on');  axis(ax,'equal');  axis(ax,'off');
    T = @(x,y,s,varargin) text(ax, x, y, s, 'Interpreter','latex', 'FontSize',14, varargin{:});
    W = @(v) M*(v - o);                % 그림 좌표 (태그 원점) / plot coords, tag at origin
    % 스테이지 사각형 / stage square
    sq = E.c0 + Rb*0 ;  %#ok<NASGU>
    cs = [-1 1 1 -1 -1; -1 -1 1 1 -1]*E.h;
    S = zeros(2,5);  for k = 1:5, S(:,k) = W(E.c0 + E.R*cs(:,k)); end
    yn = W(E.p - 75*E.R(:,2));  xn = W(E.p + 110*E.R(:,1));
    plot(ax, [0 yn(1)], [0 yn(2)], '--', 'Color',C.gry, 'LineWidth',0.8);
    plot(ax, [0 xn(1)], [0 xn(2)], '--', 'Color',C.gry, 'LineWidth',0.8);
    % 직선 / line
    l1 = W([-260; 300]);  l2 = W([30; 300]);
    plot(ax, [l1(1) l2(1)], [l1(2) l2(2)], '-', 'Color',C.blu, 'LineWidth',4);
    % 축 / axes
    if pnl == 1
        arw(ax, [-95 -95], [-55 -95], 'k', 1.2, 4);  T(-52, -95, '$x$', 'FontSize',13);
        arw(ax, [-95 -95], [-95 -55], 'k', 1.2, 4);  T(-97, -48, '$y$', 'FontSize',13);
    end
    xb = W(E.p + 70*E.R(:,1));  yb = W(E.p + 70*E.R(:,2));
    arw(ax, [0 0], xb', C.gry, 1.0, 4);  T(xb(1)+2, xb(2), '$x_B$', 'Color',C.gry, 'FontSize',13);
    arw(ax, [0 0], yb', C.gry, 1.0, 4);  T(yb(1)-12, yb(2)+5, '$y_B$', 'Color',C.gry, 'FontSize',13);
    % 삼각형 / triangle
    q0 = W(E.c0);  qs = W(E.pst);
    arw(ax, [0 0], q0', C.pur, 2.2, 4);
    arw(ax, [0 0], qs', C.grn, 2.2, 4);
    arw(ax, q0', qs', C.org, 2.8, 4);
    plot(ax, 0, 0, 's', 'MarkerSize',12, 'MarkerFaceColor',[0.2 0.2 0.2], 'MarkerEdgeColor','k');
    plot(ax, q0(1), q0(2), 'o', 'MarkerSize',9, 'MarkerFaceColor','w', 'MarkerEdgeColor','k', 'LineWidth',1.2);
    plot(ax, qs(1), qs(2), 'd', 'MarkerSize',10, 'MarkerFaceColor',C.grn, 'MarkerEdgeColor','k');
    T(-22, -10, '$\hat p$', 'FontSize',15);
    T(q0(1)+4, q0(2)-9, '$c_0$', 'FontSize',15);  T(qs(1)+4, qs(2)+7, '$p^{\star}$', 'Color',C.grn, 'FontSize',15);
    T(-95, 70, lg, 'Color',C.grn);  T(-95, 56, ld, 'Color',C.pur);  T(-95, 42, le, 'Color',C.org);
    if pnl == 1
        % 초록 화살표를 x_B, y_B 위로 내린 그림자 / projections of the green arrow on x_B and y_B
        px = W(E.p + E.gB(1)*E.R(:,1));  py = W(E.p + E.gB(2)*E.R(:,2));
        plot(ax, [qs(1) px(1)], [qs(2) px(2)], ':', 'Color',C.grn, 'LineWidth',1.2);
        plot(ax, [qs(1) py(1)], [qs(2) py(2)], ':', 'Color',C.grn, 'LineWidth',1.2);
        plot(ax, px(1), px(2), '.', 'Color',C.grn, 'MarkerSize',14);  plot(ax, py(1), py(2), '.', 'Color',C.grn, 'MarkerSize',14);
        T(px(1)-30, px(2)+9, sprintf('%.1f along $x_B$', E.gB(1)), 'Color',C.grn, 'FontSize',12);
        T(py(1)+3, py(2)-4, sprintf('%.1f along $y_B$', E.gB(2)), 'Color',C.grn, 'FontSize',12);
        T(-95, -118, 'shadows of the green arrow on the robot axes $=$ $R^{\top}(\hat\psi)\,(p^{\star} - \hat p)$', 'FontSize',12);
    else
        T(-95, -118, 'same arrows turned by $-\hat\psi = -3^{\circ}$: robot axes now horizontal and vertical', 'FontSize',12);
    end
    T(-95, 92, ttl, 'FontSize',15);
    xlim(ax, [-100 130]);  ylim(ax, [-125 100]);
end
annotation(fig, 'arrow', [0.475 0.525], [0.52 0.52], 'LineWidth',1.6);
annotation(fig, 'textbox', [0.455 0.53 0.09 0.05], 'String','$R^{\top}(\hat\psi)$', 'Interpreter','latex', ...
    'FontSize',18, 'EdgeColor','none', 'HorizontalAlignment','center');
exportgraphics(fig, f, 'Resolution',200);  close(fig);
end

% =====================================================================
function stage(E, f)
%  (a) Zaber 눈금 위의 s_cmd, (b) Y 축 세 눈금 (제어기 m, 제어기 mm, Zaber mm)
C = E.col;
fig = figure('Color','w', 'Position',[50 50 1600 720], 'Visible','off');
ax = axes(fig, 'Position',[0.05 0.11 0.40 0.80]);  hold(ax,'on');  axis(ax,'equal');  box(ax,'on');
T = @(x,y,s,varargin) text(ax, x, y, s, 'Interpreter','latex', 'FontSize',14, varargin{:});
fill(ax, [0 101.6 101.6 0], [0 0 101.6 101.6], [0.93 0.97 0.97], 'EdgeColor',C.stg, 'LineWidth',1.6);
plot(ax, 0, 0, 'ks', 'MarkerSize',10, 'MarkerFaceColor','k');
T(2, 5, 'Zaber $0$ (home)', 'FontSize',13);
plot(ax, [E.sc E.sc], [0 101.6], ':', 'Color',C.gry);  plot(ax, [0 101.6], [E.sc E.sc], ':', 'Color',C.gry);
plot(ax, E.sc, E.sc, '+', 'Color','k', 'MarkerSize',16, 'LineWidth',1.6);
T(E.sc-49, E.sc-5, '$s_c = (50.8,\ 50.8)$: controller $0$', 'FontSize',13);
arw(ax, [E.sc E.sc], E.s', C.org, 2.6, 3);
plot(ax, E.s(1), E.s(2), 'o', 'MarkerSize',11, 'MarkerFaceColor',C.red, 'MarkerEdgeColor','k');
T(E.s(1)+2, E.s(2)+5, sprintf('$s^{\\star} = (%.2f,\\ %.2f)$', E.s), 'Color',C.red, 'FontSize',14);
T(E.sc+3, E.sc+4, '$1000\,\Delta s$', 'Color',C.org, 'FontSize',14);
% 잘못된 경우 / wrong cases
w1 = E.eb;  plot(ax, w1(1), w1(2), 'x', 'Color',C.gry, 'MarkerSize',14, 'LineWidth',2);
T(w1(1)+3, w1(2)+3, sprintf('no $+s_c$: $(%.2f,\\ %.2f)$ near home', w1), 'Color',C.gry, 'FontSize',12);
w2 = E.sc + E.eb/1000;  plot(ax, w2(1)+0.8, w2(2)-2.5, 'x', 'Color',C.gry, 'MarkerSize',14, 'LineWidth',2);
T(w2(1)+3, w2(2)-11, 'no $\times 1000$: $(50.80,\ 50.81)$, pen hardly moves', 'Color',C.gry, 'FontSize',12);
xlabel(ax, '$X_s$ [mm] (along $x_B$, forward)', 'Interpreter','latex', 'FontSize',13);
ylabel(ax, '$Y_s$ [mm] (along $y_B$, robot left)', 'Interpreter','latex', 'FontSize',13);
set(ax, 'TickLabelInterpreter','latex', 'FontSize',11, 'XTick',[0 25.4 50.8 76.2 101.6], 'YTick',[0 25.4 50.8 76.2 101.6]);
xlim(ax, [-6 108]);  ylim(ax, [-6 108]);
title(ax, '(a) Zaber stage, body-fixed, 1 grid = 25.4 mm', 'Interpreter','latex', 'FontSize',15);
% ---- (b) 세 눈금 / three scales for the Y axis
ax = axes(fig, 'Position',[0.52 0.06 0.46 0.88]);  hold(ax,'on');  axis(ax,'off');
T = @(x,y,s,varargin) text(ax, x, y, s, 'Interpreter','latex', 'FontSize',14, varargin{:});
rows = {'controller, metre: $\Delta s_Y$', 'controller, mm: $1000\,\Delta s_Y$', 'Zaber, mm: $s^{\star}_Y = s_c + 1000\,\Delta s_Y$'};
vals = {[-0.0508 0 0.0107 0.0508], [-50.8 0 10.72 50.8], [0 50.8 61.52 101.6]};
lab  = {{'$-0.0508$','$0$','$0.0107$','$0.0508$'}, {'$-50.8$','$0$','$10.72$','$50.8$'}, {'$0$','$50.8$','$61.52$','$101.6$'}};
X = [0 0.5 0.605 1];  yy = [0.80 0.50 0.20];
for r = 1:3
    plot(ax, [0 1], [yy(r) yy(r)], 'k-', 'LineWidth',1.4);
    for k = 1:4
        cc = 'k';  if k == 3, cc = C.red; end
        plot(ax, [X(k) X(k)], yy(r) + [-0.02 0.02], '-', 'Color',cc, 'LineWidth',1.6);
        T(X(k)-0.03, yy(r)-0.06, lab{r}{k}, 'Color',cc, 'FontSize',13);
    end
    T(0, yy(r)+0.07, rows{r}, 'FontSize',14);
end
for r = 1:2
    arw(ax, [0.80 yy(r)-0.09], [0.80 yy(r+1)+0.10], C.org, 1.4, 0.02);
end
T(0.82, 0.66, '$\times 1000$ (m $\to$ mm)', 'Color',C.org);
T(0.82, 0.36, '$+\,s_c$ (move the zero)', 'Color',C.org);
T(0.42, 0.92, 'centre', 'FontSize',12);  T(0.60, 0.92, 'target', 'Color',C.red, 'FontSize',12);
T(0, 0.04, 'one point on the stage, three ways of writing it', 'FontSize',12);
title(ax, '(b) $Y$ axis of this example', 'Interpreter','latex', 'FontSize',15);
xlim(ax, [-0.05 1.25]);  ylim(ax, [0 1]);
exportgraphics(fig, f, 'Resolution',200);  close(fig);
end

% =====================================================================
function robot_body(ax, c, R, C)
%  로봇 외곽(420 x 343 mm)과 궤도 / robot outline and tracks [mm]
Lr = 420;  Wr = 343;
P = R*[Lr/2*[-1 1 1 -1]; Wr/2*[-1 -1 1 1]] + c;
patch(ax, P(1,:), P(2,:), [0.96 0.96 0.97], 'EdgeColor',C.gry, 'LineWidth',1.3);
for sgn = [-1 1]
    Q = R*[Lr/2*[-1 1 1 -1]; sgn*140 + 25*[-1 -1 1 1]] + c;
    patch(ax, Q(1,:), Q(2,:), [0.80 0.80 0.83], 'EdgeColor','none');
end
end

function arw(ax, p, q, c, w, hl)
%  선 + 채운 화살촉. hl = 화살촉 길이 (축 단위) / line with a filled head, hl = head length in axis units
plot(ax, [p(1) q(1)], [p(2) q(2)], '-', 'Color',c, 'LineWidth',w);
d = (q - p) / max(norm(q - p), 1e-9);  nrm = [-d(2) d(1)];  hw = 0.38*hl;
b = q - hl*d;
patch(ax, [q(1) b(1)+hw*nrm(1) b(1)-hw*nrm(1)], [q(2) b(2)+hw*nrm(2) b(2)-hw*nrm(2)], c, 'EdgeColor',c);
end
