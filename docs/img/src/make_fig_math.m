function make_fig_math()
%MAKE_FIG_MATH  README 4 장의 수식 그림 두 장을 LaTeX 글자로 그린다
%   Draws the two LaTeX-labelled figures of README chapter 4.
%     ../fig_geometry.png  벽면 좌표계, 로봇·태그·레버암·스테이지·펜·직선·횡오차 기하
%     ../fig_control.png   제어 구조 (외부 루프 -> PWM -> 내부 PI -> 동역학 / 펜 보상 -> 스테이지 서보)
%   모든 기호는 README 4 장의 식과 같다. 수치는 그림용 예시 (실제 값 아님).

here = fileparts(mfilename('fullpath'));
geometry(fullfile(here, '..', 'fig_geometry.png'));
control(fullfile(here, '..', 'fig_control.png'));
end

% =====================================================================
function geometry(f)
%  기하 그림. 레버암·스테이지 크기는 읽기 쉽게 키운 예시 (축척 아님)
%  Geometry figure; lever arm and stage travel are enlarged for legibility (not to scale).
fig = figure('Color','w', 'Position',[50 50 1400 820], 'Visible','off');
ax = axes(fig, 'Position',[0.01 0.01 0.98 0.98]);  hold(ax,'on');  axis(ax,'equal');  axis(ax,'off');
navy = [0.12 0.23 0.37];  gry = [0.45 0.45 0.45];  red = [0.80 0.10 0.10];  grn = [0.20 0.55 0.25];
pur = [0.55 0.25 0.65];  org = [0.85 0.45 0.05];
T = @(x,y,s,varargin) text(ax, x, y, s, 'Interpreter','latex', 'FontSize',17, varargin{:});

%  벽면 프레임과 중력 / wall frame and gravity
o = [0.00 -0.16];
arw(ax, o, o + [0 0.17], 'k', 1.4);  arw(ax, o, o + [0.17 0], 'k', 1.4);
T(o(1)-0.005, o(2)+0.20, '$p_1$ (up)', 'FontSize',15);
T(o(1)+0.18, o(2)+0.005, '$-p_2$', 'FontSize',15);
T(o(1)+0.03, o(2)-0.04, '$\mathcal{W}$: wall frame ($p_2$ = left)', 'FontSize',13);
arw(ax, [0.06 0.93], [0.06 0.80], gry, 1.6);  T(0.08, 0.87, '$g$', 'Color',gry);

%  직선 A -> B
A = [0.10 0.30];  B = [1.40 0.30];
plot(ax, [A(1) B(1)], [A(2) B(2)], '-', 'Color',[0.60 0.74 0.96], 'LineWidth',5);
plot(ax, A(1), A(2), 'o', 'MarkerSize',11, 'MarkerFaceColor','w', 'MarkerEdgeColor',grn, 'LineWidth',2);
plot(ax, B(1), B(2), 's', 'MarkerSize',11, 'MarkerFaceColor','w', 'MarkerEdgeColor',red, 'LineWidth',2);
T(A(1)-0.015, A(2)+0.04, '$A$');  T(B(1)-0.015, B(2)+0.045, '$B$');
arw(ax, [0.22 0.30], [0.36 0.30], navy, 2);  T(0.30, 0.335, '$u$', 'Color',navy);
arw(ax, [0.22 0.30], [0.22 0.44], navy, 2);  T(0.18, 0.42, '$n$', 'Color',navy);

%  로봇 / robot (screen heading angle a, body axes ex, ey)
a = deg2rad(-6);  ex = [cos(a) sin(a)];  ey = [-sin(a) cos(a)];
c = [0.78 0.52];  Lr = 0.56;  Wr = 0.46;
cr = [c + Lr/2*ex + Wr/2*ey; c + Lr/2*ex - Wr/2*ey; c - Lr/2*ex - Wr/2*ey; c - Lr/2*ex + Wr/2*ey];
patch(ax, cr(:,1), cr(:,2), [0.94 0.94 0.95], 'EdgeColor',gry, 'LineWidth',1.5);
T(c(1)-0.26, c(2)+0.27, 'AVT-3C (magnetic tracks)', 'FontSize',13, 'Color',gry);
xb = c + 0.20*ex;  yb = c + 0.17*ey;
arw(ax, c, xb, 'k', 1.6);  arw(ax, c, yb, 'k', 1.6);
T(xb(1)+0.01, xb(2)-0.005, '$x_B$');  T(yb(1)-0.055, yb(2)+0.01, '$y_B$');
plot(ax, [c(1) c(1)], [c(2) c(2)+0.20], 'k:', 'LineWidth',1);
tt = linspace(pi/2, a, 40);  plot(ax, c(1)+0.12*cos(tt), c(2)+0.12*sin(tt), 'k-', 'LineWidth',1);
T(c(1)+0.09, c(2)+0.10, '$\psi$');

%  태그 p, 레버암 R(psi)d, 스테이지 중앙의 펜 c0
pT = c - 0.19*ex + 0.12*ey;
c0 = c + 0.06*ex - 0.04*ey;
plot(ax, pT(1), pT(2), 's', 'MarkerSize',12, 'MarkerFaceColor',[0.2 0.2 0.2], 'MarkerEdgeColor','k');
T(pT(1)-0.07, pT(2)+0.04, '$p$ (tag)');
arw(ax, pT, c0, pur, 1.8);
m = (pT + c0)/2;  T(m(1)-0.11, m(2)-0.02, '$R(\psi)\,d$', 'Color',pur, 'FontSize',16);

%  스테이지 행정과 캐리지 이동 / stage travel and carriage offset
h = 0.095;
sq = [c0 + h*ex + h*ey; c0 + h*ex - h*ey; c0 - h*ex - h*ey; c0 - h*ex + h*ey; c0 + h*ex + h*ey];
plot(ax, sq(:,1), sq(:,2), '--', 'Color',grn, 'LineWidth',1.4);
T(sq(1,1)+0.012, c0(2)-0.05, {'stage travel', '$s \in [0,\,101.6]$ mm'}, 'FontSize',12, 'Color',grn);
P = c0 + 0.012*ex - 0.075*ey;
plot(ax, c0(1), c0(2), 'o', 'MarkerSize',9, 'MarkerFaceColor','w', 'MarkerEdgeColor','k', 'LineWidth',1.2);
T(c0(1)+0.015, c0(2)+0.025, '$c_0$', 'FontSize',16);
arw(ax, c0, P, org, 1.8);
T(P(1)-0.165, (c0(2)+P(2))/2-0.005, '$R(\psi)\,\Delta s$', 'Color',org, 'FontSize',16);
plot(ax, P(1), P(2), 'o', 'MarkerSize',11, 'MarkerFaceColor',red, 'MarkerEdgeColor','k');
T(P(1)+0.04, P(2)-0.045, '$P$ (pen)', 'Color',red);

%  수선의 발 p*, 진행 거리 sigma, 횡오차 e
ps = [c0(1) A(2)];
plot(ax, [c0(1) ps(1)], [c0(2) ps(2)], ':', 'Color',navy, 'LineWidth',1.2);
plot(ax, ps(1), ps(2), 'd', 'MarkerSize',10, 'MarkerFaceColor',navy, 'MarkerEdgeColor',navy);
T(ps(1)+0.012, ps(2)-0.035, '$p^{\star}$', 'Color',navy);
yd = A(2) - 0.085;
plot(ax, [A(1) ps(1)], [yd yd], '-', 'Color',gry);
plot(ax, [A(1) A(1)], yd + [-0.012 0.012], '-', 'Color',gry);  plot(ax, [ps(1) ps(1)], yd + [-0.012 0.012], '-', 'Color',gry);
T((A(1)+ps(1))/2-0.10, yd-0.035, '$\sigma = u^{\top}(c_0 - A)$', 'Color',gry, 'FontSize',16);
xe = c(1) + 0.36;
plot(ax, [P(1) xe+0.01], [P(2) P(2)], ':', 'Color',red);
arw(ax, [xe A(2)], [xe P(2)], red, 1.2);
T(xe+0.015, (A(2)+P(2))/2+0.005, '$e = n^{\top}(P - A)$', 'Color',red, 'FontSize',16);
T(0.12, 0.11, '$u = (B-A)/\|B-A\|,\quad n = [-u_2,\; u_1]^{\top}$', 'FontSize',15, 'Color',navy);

%  기호 정리 / symbol list
lx = 1.25;  ly = 0.93;  dy = 0.048;
L = {'$p$: tag position (ZeroKey)'
     '$\psi$: heading (Xsens IMU)'
     '$d = r_{pen,0} - r_{tag}$: lever arm, body frame'
     '$c_0 = p + R(\psi)\,d$: pen at stage centre'
     '$\Delta s = (s - s_c)/1000$: carriage offset [m]'
     '$P = c_0 + R(\psi)\,\Delta s$: pen tip'
     '$R(\psi) = \left[\begin{array}{cc} \cos\psi & -\sin\psi \\ \sin\psi & \cos\psi \end{array}\right]$'};
for k = 1:numel(L), T(lx, ly - (k-1)*dy - (k==7)*0.025, L{k}, 'FontSize',14); end
xlim(ax, [-0.03 1.95]);  ylim(ax, [-0.24 0.98]);
exportgraphics(fig, f, 'Resolution',220);  close(fig);
end

function arw(ax, p, q, c, w)
%  선 + 채운 삼각형 화살촉 (길이와 무관한 일정한 크기) / line with a fixed-size filled head
plot(ax, [p(1) q(1)], [p(2) q(2)], '-', 'Color',c, 'LineWidth',w);
d = (q - p) / max(norm(q - p), 1e-9);  nrm = [-d(2) d(1)];  hl = 0.028;  hw = 0.011;
b = q - hl*d;
patch(ax, [q(1) b(1)+hw*nrm(1) b(1)-hw*nrm(1)], [q(2) b(2)+hw*nrm(2) b(2)-hw*nrm(2)], c, 'EdgeColor',c);
end

% =====================================================================
function control(f)
fig = figure('Color','w', 'Position',[50 50 1500 620], 'Visible','off');
ax = axes(fig, 'Position',[0 0 1 1]);  hold(ax,'on');  axis(ax,'off');  xlim(ax,[0 15]);  ylim(ax,[0 6.2]);
navy = [0.12 0.23 0.37];  bg = [0.91 0.93 0.96];  pl = [0.95 0.95 0.95];
T = @(x,y,s,varargin) text(ax, x, y, s, 'Interpreter','latex', 'FontSize',15, 'HorizontalAlignment','center', varargin{:});
blk = @(x,y,w,h,s,c) [rectangle(ax,'Position',[x-w/2 y-h/2 w h],'FaceColor',c,'EdgeColor',navy,'LineWidth',1.2), T(x,y,s)];

y1 = 4.3;  y2 = 1.6;
blk(1.3, y1, 1.9, 1.0, {'Mission','$\mathrm{mode},\ x_r(t)$'}, pl);
blk(3.9, y1, 2.3, 1.2, {'Outer loop','$e = R^{\top}(\psi)(x_r - p)$','$v_c,\ \omega_c$  (eq. 11)'}, bg);
blk(6.6, y1, 1.9, 1.0, {'PWM map','$\mathtt{calc\_pwm}$'}, bg);
blk(9.0, y1, 2.3, 1.2, {'ESC + inner PI','$K_{P,v},K_{I,v}$','$K_{P,\omega},K_{I,\omega}$'}, pl);
blk(11.6, y1, 2.0, 1.2, {'Dynamics','$\tilde M\dot V + \tilde C V$','$+\tilde f = \tilde B\tau + \tilde f_g$'}, pl);
blk(13.9, y1, 1.6, 1.0, {'Kinematics','$\dot q = S(q)V$'}, pl);
blk(3.9, y2, 2.3, 1.2, {'Pen compensation','$s^{*} = s_c + 1000\,\Delta s$','(eq. 14)'}, bg);
blk(7.8, y2, 2.3, 1.2, {'Zaber stage','trajectory + servo','$\omega_{n,s},\ \zeta_s$'}, pl);
blk(11.6, y2, 2.0, 1.0, {'Pen tip','$P = c_0 + R(\psi)\Delta s$'}, pl);
blk(1.3, y2, 1.9, 1.0, {'Nav','$\hat p,\ \hat\psi$'}, bg);

A = @(p,q) quiver(ax, p(1), p(2), q(1)-p(1), q(2)-p(2), 0, 'Color','k', 'LineWidth',1.2, 'MaxHeadSize',0.25/max(norm(q-p),0.5));
A([2.25 y1], [2.75 y1]);  A([5.05 y1], [5.65 y1]);  T(5.35, y1+0.3, '$v_c,\omega_c$', 'FontSize',13);
A([7.55 y1], [7.85 y1]);  T(7.7, y1+0.72, '$u_{pwm}$', 'FontSize',13);
A([10.15 y1], [10.6 y1]); T(10.38, y1+0.72, '$F_l,F_r$', 'FontSize',13);
A([12.6 y1], [13.1 y1]);  T(12.85, y1+0.72, '$V$', 'FontSize',13);
A([5.05 y2], [6.65 y2]);  T(5.85, y2+0.3, '$s^{*}$ /stage/target\_pos', 'FontSize',12);
A([8.95 y2], [10.6 y2]);  T(9.8, y2+0.3, '$s$', 'FontSize',13);
%  측정 되먹임
plot(ax, [14.7 14.7 14.95 14.95], [y1 y1 y1 0.35], 'k-', 'LineWidth',1.2);  T(14.6, 3.2, '$q$', 'FontSize',14);
plot(ax, [14.95 0.25 0.25], [0.35 0.35 y2], 'k-', 'LineWidth',1.2);  A([0.25 y2], [0.35 y2]);
T(7.5, 0.6, 'ZeroKey $/\mathrm{zerokey/position}$,  Xsens $/\mathrm{imu/data}$  (noise $\sigma_p$, $\sigma_\psi$, drift $\dot b_\psi$)', 'FontSize',13);
plot(ax, [1.3 1.3], [y2+0.5 y1-0.5], 'k-', 'LineWidth',1.2);  A([1.3 y1-0.9], [1.3 y1-0.5]);
plot(ax, [2.25 2.5 2.5 2.6], [y2 y2 y1-0.35 y1-0.35], 'k-', 'LineWidth',1.2);  A([2.5 y1-0.35], [2.75 y1-0.35]);
A([2.25 y2], [2.75 y2]);  T(2.0, 2.65, '$\hat p,\hat\psi$', 'FontSize',13);
%  스테이지 측정 -> 중앙 유지 (선택)
plot(ax, [7.8 7.8], [y2+0.6 3.0], 'k--', 'LineWidth',1);  plot(ax, [7.8 4.6], [3.0 3.0], 'k--', 'LineWidth',1);
A([4.6 3.0], [4.6 y1-0.6]);  T(6.3, 3.2, '/stage/pos $\to k_c, k_{ci}$ (optional)', 'FontSize',12);
T(3.9, 5.3, 'macro: robot (ROS 2 /motor\_pwm)', 'FontSize',14, 'Color',navy);
T(10.2, 2.45, 'micro: XY stage', 'FontSize',14, 'Color',navy);
exportgraphics(fig, f, 'Resolution',200);  close(fig);
end

