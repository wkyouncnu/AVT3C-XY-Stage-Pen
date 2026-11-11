function make_fig_math()
%MAKE_FIG_MATH  README 4 장의 수식 그림 두 장을 LaTeX 글자로 그린다
%   Draws the two LaTeX-labelled figures of README chapter 4.
%     ../fig_geometry.png  벽면 좌표계, 로봇·태그·레버암·스테이지·펜·직선·횡오차 기하
%     (제어 구조 그림은 src/diagram_control.tex, TikZ)
%   모든 기호는 README 4 장의 식과 같다. 수치는 그림용 예시 (실제 값 아님).

here = fileparts(mfilename('fullpath'));
geometry(fullfile(here, '..', 'fig_geometry.png'));
% 제어 구조는 TikZ (src/diagram_control.tex) 로 옮김 / control structure moved to TikZ
frames(fullfile(here, '..', 'fig_frames.png'));
rotation(fullfile(here, '..', 'fig_rotation.png'));
pen_example(fullfile(here, '..', 'fig_pen_example.png'));
end

% =====================================================================
function frames(f)
%  좌표계 세 개: 벽면 W (heading 규약 포함), 바디 B, 스테이지 S. 바디 그림은 실제 치수 (m)
%  Three frames: wall W with the heading convention, body B and stage S (body panel to scale, m).
fig = figure('Color','w', 'Position',[50 50 1500 700], 'Visible','off');
navy = [0.12 0.23 0.37];  gry = [0.45 0.45 0.45];  red = [0.80 0.10 0.10];  grn = [0.20 0.55 0.25];  pur = [0.55 0.25 0.65];
%  (a) 벽면 좌표계 — 화면 좌표 = (-p2, p1)
ax = axes(fig, 'Position',[0.01 0.03 0.45 0.94]);  hold(ax,'on');  axis(ax,'equal');  axis(ax,'off');
T = @(x,y,s,varargin) text(ax, x, y, s, 'Interpreter','latex', 'FontSize',16, varargin{:});
arw(ax, [0 0], [0 0.95], 'k', 1.6);   T(0.03, 0.97, '$p_1$ (up, ceiling)');
arw(ax, [0 0], [-0.95 0], 'k', 1.6);  T(-1.18, 0.07, '$p_2$ (left)');
plot(ax, 0, 0, 'o', 'MarkerSize',14, 'MarkerEdgeColor','k', 'LineWidth',1.3);  plot(ax, 0, 0, 'k.', 'MarkerSize',13);
T(0.05, -0.07, '$z_W$ out of the wall (towards the viewer)', 'FontSize',13);
tt = linspace(0, 2*pi, 200);  plot(ax, 0.62*cos(tt), 0.62*sin(tt), ':', 'Color',gry);
H = [0 0.62; -0.62 0; 0 -0.62; 0.62 0];
L = {'$\psi = 0^{\circ}$', '$\psi = +90^{\circ}$', '$\psi = \pm 180^{\circ}$', '$\psi = -90^{\circ}$'};
O = [0.04 0.05; -0.38 -0.07; 0.04 -0.08; 0.02 0.07];
for k = 1:4
    arw(ax, [0 0], H(k,:), navy, 1.4);  T(H(k,1)+O(k,1), H(k,2)+O(k,2), L{k}, 'Color',navy, 'FontSize',14);
end
ta = linspace(pi/2+0.25, pi-0.25, 30);  plot(ax, 0.40*cos(ta), 0.40*sin(ta), '-', 'Color',red, 'LineWidth',1.6);
arw(ax, 0.40*[cos(ta(end-1)) sin(ta(end-1))], 0.40*[cos(ta(end)) sin(ta(end))], red, 1.6);
T(-0.40, 0.42, '$+\psi$', 'Color',red);
plot(ax, [-0.70 0.50], [-0.85 -0.85], '-', 'Color',[0.60 0.74 0.96], 'LineWidth',5);
arw(ax, [-0.10 -0.85], [0.20 -0.85], navy, 1.6);
T(-0.80, -0.95, 'line $A \to B$ points right: $u = [0,\,-1]^{\top}$, $\theta_l = -90^{\circ}$', 'FontSize',13);
arw(ax, [0.85 0.80], [0.85 0.45], gry, 1.8);  T(0.88, 0.62, '$g$', 'Color',gry);
T(-1.15, 1.12, '(a) wall frame $\mathcal{W}$ and heading $\psi$', 'FontSize',15);
xlim(ax, [-1.25 1.05]);  ylim(ax, [-1.05 1.18]);
%  (b) 바디·스테이지 좌표계 — 실제 치수 [m], x_B 오른쪽, y_B 위
ax = axes(fig, 'Position',[0.50 0.03 0.49 0.94]);  hold(ax,'on');  axis(ax,'equal');  axis(ax,'off');
T = @(x,y,s,varargin) text(ax, x, y, s, 'Interpreter','latex', 'FontSize',15, varargin{:});
Lr = 0.420;  Wr = 0.343;
patch(ax, Lr/2*[-1 1 1 -1], Wr/2*[-1 -1 1 1], [0.95 0.95 0.96], 'EdgeColor',gry, 'LineWidth',1.4);
for sgn = [-1 1]
    patch(ax, Lr/2*[-1 1 1 -1], sgn*0.140 + 0.025*[-1 -1 1 1], [0.75 0.75 0.78], 'EdgeColor','none');
end
arw(ax, [0 0], [0.27 0], 'k', 1.6);  T(0.275, -0.005, '$x_B$ (forward)');
arw(ax, [0 0], [0 0.23], 'k', 1.6);  T(-0.03, 0.25, '$y_B$ (left)');
T(-0.205, -0.205, 'robot centre $c$ = body origin, $z_B$ out of the wall', 'FontSize',12);
rt = [-0.080 0.060];  rp = [0 0];  h = 0.0508;
plot(ax, rp(1)+h*[-1 1 1 -1 -1], rp(2)+h*[-1 -1 1 1 -1], '--', 'Color',grn, 'LineWidth',1.5);
o = rp - h;  arw(ax, o, o + [0.13 0], grn, 1.3);  arw(ax, o, o + [0 0.13], grn, 1.3);
T(o(1)+0.135, o(2)-0.005, '$X_s$', 'Color',grn, 'FontSize',14);  T(o(1)+0.008, o(2)+0.135, '$Y_s$', 'Color',grn, 'FontSize',14);
T(o(1)-0.075, o(2)-0.022, '$s = (0,0)$', 'Color',grn, 'FontSize',12);
T(rp(1)+h+0.01, rp(2)+h+0.012, '$s = (101.6,\,101.6)$ mm', 'Color',grn, 'FontSize',12);
plot(ax, rt(1), rt(2), 's', 'MarkerSize',12, 'MarkerFaceColor',[0.2 0.2 0.2], 'MarkerEdgeColor','k');
T(-0.205, 0.128, 'tag $r_{tag} = (-0.08,\,0.06)$', 'FontSize',13);
plot(ax, rp(1), rp(2), 'o', 'MarkerSize',10, 'MarkerFaceColor',red, 'MarkerEdgeColor','k');
T(0.062, 0.018, 'pen $r_{pen,0} = (0,0)$ at $s = s_c$', 'Color',red, 'FontSize',12);
arw(ax, rt, rp - 0.012*(rp-rt)/norm(rp-rt), pur, 1.6);
T(-0.205, -0.112, '$d = r_{pen,0} - r_{tag} = (0.08,\,-0.06)$ m', 'Color',pur, 'FontSize',13);
T(-0.24, 0.29, '(b) body frame $\mathcal{B}$ and stage frame $\mathcal{S}$ (to scale)', 'FontSize',15);
xlim(ax, [-0.26 0.40]);  ylim(ax, [-0.23 0.31]);
exportgraphics(fig, f, 'Resolution',200);  close(fig);
end

% =====================================================================
function rotation(f)
%  회전행렬의 열 = 바디 단위벡터를 벽면 좌표로 쓴 것. psi = 30 deg 예시, 화면 좌표 = (-p2, p1)
%  Columns of R(psi) are the body unit vectors written in wall coordinates (psi = 30 deg example).
fig = figure('Color','w', 'Position',[50 50 1450 720], 'Visible','off');
ax = axes(fig, 'Position',[0.01 0.02 0.98 0.96]);  hold(ax,'on');  axis(ax,'equal');  axis(ax,'off');
T = @(x,y,s,varargin) text(ax, x, y, s, 'Interpreter','latex', 'FontSize',17, varargin{:});
navy = [0.12 0.23 0.37];  red = [0.80 0.10 0.10];  blu = [0.10 0.35 0.75];  gry = [0.5 0.5 0.5];
S = @(w) [-w(2), w(1)];
ps = deg2rad(30);  xw = [cos(ps) sin(ps)];  yw = [-sin(ps) cos(ps)];
arw(ax, [0 0], S([1.15 0]), 'k', 1.4);  T(0.03, 1.18, '$p_1$');
arw(ax, [0 0], S([0 1.15]), 'k', 1.4);  T(-1.25, 0.06, '$p_2$');
arw(ax, [0 0], S([0 -0.6]), gry, 0.8);  arw(ax, [0 0], S([-0.6 0]), gry, 0.8);
arw(ax, [0 0], S(xw), red, 2.2);  arw(ax, [0 0], S(yw), blu, 2.2);
T(S(xw)*[1;0]-0.10, S(xw)*[0;1]+0.07, '$x_B$', 'Color',red);
T(S(yw)*[1;0]-0.13, S(yw)*[0;1]-0.02, '$y_B$', 'Color',blu);
X = S(xw);  plot(ax, [X(1) 0], [X(2) X(2)], ':', 'Color',red);  plot(ax, [X(1) X(1)], [X(2) 0], ':', 'Color',red);
T(0.03, X(2), '$\cos\psi$', 'Color',red, 'FontSize',15);  T(X(1)-0.07, -0.07, '$\sin\psi$', 'Color',red, 'FontSize',15);
Y = S(yw);  plot(ax, [Y(1) 0], [Y(2) Y(2)], ':', 'Color',blu);  plot(ax, [Y(1) Y(1)], [Y(2) 0], ':', 'Color',blu);
T(0.03, Y(2), '$-\sin\psi$', 'Color',blu, 'FontSize',15);  T(Y(1)-0.08, 0.07, '$\cos\psi$', 'Color',blu, 'FontSize',15);
ta = linspace(pi/2, pi/2 + ps, 30);  plot(ax, 0.35*cos(ta), 0.35*sin(ta), 'k-', 'LineWidth',1.2);
T(0.03, 0.30, '$\psi = 30^{\circ}$', 'FontSize',15);
T(-1.25, -0.95, 'wall frame seen from the front: $p_1$ up, $p_2$ left; $\psi$ counter-clockwise from $p_1$', 'FontSize',13);
x0 = 1.55;
T(x0, 1.05, '$x_B$ in $\mathcal{W}$: $\;\left[\begin{array}{c}\cos\psi\\ \sin\psi\end{array}\right]$, $\quad y_B$ in $\mathcal{W}$: $\;\left[\begin{array}{c}-\sin\psi\\ \cos\psi\end{array}\right]$');
T(x0, 0.70, '$R(\psi) = \big[\; x_B \;\; y_B \;\big] = \left[\begin{array}{cc}\cos\psi & -\sin\psi\\ \sin\psi & \cos\psi\end{array}\right]$');
T(x0, 0.38, 'body $\to$ wall: $\;v^{\mathcal{W}} = R(\psi)\, v^{\mathcal{B}} = a\, x_B + b\, y_B, \;\; v^{\mathcal{B}} = [a,\, b]^{\top}$', 'FontSize',15);
T(x0, 0.16, 'wall $\to$ body: $\;v^{\mathcal{B}} = R^{\top}(\psi)\, v^{\mathcal{W}} \quad (R^{\top}R = I)$', 'FontSize',15);
T(x0, -0.12, 'check, $\psi = -90^{\circ}$ (driving right along $A \to B$):', 'FontSize',15);
T(x0, -0.40, '$R = \left[\begin{array}{cc}0 & 1\\ -1 & 0\end{array}\right]$: $\;x_B \mapsto [0,\,-1]^{\top}$ (right), $\;y_B \mapsto [1,\,0]^{\top}$ (up)', 'FontSize',15);
T(x0, -0.68, 'so a stage move $+\Delta s_Y$ (body left) moves the pen up the wall', 'FontSize',15);
xlim(ax, [-1.35 4.10]);  ylim(ax, [-1.05 1.30]);
exportgraphics(fig, f, 'Resolution',200);  close(fig);
end

% =====================================================================
function pen_example(f)
%  펜 보상 수치 예제 (README 4.6). A, B, d 는 XY_setup 값, 자세는 직선 주행 중 대표값
%  Worked pen-compensation example with the XY_setup values of A, B and d.
A = [0.30; 0.70];  B = [0.30; -0.50];  d = [0.08; -0.06];  rtag = [-0.08; 0.06];  sc = 50.8;
p = [0.345; 0.200];  ps = deg2rad(-87);
R = [cos(ps) -sin(ps); sin(ps) cos(ps)];
u = (B - A)/norm(B - A);  n = [-u(2); u(1)];
c0 = p + R*d;  sg = u'*(c0 - A);  e0 = n'*(c0 - A);  pst = A + sg*u;
ds = R'*(pst - p) - d;  s = sc + 1000*ds;  c = p - R*rtag;
S = @(w) [-w(2), w(1)];
fig = figure('Color','w', 'Position',[50 50 1600 760], 'Visible','off');
navy = [0.12 0.23 0.37];  gry = [0.45 0.45 0.45];  red = [0.80 0.10 0.10];  grn = [0.20 0.55 0.25];
pur = [0.55 0.25 0.65];  org = [0.85 0.45 0.05];
%  (a) 벽면 (실제 치수)
ax = axes(fig, 'Position',[0.01 0.03 0.50 0.92]);  hold(ax,'on');  axis(ax,'equal');  axis(ax,'off');
T = @(x,y,s,varargin) text(ax, x, y, s, 'Interpreter','latex', 'FontSize',14, varargin{:});
ex = R*[1;0];  ey = R*[0;1];  Lr = 0.42;  Wr = 0.343;
K = [c + Lr/2*ex + Wr/2*ey, c + Lr/2*ex - Wr/2*ey, c - Lr/2*ex - Wr/2*ey, c - Lr/2*ex + Wr/2*ey];
Ks = [S(K(:,1)); S(K(:,2)); S(K(:,3)); S(K(:,4))];
patch(ax, Ks(:,1), Ks(:,2), [0.96 0.96 0.97], 'EdgeColor',gry, 'LineWidth',1.2);
plot(ax, [-0.36 0.02], [0.30 0.30], '-', 'Color',[0.60 0.74 0.96], 'LineWidth',5);
T(-0.355, 0.292, 'line $p_1 = 0.30$ m ($A \to B$ runs to the right)', 'Color',navy, 'FontSize',13);
q = S(p);  q0 = S(c0);  qs = S(pst);
plot(ax, q(1), q(2), 's', 'MarkerSize',12, 'MarkerFaceColor',[0.2 0.2 0.2], 'MarkerEdgeColor','k');
T(q(1)-0.075, q(2)+0.010, sprintf('tag $\\hat p = (%.3f,\\ %.3f)$', p));
arw2(ax, q, q0, pur, 1.8);
T(-0.335, 0.330, sprintf('$R(\\hat\\psi)\\,d = (%.4f,\\ %.4f)$', R*d), 'Color',pur, 'FontSize',13);
plot(ax, [-0.215 -0.165], [0.327 0.322], ':', 'Color',pur);
plot(ax, q0(1), q0(2), 'o', 'MarkerSize',10, 'MarkerFaceColor','w', 'MarkerEdgeColor','k', 'LineWidth',1.3);
T(q0(1)+0.006, q0(2)-0.010, sprintf('$c_0 = (%.4f,\\ %.4f)$ (pen, stage centred)', c0), 'FontSize',13);
plot(ax, qs(1), qs(2), 'd', 'MarkerSize',11, 'MarkerFaceColor',red, 'MarkerEdgeColor','k');
T(qs(1)+0.006, qs(2)+0.012, sprintf('$p^{\\star} = P = (%.4f,\\ %.4f)$', pst), 'Color',red);
arw2(ax, q0, qs, org, 1.8);
T(q0(1)-0.03, 0.272, sprintf('$R(\\hat\\psi)\\Delta s$: pen moves up $|e_0| = %.1f$ mm', abs(1000*e0)), 'Color',org, 'FontSize',13);
xe = S(c0 + 0.05*ex);  arw2(ax, q0, xe, 'k', 1.2);  T(xe(1)+0.003, xe(2)-0.002, '$x_B$', 'FontSize',13);
T(-0.355, 0.382, sprintf('(a) wall view, to scale: $\\hat\\psi = %.0f^{\\circ}$ (nose $3^{\\circ}$ above the line)', rad2deg(ps)), 'FontSize',15);
xlim(ax, [-0.36 0.03]);  ylim(ax, [0.262 0.39]);
%  (b) 스테이지 (바디 좌표, mm)
ax = axes(fig, 'Position',[0.55 0.09 0.21 0.82]);  hold(ax,'on');  axis(ax,'equal');  box(ax,'on');
plot(ax, [0 101.6 101.6 0 0], [0 0 101.6 101.6 0], '--', 'Color',grn, 'LineWidth',1.4);
plot(ax, sc, sc, '+', 'Color','k', 'MarkerSize',14, 'LineWidth',1.4);
arw(ax, [sc sc], s', org, 1.8);
plot(ax, s(1), s(2), 'o', 'MarkerSize',10, 'MarkerFaceColor',red, 'MarkerEdgeColor','k');
text(ax, sc+2, sc-5, '$s_c$', 'Interpreter','latex', 'FontSize',14);
text(ax, s(1)+3, s(2)+4, sprintf('$s^{\\star} = (%.2f,\\ %.2f)$', s), 'Interpreter','latex', 'FontSize',14, 'Color',red);
xlabel(ax, '$X_s$ [mm] (along $x_B$)', 'Interpreter','latex', 'FontSize',14);
ylabel(ax, '$Y_s$ [mm] (along $y_B$)', 'Interpreter','latex', 'FontSize',14);
title(ax, '(b) stage command, body frame', 'Interpreter','latex', 'FontSize',15);
xlim(ax, [-5 107]);  ylim(ax, [-5 107]);  set(ax, 'TickLabelInterpreter','latex', 'FontSize',12);
%  (c) 계산 순서
ax = axes(fig, 'Position',[0.785 0.03 0.215 0.94]);  axis(ax,'off');  hold(ax,'on');
L = {sprintf('1) $\\sigma = u^{\\top}(c_0 - A) = %.4f$ m', sg)
     sprintf('2) $e_0 = n^{\\top}(c_0 - A) = %.4f$ m', e0)
     sprintf('3) $p^{\\star} = A + \\sigma u = (%.4f,\\ %.4f)$', pst)
     sprintf('4) $p^{\\star} - \\hat p = (%.4f,\\ %.4f)$', pst - p)
     sprintf('5) $R^{\\top}(p^{\\star} - \\hat p) = (%.4f,\\ %.4f)$', R'*(pst - p))
     sprintf('6) $\\Delta s = (5) - d = (%.4f,\\ %.4f)$ m', ds)
     '7) $s^{\star} = s_c + 1000\,\Delta s$'
     sprintf('$\\quad\\ \\ = (%.2f,\\ %.2f)$ mm', s)
     sprintf('check: $n^{\\top}(P - A) = %.0f$', abs(n'*(p + R*(d + ds) - A)))};
for k = 1:numel(L), text(ax, 0, 1 - 0.105*k, L{k}, 'Interpreter','latex', 'FontSize',12.5, 'Units','normalized'); end
text(ax, 0, 0.98, '(c) steps of eq. (14)', 'Interpreter','latex', 'FontSize',15, 'Units','normalized');
exportgraphics(fig, f, 'Resolution',200);  close(fig);
end

function arw2(ax, p, q, c, w)
%  작은 화살촉 (확대 그림용) / smaller head for the zoomed figure
plot(ax, [p(1) q(1)], [p(2) q(2)], '-', 'Color',c, 'LineWidth',w);
d = (q - p) / max(norm(q - p), 1e-9);  nrm = [-d(2) d(1)];  hl = 0.006;  hw = 0.0025;
b = q - hl*d;
patch(ax, [q(1) b(1)+hw*nrm(1) b(1)-hw*nrm(1)], [q(2) b(2)+hw*nrm(2) b(2)-hw*nrm(2)], c, 'EdgeColor',c);
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
%  p1 = 위, p2 = 왼쪽 (벽을 정면에서 볼 때), z_W = 벽 밖 (보는 사람 쪽) -> 화면 좌표 (-p2, p1)
o = [0.20 -0.16];
arw(ax, o, o + [0 0.17], 'k', 1.4);  arw(ax, o, o + [-0.17 0], 'k', 1.4);
T(o(1)-0.005, o(2)+0.20, '$p_1$ (up)', 'FontSize',15);
T(o(1)-0.23, o(2)+0.035, '$p_2$ (left)', 'FontSize',15);
plot(ax, o(1), o(2), 'o', 'MarkerSize',13, 'MarkerEdgeColor','k', 'LineWidth',1.2);
plot(ax, o(1), o(2), 'k.', 'MarkerSize',12);
T(o(1)+0.02, o(2)-0.035, '$z_W$ (out of wall)', 'FontSize',13);
T(o(1)-0.20, o(2)-0.085, '$\mathcal{W}$: wall frame, seen from the front', 'FontSize',13);
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
T(c(1)-0.26, c(2)+0.34, '$\psi \approx -96^{\circ}$ in this sketch (clockwise from $p_1$ is negative)', 'FontSize',13);

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
T(sq(1,1)+0.035, c0(2)-0.05, {'stage travel', '$s \in [0,\,101.6]$ mm'}, 'FontSize',12, 'Color',grn);
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
xlim(ax, [-0.03 1.95]);  ylim(ax, [-0.28 0.98]);
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

