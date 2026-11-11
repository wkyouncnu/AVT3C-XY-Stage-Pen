function vfile = XY_animate_video(out, vfile)
%XY_ANIMATE_VIDEO  로봇(STL) + XY 스테이지 + 펜이 벽에 직선을 긋는 3D 애니메이션을 MP4 로 저장
%
%   XY_animate_video(out)                       -> video/XY_1_pen_sim.mp4
%   XY_animate_video(out, 'C:\...\my.mp4')
%
%   화면 구성
%     왼쪽 위   전체 벽면 (고정 카메라)
%     오른쪽 위 펜 주변 확대 (펜을 따라가는 카메라) — 스테이지가 움직이는 것이 보인다
%     아래      펜 횡오차 (보상 / 보상 없음)
%
%   좌표: 벽을 정면에서 본다. X = -p2 (오른쪽 +), Y = p1 (위 +), Z = 벽에서 나오는 방향
%   로봇 본체는 4_CAD/AVT-3C.stl. 스테이지·펜 홀더는 데이터시트 치수로 그린 상자다
%   (STEP 의 펜 홀더 배치는 3안과 달라 쓰지 않았다).

here = fileparts(mfilename('fullpath'));
if nargin < 2 || isempty(vfile)
    if ~isfolder(fullfile(here,'video')), mkdir(fullfile(here,'video')); end
    vfile = fullfile(here, 'video', 'XY_1_pen_sim.mp4');
end
P = struct();
for nm = {'r_tag','r_pen0','stage_c','anim_speed','anim_fps','stl_file','pen_d','line_pre','line_post'}
    P.(nm{1}) = evalin('base', nm{1});
end

lg = out.log;
[t, q]  = xy_getd(lg, 'q_true');
[~, sp] = xy_getd(lg, 'stage_pos');
[~, pt] = xy_getd(lg, 'pen_true');
[~, on] = xy_getd(lg, 'pen_on');
[~, md] = xy_getd(lg, 'mode');
[~, ep] = xy_getd(lg, 'e_pen');
[~, en] = xy_getd(lg, 'e_nocomp');
[~, ab] = xy_getd(lg, 'lineAB');
A = ab(end,1:2);  B = ab(end,3:4);
on = on > 0.5;

W = @(p1, p2) [-p2, p1];                    % 벽면 -> 화면 (X, Y)

%% 창과 축
fig = figure('Color','w', 'Position',[40 40 1280 720], 'Visible','on', 'Name','XY pen animation');
%  원근 카메라는 축 영역 밖까지 그리므로, 3D 축을 각자 패널에 넣어 잘라낸다
pn1 = uipanel(fig, 'Position',[0.01 0.29 0.55 0.66], 'BackgroundColor','w', 'BorderType','line', ...
              'Title','전체 벽면', 'FontSize',11);
pn2 = uipanel(fig, 'Position',[0.57 0.29 0.42 0.66], 'BackgroundColor','w', 'BorderType','line', ...
              'Title','펜 확대 — 초록 = XY 스테이지 (움직임), 주황 = 펜 홀더', 'FontSize',11);
ax1 = axes(pn1, 'Position',[0 0 1 1]);
ax2 = axes(pn2, 'Position',[0 0 1 1]);
ax3 = axes(fig, 'Position',[0.07 0.06 0.88 0.18]);
for ax = [ax1 ax2]
    hold(ax,'on'); axis(ax,'equal'); axis(ax,'off');
    set(ax, 'Projection','perspective', 'CameraUpVector',[0 1 0]);
end

%% 벽 · 목표 직선
xy = [W(q(:,1), q(:,2)); W(A(1),A(2)); W(B(1),B(2))];
xl = [min(xy(:,1))-0.30, max(xy(:,1))+0.30];
yl = [min(xy(:,2))-0.30, max(xy(:,2))+0.30];
for ax = [ax1 ax2]
    patch(ax, xl([1 2 2 1]), yl([1 1 2 2]), [0 0 0 0], [0.93 0.89 0.82], 'EdgeColor','none', 'FaceLighting','none');
    for g = ceil(xl(1)*10)/10 : 0.1 : xl(2)
        plot3(ax, [g g], yl, [1 1]*1e-4, 'Color',[0.85 0.80 0.72]);
    end
    for g = ceil(yl(1)*10)/10 : 0.1 : yl(2)
        plot3(ax, xl, [g g], [1 1]*1e-4, 'Color',[0.85 0.80 0.72]);
    end
    a = W(A(1),A(2));  b = W(B(1),B(2));
    plot3(ax, [a(1) b(1)], [a(2) b(2)], [2e-4 2e-4], '--', 'Color',[0.3 0.5 1], 'LineWidth',1.2);
end

%% 시작점 A · 끝점 B (펜 직선, XY_setup 의 pen_A / pen_B) 와 로봇 waypoint
th = atan2(B(2)-A(2), B(1)-A(1));  u = [cos(th) sin(th)];
Rl = [cos(th) -sin(th); sin(th) cos(th)];
rA = A - (Rl*P.pen_d).' - u*P.line_pre;     % 로봇(태그) 출발 waypoint — xy_mission 과 같은 식
rB = B - (Rl*P.pen_d).' + u*P.line_post;    % 로봇(태그) 도착 waypoint
a = W(A(1),A(2));  b = W(B(1),B(2));  ra = W(rA(1),rA(2));  rb = W(rB(1),rB(2));
cA = [0.05 0.60 0.15];  cB = [0.85 0.10 0.10];  cW = [0.35 0.35 0.75];
lab = {sprintf('A 시작 (%.2f, %.2f)', A(1), A(2)), sprintf('B 끝 (%.2f, %.2f)', B(1), B(2))};
for ax = [ax1 ax2]
    z = 2.5e-3;
    plot3(ax, a(1), a(2), z, 'o', 'MarkerSize',11, 'LineWidth',2.5, 'MarkerEdgeColor',cA, 'MarkerFaceColor','w');
    plot3(ax, b(1), b(2), z, 's', 'MarkerSize',11, 'LineWidth',2.5, 'MarkerEdgeColor',cB, 'MarkerFaceColor','w');
    plot3(ax, [ra(1) rb(1)], [ra(2) rb(2)], [z z], 'x', 'MarkerSize',10, 'LineWidth',2, 'Color',cW);
end
%  글자는 로봇보다 높은 z 에 두어 로봇에 가리지 않게, Clipping off
zt = 0.25;  T = {'Clipping','off', 'HorizontalAlignment','center', 'FontWeight','bold', 'BackgroundColor','w', 'Margin',1};
text(ax1, a(1), a(2)-0.09, zt, lab{1}, 'Color',cA, 'FontSize',11, T{:});
text(ax1, b(1), b(2)-0.09, zt, lab{2}, 'Color',cB, 'FontSize',11, T{:});
text(ax1, ra(1), ra(2)+0.08, zt, '로봇 WP 시작', 'Color',cW, 'FontSize',9, T{:});
text(ax1, rb(1), rb(2)+0.08, zt, '로봇 WP 끝',   'Color',cW, 'FontSize',9, T{:});
text(ax2, a(1), a(2)-0.03, 0.01, 'A 시작', 'Color',cA, 'FontSize',13, T{:});
text(ax2, b(1), b(2)-0.03, 0.01, 'B 끝',   'Color',cB, 'FontSize',13, T{:});

%% 로봇 STL — 바디 좌표 (x 전진, y 좌측, z 벽 바깥), 원점 = 로봇 중심
TR = stlread(P.stl_file);
[F, V] = reducepatch(TR.ConnectivityList, TR.Points, 0.15);
c  = [mean([min(V(:,1)) max(V(:,1))]), min(V(:,2)), mean([min(V(:,3)) max(V(:,3))])];
Vb = [ (V(:,1)-c(1)), -(V(:,3)-c(3)), (V(:,2)-c(2)) ] / 1000;   % STL x=길이, y=높이, z=폭

g1 = make_robot(ax1, F, Vb, P);
g2 = make_robot(ax2, F, Vb, P);

%% 펜 자국
trail1 = plot3(ax1, NaN, NaN, NaN, 'r-', 'LineWidth',2.5);
trail2 = plot3(ax2, NaN, NaN, NaN, 'r-', 'LineWidth',3.0);
for ax = [ax1 ax2], camlight(ax,'headlight'); lighting(ax,'gouraud'); end

%% 카메라 (전체)
cx = mean(xl); cy = mean(yl);
camtarget(ax1, [cx cy 0]);  campos(ax1, [cx+0.20, cy-0.55, 2.6]);
camva(ax1, 2*atand(0.36*diff(xl)/2.7));
camva(ax2, 24);

%% 아래 그래프
hold(ax3,'on'); grid(ax3,'on'); box(ax3,'on');
e1 = 1000*ep; e1(~on) = NaN;  e2 = 1000*en; e2(~on) = NaN;
plot(ax3, t, e2, 'Color',[0.6 0.6 0.6], 'LineWidth',1.2);
plot(ax3, t, e1, 'r', 'LineWidth',1.5);
cur = xline(ax3, 0, 'b-', 'LineWidth',1.2);
ylabel(ax3, '펜 횡오차 [mm]'); xlabel(ax3, '시간 [s]'); xlim(ax3, [0 t(end)]);
legend(ax3, {'보상 없음 (스테이지 고정)','XY 스테이지 보상'}, 'Location','northwest');
ttl = annotation(fig, 'textbox', [0.02 0.955 0.96 0.04], 'String','', 'EdgeColor','none', ...
                 'FontSize',13, 'FontWeight','bold', 'HorizontalAlignment','left');

%% 프레임
vw = VideoWriter(vfile, 'MPEG-4');  vw.FrameRate = P.anim_fps;  vw.Quality = 90;  open(vw);
tf = 0 : P.anim_speed/P.anim_fps : t(end);
mname = {'1 출발점으로','2 방향 맞추기','3 직선 긋기','4 정지'};
for k = 1:numel(tf)
    i = find(t <= tf(k), 1, 'last');
    psi = q(i,3);
    R = [cos(psi) -sin(psi); sin(psi) cos(psi)];
    ctr = q(i,1:2).' - R*P.r_tag;                    % 로봇 중심 (벽면)
    Xc  = W(ctr(1), ctr(2));
    M  = makehgtform('translate',[Xc 0]) * makehgtform('zrotate', psi + pi/2);
    ds = (sp(i,:) - P.stage_c) / 1000;               % 스테이지 변위 [m]
    for g = [g1 g2]
        set(g.body, 'Matrix', M);
        set(g.sx,   'Matrix', makehgtform('translate',[ds(1) 0 0]));
        set(g.sy,   'Matrix', makehgtform('translate',[0 ds(2) 0]));
    end
    tr = pt(1:i,:);  tr(~on(1:i),:) = NaN;
    Wt = W(tr(:,1), tr(:,2));
    set(trail1, 'XData',Wt(:,1), 'YData',Wt(:,2), 'ZData',1.5e-3*ones(i,1));
    set(trail2, 'XData',Wt(:,1), 'YData',Wt(:,2), 'ZData',1.5e-3*ones(i,1));
    pp = W(pt(i,1), pt(i,2));
    camtarget(ax2, [pp 0.05]);  campos(ax2, [pp(1)+0.12, pp(2)-0.30, 0.85]);
    set(cur, 'Value', t(i));
    if on(i)
        es = sprintf('펜 다운   |   횡오차 %5.2f mm   (보상 없으면 %5.2f mm)', 1000*ep(i), 1000*en(i));
    else
        es = '펜 업';
    end
    set(ttl, 'String', sprintf('t = %5.1f s   |   %s   |   %s', t(i), mname{max(1,min(4,round(md(i))))}, es));
    drawnow;
    writeVideo(vw, getframe(fig));
end
close(vw);
fprintf('  동영상 저장: %s (%d 프레임)\n', vfile, numel(tf));
end

% ---------------------------------------------------------------------
function g = make_robot(ax, F, Vb, P)
%  hgtransform 계층: 로봇 -> 스테이지 X 이동부 -> 스테이지 Y 이동부 -> 펜
g.body = hgtransform('Parent', ax);
patch('Parent',g.body, 'Faces',F, 'Vertices',Vb, 'FaceColor',[0.55 0.57 0.60], ...
      'EdgeColor','none', 'FaceLighting','gouraud', 'AmbientStrength',0.45);
px = P.r_pen0(1);  py = P.r_pen0(2);
green = [0.30 0.68 0.32];  orange = [0.98 0.62 0.15];
%  아래 축 (전진 방향 s_x) — 바닥판 위에 고정
box3(g.body, [px py-0.090 0.112], [0.210 0.045 0.021], green*0.85);
g.sx = hgtransform('Parent', g.body);
box3(g.sx, [px py-0.090 0.127], [0.060 0.045 0.010], green);          % X 캐리지
box3(g.sx, [px py-0.020 0.143], [0.045 0.210 0.021], green*0.85);     % 위 축 (좌우 방향 s_y)
g.sy = hgtransform('Parent', g.sx);
box3(g.sy, [px py 0.159], [0.045 0.060 0.010], green);                % Y 캐리지
box3(g.sy, [px py 0.170], [0.050 0.050 0.008], orange);               % 펜 홀더
[xc, yc, zc] = cylinder(0.006, 16);
surf(xc+px, yc+py, zc*0.20, 'Parent',g.sy, 'FaceColor',[0.1 0.1 0.1], 'EdgeColor','none');   % 펜
[xs, ys, zs] = sphere(10);
surf(0.007*xs+px, 0.007*ys+py, 0.007*zs+0.002, 'Parent',g.sy, 'FaceColor',[0.9 0.1 0.1], 'EdgeColor','none');
end

function box3(parent, c, s, col)
x = c(1) + s(1)/2*[-1 1 1 -1 -1 1 1 -1];
y = c(2) + s(2)/2*[-1 -1 1 1 -1 -1 1 1];
z = c(3) + s(3)/2*[-1 -1 -1 -1 1 1 1 1];
f = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
patch('Parent',parent, 'Faces',f, 'Vertices',[x' y' z'], 'FaceColor',col, ...
      'EdgeColor',[0.2 0.3 0.2], 'FaceLighting','gouraud');
end

% ---------------------------------------------------------------------
function [t, D] = xy_getd(log, name)
%  out.log.<name> (timeseries) -> [시간 N x 1, 값 N x k]
ts = log.(name);
t  = ts.Time(:);
D  = squeeze(ts.Data);
if size(D,1) ~= numel(t), D = D.'; end
end
