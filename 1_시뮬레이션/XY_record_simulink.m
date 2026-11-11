function vfile = XY_record_simulink(vfile)
if nargin == 1 && strcmp(vfile, 'attach'), rec_attach(); return; end
%XY_RECORD_SIMULINK  Simulink 모델이 실행되는 동안의 도면 + 직선 이탈 그래프 + 3D 를 한 동영상으로
%
%   >> XY_record_simulink           -> video/XY_1_simulink_live.mp4
%
%   한 화면 (1600 x 900)
%     위       : XY_1_pen_sim 도면 전체 (모든 블록). 녹화용 Display 블록(노란색)을 빈 자리에 잠깐 붙여
%                로봇 자세 · 스테이지 명령/위치 · 모드 · 펜 오차가 실행 중에 바뀌는 것을 보인다 (1 s 마다 다시 그림)
%     아래 왼쪽: 펜이 직선에서 벗어난 거리 [mm] — 보상 / 보상 없음, 실시간
%     아래 오른쪽: 3D 장면 (전체 벽면 · 펜 확대)
%   모든 값은 시뮬레이션이 끝난 뒤가 아니라, 실행 중인 Simulink 블록 값(RuntimeObject)으로 0.1 s 마다 갱신한다.
%
%   화면 캡처를 쓰지 않는다: 그림 창 하나를 화면 밖에서 렌더링(print -RGBImage)해 프레임을 만든다.
%   그래서 다른 창(메신저 · 알림 등)이 위에 떠도 동영상에 찍히지 않는다.
%   녹화가 끝나면 모델을 저장하지 않고 닫는다 (Display 블록 · StartFcn 은 남지 않는다).
%
%   동영상: 시뮬레이션 0.1 s 마다 1 프레임, 20 fps -> 실제 시간의 2 배속

here = fileparts(mfilename('fullpath'));  cd(here);
if nargin < 1 || isempty(vfile)
    if ~isfolder('video'), mkdir('video'); end
    vfile = fullfile(here, 'video', 'XY_1_simulink_live.mp4');
end
evalin('base', 'XY_setup;');
if ~isfolder('logs'), mkdir('logs'); end
logf = fullfile(here,'logs','record_log.txt');  if isfile(logf), delete(logf); end
L = @(txt) rec_log(logf, txt);  L('setup OK');
m = 'XY_1_pen_sim';
if ~isfile(fullfile(here, [m '.slx'])), build_xy_models(1); end
bdclose('all');
load_system(m);                          % 창은 열지 않는다 — 도면은 print -s 로 화면 밖에서 그린다

%% 1) 녹화용 실시간 표시 블록 — 도면의 빈 자리(제어기 아래, 가상 실험장 왼쪽)에 두 줄로
%     도면 크기가 늘지 않아야 FitSystem 배율이 커진다
x0 = 200;  y0 = 1020;  per_row = 4;
live = { % 태그            이름           게인   설명
    'q_true',           'D_pose',        '',    '로봇 자세 [p1 p2 psi]'
    'stage_target_pos', 'D_stage_cmd',   '',    '스테이지 명령 [mm]  (/stage/target_pos)'
    'stage_pos',        'D_stage_pos',   '',    '스테이지 위치 [mm]  (/stage/pos)'
    'mode',             'D_mode',        '',    '미션 모드'
    'pen_on',           'D_pen_on',      '',    '펜 다운'
    'e_pen',            'D_err_mm',      '1000','펜 직선 오차 [mm]'
    'e_nocomp',         'D_errn_mm',     '1000','보상 없을 때 [mm]'
    'pen_true',         'D_pen',         '',    '펜 끝 위치 [p1 p2]'};
x = x0;  ybase = y0;
for k = 1:size(live,1)
    if k > 1 && mod(k-1, per_row) == 0, x = x0;  y0 = y0 + 160; end
    tag = live{k,1};  dn = live{k,2};
    add_block('simulink/Signal Routing/From', [m '/LF_' dn], 'GotoTag', tag, ...
              'Position', [x y0 x+110 y0+24], 'ShowName','off');
    src = ['LF_' dn '/1'];  xd = x + 140;
    if ~isempty(live{k,3})
        add_block('simulink/Math Operations/Gain', [m '/LG_' dn], 'Gain', live{k,3}, ...
                  'Position', [xd y0-3 xd+40 y0+27], 'ShowName','off');
        add_line(m, src, ['LG_' dn '/1']);  src = ['LG_' dn '/1'];  xd = xd + 60;
    end
    add_block('simulink/Sinks/Display', [m '/' dn], 'Position', [xd y0-14 xd+190 y0+38], ...
              'Format','short', 'BackgroundColor','yellow');
    try, set_param([m '/' dn], 'FontSize', 16, 'FontWeight','bold'); catch, end
    set_param([m '/' dn], 'Name', dn);
    add_line(m, src, [dn '/1']);
    try
        add_block('built-in/Note', [m '/LN_' dn], 'Position', [x+60 y0+60 x+60 y0+60], ...
                  'Text', live{k,4}, 'FontSize', 11);
    catch
    end
    x = xd + 240;
end
try
    add_block('built-in/Note', [m '/LN_title'], 'Position', [x0+300 ybase-50 x0+300 ybase-50], ...
              'Text', '▶ 실행 중 실시간 값 (녹화용 표시 — 저장하지 않음)', 'FontSize', 14, ...
              'ForegroundColor','red');
catch
end

L('live blocks OK');
%% 2) 한 화면 구성 (그림 창 하나)
P = struct();
for nm = {'r_tag','r_pen0','stage_c','stl_file','pen_d','line_pre','line_post','pen_A','pen_B','q0','T_run'}
    P.(nm{1}) = evalin('base', nm{1});
end
fig = figure('Color','w', 'MenuBar','none', 'ToolBar','none', 'NumberTitle','off', ...
             'Name','XY_1_pen_sim — Simulink 실행 녹화', 'Units','pixels', 'Position', [20 40 1600 900]);
%  위: Simulink 도면 (전체)
axd = axes(fig, 'Position',[0 0.47 1 0.47]);
st.diag_png = fullfile(tempdir, 'xy_rec_diagram.png');
print(['-s' m], '-dpng', '-r110', st.diag_png);
warning('off', 'MATLAB:print:ExcludesUIInFutureRelease');
st.himg = image(axd, trim_white(imread(st.diag_png)));  axis(axd, 'image', 'off');
ttl = uicontrol(fig, 'Style','text', 'Units','normalized', 'Position',[0 0.945 1 0.055], ...
                'BackgroundColor','w', 'FontSize',14, 'FontWeight','bold', 'ForegroundColor',[0.75 0 0], ...
                'String','Simulink 실행 준비');
%  아래 왼쪽: 직선 이탈 그래프
axe = axes(fig, 'Position',[0.045 0.06 0.33 0.36]);  hold(axe,'on');  grid(axe,'on');
yline(axe, 0, 'k-');  yline(axe, [-1 1], ':', 'Color',[0.2 0.6 0.2]);
he  = animatedline(axe, 'Color','r', 'LineWidth',2.0);
hn  = animatedline(axe, 'Color',[0.55 0.55 0.55], 'LineWidth',1.5);
xlim(axe, [0 30]);  ylim(axe, [-20 6]);
xlabel(axe, '시간 [s]');  ylabel(axe, '직선에서 벗어난 거리 [mm]');
legend(axe, [he hn], {'XY 스테이지 보상', '보상 없음 (스테이지 중앙 고정)'}, 'Location','southeast');
etl = title(axe, '펜이 직선에서 벗어난 정도 (펜 다운 구간)', 'FontSize',11);
%  아래 오른쪽: 3D
holder = uipanel(fig, 'Position',[0.39 0 0.61 0.46], 'BorderType','none', 'BackgroundColor','w');
S = rec_scene(holder, P, P.pen_A(:).', P.pen_B(:).', P.q0(1:2).', 'h');
S.update(P.q0, P.stage_c, P.pen_A, false);
drawnow;

L('scene OK');
%% 3) 녹화 준비
st.fig = fig;  st.m = m;  st.next_diag = 0;
st.vw    = VideoWriter(vfile, 'MPEG-4');  st.vw.FrameRate = 20;  st.vw.Quality = 85;  open(st.vw);  L('videowriter OK');
tic; I0 = rec_grab(st); L(sprintf('grab OK %dx%d  %.2f s', size(I0,2), size(I0,1), toc));
st.S = S;  st.ttl = ttl;  st.next_t = 0;  st.dt = 0.1;
st.he = he;  st.hn = hn;  st.etl = etl;  st.esum = [0 0];
st.blk = struct('q',[m '/D_pose'], 'stage',[m '/D_stage_pos'], 'pen',[m '/D_pen'], ...
                'on',[m '/D_pen_on'], 'mode',[m '/D_mode'], 'err',[m '/D_err_mm'], 'errn',[m '/D_errn_mm']);
for k = 1:15, writeVideo(st.vw, rec_grab(st)); end      % 시작 전 화면 0.75 s
setappdata(0, 'XYREC', st);

%% 4) 실행 — sim 은 블로킹 호출. 리스너(StartFcn 에서 붙임)가 0.1 s 마다 프레임을 쓴다
%  (set_param ... 'start' 는 이 함수가 끝나야 돌기 시작하므로 쓰지 않는다)
set_param(m, 'StartFcn', 'XY_record_simulink(''attach'')');
L('sim start');
try
    assignin('base','XY_batch',1);  assignin('base','animate',0);
    sim(m, 'ReturnWorkspaceOutputs','on');
    assignin('base','XY_batch',0);  assignin('base','animate',1);
    L('sim OK');
catch e
    L(['sim FAIL: ' getReport(e, 'extended', 'hyperlinks','off')]);
end
st = getappdata(0, 'XYREC');
set(st.ttl, 'String', sprintf('Simulink 실행 완료   t = %.1f s', st.next_t - st.dt));
drawnow;  pause(0.3);
for k = 1:40, writeVideo(st.vw, rec_grab(st)); end      % 끝난 화면 2 s
close(st.vw);
rmappdata(0, 'XYREC');
close(st.fig);
close_system(m, 0);                                          % 녹화용 블록은 저장하지 않는다
L('done');
fprintf('  Simulink 실행 화면 동영상: %s\n', vfile);
end

% =====================================================================
%  아래는 녹화 전용 지역 함수 (StartFcn 리스너 · 화면 캡처 · 3D 장면)
% =====================================================================
function rec_attach()
%XY_REC_ATTACH  (녹화 전용) 모델 StartFcn 에서 호출된다. 실행 중 블록 값을 읽는 리스너를 붙인다.
%   XY_record_simulink 가 StartFcn 에 임시로 넣는다. 모델에는 저장되지 않는다.
st = getappdata(0, 'XYREC');
st.lh = add_exec_event_listener(st.blk.pen, 'PostOutputs', @rec_cb);
setappdata(0, 'XYREC', st);
end

function rec_cb(block, ev)
try
    rec_step(block, ev);
catch e
    fid = fopen(fullfile(pwd,'logs','record_cb_err.txt'), 'a');
    fprintf(fid, '%s\n', getReport(e, 'extended', 'hyperlinks','off'));  fclose(fid);
end
end

function rec_step(block, ~)
%  0.1 s(시뮬레이션 시간)마다: Simulink 블록 값으로 3D 장면 갱신 -> 화면 전체 캡처 -> 동영상 1 프레임
st = getappdata(0, 'XYREC');
t = block.CurrentTime;
if t + 1e-9 < st.next_t, return; end
st.next_t = st.next_t + st.dt;

rv = @(p) double(get_param(p, 'RuntimeObject').InputPort(1).Data);
q   = rv(st.blk.q);      sp = rv(st.blk.stage);   pen = rv(st.blk.pen);
on  = rv(st.blk.on) > 0.5;  md = round(rv(st.blk.mode));  e = rv(st.blk.err);

st.S.update(q, sp, pen, on);
mname = {'1 출발점으로','2 방향 맞추기','3 직선 긋기','4 정지'};
if on
    es = sprintf('펜 다운  |  직선 오차 %5.2f mm', e);
    en = rv(st.blk.errn);
    addpoints(st.he, t, e);  addpoints(st.hn, t, en);
    st.esum = st.esum + [e^2 1];
    set(st.etl, 'String', sprintf('펜이 직선에서 벗어난 정도   지금 %+5.2f mm   |   RMS %4.2f mm   (보상 없음 %+5.1f mm)', ...
        e, sqrt(st.esum(1)/st.esum(2)), en));
else
    es = '펜 업';
end
set(st.ttl, 'String', sprintf('Simulink 실행 중   t = %5.1f s   |   %s   |   %s', ...
    t, mname{max(1, min(4, md))}, es));
if t + 1e-9 >= st.next_diag                       % 도면(Display 값 포함)은 1 s 마다 다시 그린다
    st.next_diag = st.next_diag + 1.0;
    try
        print(['-s' st.m], '-dpng', '-r110', st.diag_png);
        Id = trim_white(imread(st.diag_png));
        set(st.himg, 'CData', Id, 'XData', [1 size(Id,2)], 'YData', [1 size(Id,1)]);
        axis(st.himg.Parent, 'image', 'off');
    catch
    end
end
drawnow;
writeVideo(st.vw, rec_grab(st));
setappdata(0, 'XYREC', st);
end

function I = rec_grab(st)
%XY_REC_GRAB  (녹화 전용) 녹화용 그림 창 하나만 화면 밖에서 렌더링해 RGB 영상으로 돌려준다.
%   화면 캡처가 아니므로 다른 창이 위에 있어도 찍히지 않는다. 크기는 1600 x 900 으로 맞춘다
I = print(st.fig, '-RGBImage', '-r0');
I = imresize(I, [900 1600]);
end

function rec_log(f, txt)
%XY_REC_LOG  (녹화 전용) 진행 기록을 한 줄 덧붙인다
fid = fopen(f, 'a', 'n', 'UTF-8');  fprintf(fid, '%s  %s\n', datestr(now,'HH:MM:SS'), txt);  fclose(fid);
end

function S = rec_scene(parent, P, A, B, pts, layout)
%XY_SCENE  벽면 + 로봇(STL) + XY 스테이지 + 펜 3D 장면을 만든다 (실시간 녹화용)
%
%   S = xy_scene(parent, P, A, B, pts)
%     parent : figure 또는 uipanel
%     P      : r_tag, r_pen0, stage_c, stl_file, pen_d, line_pre, line_post 를 가진 구조체
%     A, B   : 펜 직선 시작점 / 끝점 [p1 p2]
%     pts    : 화면 범위를 정할 벽면 점들 (N x 2, [p1 p2])
%   S.update(q, stage_pos, pen_xy, pen_on)  로 한 프레임 갱신
%
%   XY_animate_video 와 같은 그림 규칙: 화면 X = -p2, Y = p1, Z = 벽 바깥.

W = @(p1, p2) [-p2(:), p1(:)];

th = atan2(B(2)-A(2), B(1)-A(1));  u = [cos(th) sin(th)];
Rl = [cos(th) -sin(th); sin(th) cos(th)];
rA = A - (Rl*P.pen_d).' - u*P.line_pre;
rB = B - (Rl*P.pen_d).' + u*P.line_post;

xy = W([pts(:,1); A(1); B(1); rA(1); rB(1)], [pts(:,2); A(2); B(2); rA(2); rB(2)]);
xl = [min(xy(:,1))-0.25, max(xy(:,1))+0.25];
yl = [min(xy(:,2))-0.25, max(xy(:,2))+0.25];

if nargin < 6, layout = 'v'; end
if strcmp(layout, 'h'), p1 = [0.00 0 0.50 1];  p2 = [0.50 0 0.50 1];
else,                   p1 = [0.00 0.50 1.00 0.50];  p2 = [0.00 0.00 1.00 0.50]; end
pn1 = uipanel(parent, 'Position',p1, 'BackgroundColor','w', 'Title','전체 벽면', 'FontSize',11);
pn2 = uipanel(parent, 'Position',p2, 'BackgroundColor','w', ...
              'Title','펜 확대 — 초록 = XY 스테이지 (움직임), 주황 = 펜 홀더', 'FontSize',11);
ax1 = axes(pn1, 'Position',[0 0 1 1]);
ax2 = axes(pn2, 'Position',[0 0 1 1]);
for ax = [ax1 ax2]
    hold(ax,'on'); axis(ax,'equal'); axis(ax,'off');
    set(ax, 'Projection','perspective', 'CameraUpVector',[0 1 0]);
    patch(ax, xl([1 2 2 1]), yl([1 1 2 2]), [0 0 0 0], [0.93 0.89 0.82], 'EdgeColor','none', 'FaceLighting','none');
    for g = ceil(xl(1)*10)/10 : 0.1 : xl(2), plot3(ax, [g g], yl, [1 1]*1e-4, 'Color',[0.85 0.80 0.72]); end
    for g = ceil(yl(1)*10)/10 : 0.1 : yl(2), plot3(ax, xl, [g g], [1 1]*1e-4, 'Color',[0.85 0.80 0.72]); end
end

a = W(A(1),A(2));  b = W(B(1),B(2));  ra = W(rA(1),rA(2));  rb = W(rB(1),rB(2));
cA = [0.05 0.60 0.15];  cB = [0.85 0.10 0.10];  cW = [0.35 0.35 0.75];
T = {'Clipping','off', 'HorizontalAlignment','center', 'FontWeight','bold', 'BackgroundColor','w', 'Margin',1};
for ax = [ax1 ax2]
    z = 2.5e-3;
    plot3(ax, [a(1) b(1)], [a(2) b(2)], [2e-4 2e-4], '--', 'Color',[0.3 0.5 1], 'LineWidth',1.2);
    plot3(ax, a(1), a(2), z, 'o', 'MarkerSize',11, 'LineWidth',2.5, 'MarkerEdgeColor',cA, 'MarkerFaceColor','w');
    plot3(ax, b(1), b(2), z, 's', 'MarkerSize',11, 'LineWidth',2.5, 'MarkerEdgeColor',cB, 'MarkerFaceColor','w');
    plot3(ax, [ra(1) rb(1)], [ra(2) rb(2)], [z z], 'x', 'MarkerSize',10, 'LineWidth',2, 'Color',cW);
end
zt = 0.25;
text(ax1, a(1), a(2)-0.09, zt, sprintf('A 시작 (%.2f, %.2f)', A(1), A(2)), 'Color',cA, 'FontSize',11, T{:});
text(ax1, b(1), b(2)-0.09, zt, sprintf('B 끝 (%.2f, %.2f)',   B(1), B(2)), 'Color',cB, 'FontSize',11, T{:});
text(ax1, ra(1), ra(2)+0.08, zt, '로봇 WP 시작', 'Color',cW, 'FontSize',9, T{:});
text(ax1, rb(1), rb(2)+0.08, zt, '로봇 WP 끝',   'Color',cW, 'FontSize',9, T{:});
text(ax2, a(1), a(2)-0.03, 0.01, 'A 시작', 'Color',cA, 'FontSize',13, T{:});
text(ax2, b(1), b(2)-0.03, 0.01, 'B 끝',   'Color',cB, 'FontSize',13, T{:});

TR = stlread(P.stl_file);
[F, V] = reducepatch(TR.ConnectivityList, TR.Points, 0.15);
c  = [mean([min(V(:,1)) max(V(:,1))]), min(V(:,2)), mean([min(V(:,3)) max(V(:,3))])];
Vb = [ (V(:,1)-c(1)), -(V(:,3)-c(3)), (V(:,2)-c(2)) ] / 1000;
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
S.ax = [ax1 ax2];

    function update(q, sp, pen, on)
        q = q(:);  psi = q(3);
        R = [cos(psi) -sin(psi); sin(psi) cos(psi)];
        ctr = q(1:2) - R*P.r_tag(:);
        Xc  = W(ctr(1), ctr(2));
        M  = makehgtform('translate',[Xc 0]) * makehgtform('zrotate', psi + pi/2);
        sp = sp(:).';  if isscalar(sp), sp = [sp sp]; end
        c0 = P.stage_c(:).';  if isscalar(c0), c0 = [c0 c0]; end
        ds = (sp - c0) / 1000;
        for g = [g1 g2]
            set(g.body, 'Matrix', M);
            set(g.sx,   'Matrix', makehgtform('translate',[ds(1) 0 0]));
            set(g.sy,   'Matrix', makehgtform('translate',[0 ds(2) 0]));
        end
        if on, trail(end+1,:) = W(pen(1), pen(2)); else, trail(end+1,:) = [NaN NaN]; end
        n = size(trail,1);
        set(tr1, 'XData',trail(:,1), 'YData',trail(:,2), 'ZData',1.5e-3*ones(n,1));
        set(tr2, 'XData',trail(:,1), 'YData',trail(:,2), 'ZData',1.5e-3*ones(n,1));
        pp = W(pen(1), pen(2));
        camtarget(ax2, [pp 0.05]);  campos(ax2, [pp(1)+0.12, pp(2)-0.30, 0.85]);
    end
end

% ---------------------------------------------------------------------
function g = make_robot(ax, F, Vb, P)
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

function I = trim_white(I)
%TRIM_WHITE  (녹화 전용) 도면 그림 둘레의 흰 여백을 잘라 도면이 크게 보이게 한다
nw = any(I < 245, 3);
r = find(any(nw, 2));  c = find(any(nw, 1));
if isempty(r), return; end
p = 8;
I = I(max(1,r(1)-p):min(end,r(end)+p), max(1,c(1)-p):min(end,c(end)+p), :);
end
