function vfile = XY_record_live(out, vfile)
%XY_RECORD_LIVE  실시간 대시보드 (XY_animate) 와 같은 화면을 동영상으로 만든다 — 시뮬레이션이 끝난 뒤 재생
%   Make an MP4 of the live dashboard by replaying the simulation log after the run.
%
%   >> XY_record_live                 % 모델을 한 번 빠르게 돌린 뒤 (animate = 0) 재생해 녹화
%   >> XY_record_live(out)            % 이미 있는 결과 (Run 버튼 뒤 base 의 out) 로 녹화
%   -> video/XY_1_live_view.mp4
%
%   왜 재생인가
%     녹화를 실행 중에 하면 매 프레임 그림 창을 렌더링하느라 시뮬레이션이 몇 배 느려진다.
%     기록(out.log)에는 대시보드가 쓰는 신호가 모두 있으므로, 끝난 뒤 0.1 s 간격으로 같은 함수
%     XY_animate 에 넣어 프레임을 만든다. 화면은 실행 중 대시보드와 같다.
%   프레임은 그림 창 하나를 화면 밖에서 렌더링 (print -RGBImage) — 화면 캡처가 아니다.
%   0.1 s 마다 1 프레임, 20 fps -> 실제 시간의 2 배속.
%   Replays out.log through XY_animate at 0.1 s steps; off-screen render of one figure, 20 fps (2x).

here = fileparts(mfilename('fullpath'));  cd(here);
if nargin < 2 || isempty(vfile)
    if ~isfolder('video'), mkdir('video'); end
    vfile = fullfile(here, 'video', 'XY_1_live_view.mp4');
end
evalin('base', 'XY_setup;');
if nargin < 1 || isempty(out)
    m = 'XY_1_pen_sim';
    load_system(m);
    assignin('base', 'animate', 0);  assignin('base', 'XY_batch', 1);
    c = onCleanup(@() evalin('base', 'animate = 1; XY_batch = 0;'));
    out = sim(m, 'ReturnWorkspaceOutputs', 'on');
end
L = out.log;

dt = 0.1;  t_end = L.q_true.Time(end);
close(findobj('Type','figure', 'Tag','XY_live'));
ws = warning('off', 'all');  cw = onCleanup(@() warning(ws));   % print 의 UI 컴포넌트 경고 (패널 제목은 그대로 찍힌다)
vw = VideoWriter(vfile, 'MPEG-4');  vw.FrameRate = 20;  vw.Quality = 90;  open(vw);
for t = 0:dt:t_end
    g = @(name) at(L.(name), t);
    XY_animate(g('q_true'), g('pen_true'), g('pen_on'), g('stage_pos'), g('lineAB'), g('e_pen'), ...
               g('v_cmd'), g('w_cmd'), g('V_true'), g('mode'), t);
    f = findobj('Type','figure', 'Tag','XY_live');
    drawnow;
    I = print(f, '-RGBImage', '-r0');
    writeVideo(vw, imresize(I, [900 1600]));
end
close(vw);
fprintf('녹화 완료: %s  (%.1f s 시뮬레이션, %d 프레임)\n', vfile, t_end, numel(0:dt:t_end));
end

function v = at(ts, t)
%  시각 t 직전의 기록값 (zero-order hold) / last logged sample at or before t
k = find(ts.Time <= t + 1e-9, 1, 'last');  if isempty(k), k = 1; end
d = ts.Data;
if ndims(d) == 3, v = d(:, :, k); else, v = d(k, :).'; end
v = double(v(:));
end
