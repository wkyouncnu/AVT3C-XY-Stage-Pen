function f = export_diagram(mdl, folder, sub)
%EXPORT_DIAGRAM  모델의 블록도를 강의노트가 싣는 PNG 로 저장한다.
%                Save a model's block diagram as the PNG the lecture embeds.
%
%   export_diagram('W02_surge_control')            -> ./img/W02_surge_control.png
%   export_diagram(mdl, '/path/to/img')
%   export_diagram(mdl, '', 'Guidance')            -> ./img/<mdl>__Guidance.png
%
%   SUB 를 주면 그 서브시스템 안을 찍는다. 파일 이름은 강의노트가 이미 쓰는
%   '<모델>__<서브시스템>.png' 꼴을 따른다.
%   With SUB, the inside of that subsystem is captured. The file name follows the
%   '<model>__<subsystem>.png' form already used by the lecture notes.
%
%   **빌더의 끝에서 부른다.** 절 스크립트에서 부르지 않는다.
%   블록도는 모델이 바뀔 때 바뀌지, 실험을 돌릴 때 바뀌지 않는다. 절 스크립트가
%   저장하게 두면 모델을 고친 뒤 실험을 돌리지 않은 동안 도면이 뒤처진다.
%   vault_check §6 이 그 뒤처짐을 세므로, 잡히면 그 주차를 다시 돌린다.
%
%   Call this at the end of the builder, not from a runner. A block diagram
%   changes when the model changes, not when an experiment is run; leaving the
%   save to a section script lets the diagram fall behind a model that has been
%   edited but not yet exercised. vault_check §6 counts that staleness, and
%   when it does, the week is run again.
%
%   The diagram depends on the builder and on nothing else: it does not change
%   when a gain changes, so regenerating it from every experiment script is
%   wasted work and leaves the figure's age depending on which script ran last.
%   The builder is also the only place that is guaranteed to have just produced
%   a correct model.
%
%   도면은 빌더에만 달려 있다. 게인이 바뀌어도 도면은 바뀌지 않으므로 실험
%   스크립트마다 다시 만드는 것은 헛일이고, 그림의 나이가 마지막에 돈 스크립트에
%   따라 달라진다. 올바른 모델을 방금 만든 것이 보장되는 곳도 빌더뿐이다.
%
%   -r150 keeps a chain of five subsystems under the 2000 px the lecture PDF
%   can display, which vault_check enforces. 넓은 모델은 그것으로도 넘치므로,
%   찍어 보고 넘치면 **해상도를 낮춰 다시 찍는다.** 어차피 PDF 가 쪽 너비에 맞춰
%   줄이므로, 큰 픽셀로 저장해 봐야 파일만 커지고 보이는 것은 같다.
%   -r150 은 서브시스템 다섯 개짜리 사슬을 강의 PDF 가 보여 줄 수 있는 2000 px
%   아래로 유지하며, vault_check 가 이를 강제한다. Wider models overflow even so;
%   if a capture overflows, it is taken again at a lower resolution. The PDF scales
%   to page width anyway, so saving more pixels only enlarges the file.

if nargin < 2 || isempty(folder), folder = fullfile(pwd, 'img'); end
if ~isfolder(folder), mkdir(folder); end

if nargin < 3 || isempty(sub)
    f   = fullfile(folder, [mdl '.png']);
    tgt = mdl;
else
    %  SUB 는 'Mission/MissionFSM' 처럼 **여러 단 아래**를 가리킬 수 있다.
    %  파일 이름에 슬래시를 그대로 두면 없는 폴더를 만들려 들다 실패하므로
    %  '__' 로 바꾼다 (2026-10-01 W08 의 MissionFSM 에서 재현)
    %  SUB may point several levels down, e.g. 'Mission/MissionFSM'. A slash left in the
    %  file name makes it try to create a missing folder and fail, so it becomes '__'
    %  (reproduced with MissionFSM in W08 on 2026-10-01)
    f   = fullfile(folder, [mdl '__' strrep(sub, '/', '__') '.png']);
    tgt = [mdl '/' sub];
end

%  넘치면 해상도를 낮춰 다시 찍는다. 다만 **픽셀을 줄이는 것으로 넓은 도면 문제를
%  풀 수는 없다** — 한 줄로 길게 늘어선 도면을 2000 px 에 욱여넣으면 37 dpi 가 되어
%  글씨를 읽을 수 없다. 폭은 배치에서 줄인다: lay_chain 의 'Wrap' 으로 사슬을 접으면
%  캔버스가 정사각형에 가까워지고, 같은 2000 px 안에 두 배 해상도가 들어간다.
%  On overflow, capture again at lower resolution. But reducing pixels cannot solve a wide
%  diagram: squeezing a long single-row diagram into 2000 px gives 37 dpi and unreadable
%  text. Width is reduced by layout: folding the chain with 'Wrap' in lay_chain makes the
%  canvas closer to square, fitting twice the resolution into the same 2000 px.
MAXPX = 2000;

opened = false;
if ~bdIsLoaded(mdl), load_system(mdl); opened = true; end

%  세 번으로 모자란 도면이 있다. 한 번 줄여도 목표에 정확히 맞지는 않아서,
%  세 번째 인쇄가 여전히 넘친 채로 파일에 남는다 (2026-09-30 W04_4_inner_loop
%  이 1214x2376 으로 저장됨). 다섯 번까지 줄이고, 목표를 3 % 낮춰 잡아
%  마지막 한 번이 경계에 걸치지 않게 한다.
%  Three attempts are not enough for some diagrams. One reduction does not hit the target
%  exactly, so the third print could remain in the file still overflowing (2026-09-30
%  W04_4_inner_loop saved at 1214x2376). Reduce up to five times, aiming 3 % lower so the
%  last attempt does not land on the boundary.
r = 150;
for t = 1:5
    print(['-s' tgt], '-dpng', sprintf('-r%d', round(r)), f);
    i = imfinfo(f);
    big = max(i.Width, i.Height);
    if big <= MAXPX, break, end
    r = max(40, floor(r * 0.97 * MAXPX / big));
end

if opened, close_system(mdl, 0); end
end
