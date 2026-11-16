function moved = lay_boxes(sys, names, varargin)
%LAY_BOXES  선이 **하나도 없는** 상자를 도면 아래 한 줄에 나란히 놓는다.
%           Lines up boxes that have **no lines at all** in one row below the diagram.
%
%   lay_boxes('W02_3_goto_turtlesim', {'Animate','Logging'})
%   n = lay_boxes(m, {'Animate','Logging'}, 'Gap', 120, 'Pitch', 170)
%
%   어떤 상자를 말하는가 / which boxes
%       Animate · Logging 처럼 **포트가 없는** 서브시스템이다. 신호를 바깥 선이
%       아니라 안쪽의 From 태그(global)로 받으므로 최상위에는 선이 한 가닥도
%       붙지 않는다. 그래서 옮겨도 선이 하나도 바뀌지 않는다 — 꺾임 수도,
%       check_lines 의 일곱 항목도 그대로다. 자리만 바뀐다.
%       Subsystems **without ports**, such as Animate · Logging. They receive signals through
%       inner From tags (global), not outside lines, so not a single line attaches at the top
%       level. Moving them changes no line — the bend count and the seven check_lines items
%       stay the same. Only the position changes.
%
%   왜 필요한가 / why this exists
%       `Simulink.BlockDiagram.arrangeSystem` 은 층을 신호로 정한다. 선이 없는
%       상자는 어느 층에도 속하지 않으므로 **왼쪽 위 빈자리**로 간다.
%       2026-10-01 에 이렇게 나왔다 —
%       `Simulink.BlockDiagram.arrangeSystem` sets layers by signal. A box with no lines belongs
%       to no layer and goes to **the empty spot at the top left**. On 2026-10-01 this gave —
%
%         W10_0_offline   Logging x =  15 · Animate x = 30
%                         (MotionModel 220 · SensorModel 450 · RateMeter 700)
%         W02_3_goto_…    Animate x = 250 · Logging x = 550
%                         (PoseSubscriber 850)
%         W03_1_frame_…   Logging x = -18   (캔버스 왼쪽 밖 / outside the canvas on the left)
%
%       로깅·화면은 사슬의 **마지막** 단계(measurement)다. 그것이 맨 왼쪽에 서면
%       도면이 오른쪽에서 왼쪽으로 읽히고 `check_flow` 가 어긋남으로 센다.
%       선 검사는 이것을 보지 못한다 — 옮길 선이 없기 때문이다.
%       Logging and display are the **last** stage of the chain (measurement). Standing at the far
%       left, they make the diagram read right to left, and `check_flow` counts it as a violation.
%       The line checks cannot see this — there is no line to move.
%
%       `lay_chain` 의 'Boxes' 가 하는 일이 바로 이것인데, `lay_chain` 을 쓰지 않는
%       빌더(W02 · W03 · W10)는 그 기능에 손이 닿지 않았다. 그래서 따로 둔다.
%       This is exactly what 'Boxes' of `lay_chain` does, but builders that do not use
%       `lay_chain` (W02 · W03 · W10) could not reach that feature. Hence a separate tool.
%
%   어디에 놓는가 / where they go
%       도면에 있는 **모든 것**(블록 사각형 · 블록 이름표 · 선의 점)보다 아래로
%       Gap 만큼 내려간 한 줄이다. 줄이 사슬보다 넓어지면 접는다.
%       `model-layout.md` §1 — "오른쪽 끝 또는 그 아래에 나란히".
%       One row placed Gap below **everything** in the diagram (block rectangles · block name
%       labels · line points). If the row gets wider than the chain, it wraps.
%       `model-layout.md` §1 — "side by side at the right end or below it".
%
%       순서는 **준 순서 그대로** 왼쪽에서 오른쪽이다. 같은 단계의 상자끼리는
%       `check_flow` 가 x 를 따지지 않으므로 읽기 좋은 순서로 적으면 된다.
%       The order is **exactly as given**, left to right. `check_flow` does not compare x among
%       boxes of the same stage, so list them in the order that reads best.
%
%   언제 부르는가 / when to call it
%       `tidy_model` **뒤에** 부른다. tidy_model 안에서 arrangeSystem 이 최상위를
%       다시 놓기 때문이다. 선을 건드리지 않으므로 뒤에 불러도 배선이 상하지 않는다.
%       Call it **after** `tidy_model`, because arrangeSystem inside tidy_model re-places the
%       top level. It does not touch lines, so calling it afterwards does not harm the wiring.
%
%   NAME/VALUE
%     'Gap'    도면 맨 아래에서 얼마나 떨어뜨릴지. 기본 120 / distance below the diagram bottom; default 120
%     'Pitch'  상자 사이의 가로 간격. 기본 170 / horizontal spacing between boxes; default 170
%     'X0'     줄의 왼쪽. 기본 [] — 남은 블록 중 가장 왼쪽에 맞춘다
%              left of the row; default [] — aligned with the leftmost remaining block
%     'Wrap'   줄의 오른쪽 한계. 기본 [] — 남은 블록 중 가장 오른쪽에 맞춘다
%              right limit of the row; default [] — aligned with the rightmost remaining block
%
%   돌려주는 값은 자리를 옮긴 상자의 수다.
%   The return value is the number of boxes moved.

p = inputParser;
p.addParameter('Gap',   120);
p.addParameter('Pitch', 170);
p.addParameter('X0',    []);
p.addParameter('Wrap',  []);
p.parse(varargin{:});
o = p.Results;
moved = 0;

if ischar(names) || isstring(names), names = {char(names)}; end

opened = false;
if ~bdIsLoaded(sys), load_system(sys); opened = true; end

blks = find_system(sys, 'SearchDepth',1, 'Type','Block');
blks = blks(~strcmp(blks, sys));

%  옮길 상자를 고른다. 이름이 없는 모델은 조용히 넘어간다 (gnc_roles 와 같은 규약).
%  **선이 붙은 블록은 옮기지 않는다** — 자리를 바꾸면 선을 다시 그어야 하고,
%  그것은 이 도구의 일이 아니다.
%  Pick the boxes to move. Names not in the model are skipped silently (same convention as gnc_roles).
%  **Blocks with lines attached are not moved** — moving them would need the lines redrawn,
%  which is not this tool's job.
move = {};
for k = 1:numel(names)
    b = [sys '/' names{k}];
    if getSimulinkBlockHandle(b) <= 0, continue, end
    ph = get_param(b, 'PortHandles');
    if ~isempty(ph.Inport) || ~isempty(ph.Outport)
        warning('lay_boxes:ports', '%s 에 포트가 있다 — 옮기지 않는다', names{k});
        continue
    end
    move{end+1} = names{k}; %#ok<AGROW>
end
if isempty(move)
    if opened, close_system(sys, 0); end
    return
end

%  남는 것들의 아래 끝. 사각형만 보면 **이름표**와 **선**이 상자 위로 올라온다
%  Bottom edge of what stays. Looking only at rectangles would let **labels** and **lines** reach over the boxes
stay  = blks(~ismember(cellfun(@(b) get_param(b,'Name'), blks, 'UniformOutput',false), move));
yMax  = -inf;  xMin = inf;  xMax = -inf;
for i = 1:numel(stay)
    r = get_param(stay{i}, 'Position');
    yMax = max(yMax, r(4));  xMin = min(xMin, r(1));  xMax = max(xMax, r(3));
    nb = name_box(stay{i});
    if ~any(isnan(nb))
        yMax = max(yMax, nb(4));  xMin = min(xMin, nb(1));  xMax = max(xMax, nb(3));
    end
end
L = find_system(sys, 'FindAll','on', 'SearchDepth',1, 'Type','line');
for i = 1:numel(L)
    try, q = get_param(L(i), 'Points'); catch, continue, end
    yMax = max(yMax, max(q(:,2)));
    xMin = min(xMin, min(q(:,1)));  xMax = max(xMax, max(q(:,1)));
end
if ~isfinite(yMax)
    if opened, close_system(sys, 0); end
    return
end

x0   = o.X0;   if isempty(x0),   x0   = xMin; end
xLim = o.Wrap; if isempty(xLim), xLim = max(xMax, x0 + 400); end

bx = x0;  by = yMax + o.Gap;  bh = 0;
for k = 1:numel(move)
    b = [sys '/' move{k}];
    r = get_param(b, 'Position');
    w = r(3)-r(1);  h = r(4)-r(2);
    if bx > x0 && bx + w > xLim, bx = x0;  by = by + bh + 70;  bh = 0; end
    set_param(b, 'Position', round([bx, by, bx+w, by+h]));
    bx    = bx + w + o.Pitch;
    bh    = max(bh, h);
    moved = moved + 1;
end

save_system(sys);
if opened, close_system(sys, 0); end
end
