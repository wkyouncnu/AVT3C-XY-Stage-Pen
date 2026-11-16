function lay_chain(m, stages, varargin)
%LAY_CHAIN  최상위를 GNC 신호 사슬로 배치하고, 모든 선을 규칙대로 다시 긋는다.
%           Lays out the top level as the GNC signal chain and redraws every line by the rules.
%
%   lay_chain('W05_0_offline', {'Guidance','InnerLoop','Thrusters','MotionModel'})
%   lay_chain(m, stages, 'Row', 400, 'Pitch', 330, 'Boxes', {'Animate','Logging'})
%
%   하는 일 / what it does
%     1) 사슬 단계를 한 줄 위에 **가운데 정렬**로 놓는다 (왼쪽 -> 오른쪽)
%        Places the chain stages on one row, **centre-aligned** (left -> right)
%     2) 최상위의 모든 신호선을 지웠다가 다시 긋는다
%        Deletes and redraws every signal line at the top level
%        - 사슬 단계끼리 : 통로 x = 출발 포트 x  -> 꺾임 1회
%          between chain stages: lane x = source port x -> 1 bend
%        - Goto  : 출발 포트 높이로 옮기고 오른쪽에 붙인다 -> **직선**
%          Goto  : moved to the source port height, attached on the right -> **straight**
%        - From  : 도착 포트 높이로 옮기고 왼쪽에 붙인다  -> **직선**
%          From  : moved to the destination port height, attached on the left -> **straight**
%     3) 포트 없는 상자(Animate·Logging)는 사슬 아래에 나란히 놓는다
%        Boxes without ports (Animate · Logging) are lined up below the chain
%
%   왜 tidy_layout 을 쓰지 않는가 / why not tidy_layout
%       tidy_layout 은 autorouting 에 배선을 맡긴다. autorouting 은 짧은 경로를
%       고르지 **읽기 좋은** 경로를 고르지 않는다. 그래서 같은 통로를 여러 신호가
%       나눠 쓰고(겹침), 블록 사이가 좁으면 블록 위를 지나간다(관통).
%       배치를 정하면 배선은 따라온다 — references/line-routing.md
%       tidy_layout leaves the wiring to autorouting. Autorouting picks short routes, not
%       **readable** ones, so several signals share a lane (overlap) and, where blocks are
%       close, lines pass over blocks (crossing). Fix the layout and the wiring follows —
%       references/line-routing.md
%
%   NAME/VALUE
%     'Row'    사슬이 놓이는 높이. 기본 40 — 도면 **맨 위**에서 시작한다
%              height of the chain; default 40 — starts at the **top** of the diagram
%     'Pitch'  단계 사이의 간격. 기본 190. 태그 열과 통로가 들어갈 만큼만 띄운다.
%              태그를 내는 블록 뒤에서는 `26*태그수 + 120` 으로 저절로 넓어진다
%              spacing between stages; default 190, just enough for the tag column and lanes.
%              After a block that emits tags it widens automatically to `26*tags + 120`
%     'X0'     첫 단계의 왼쪽. 기본 40 — 도면 **맨 왼쪽**에서 시작한다
%              left of the first stage; default 40 — starts at the **far left** of the diagram
%     'Boxes'  포트 없는 서브시스템 이름들. 사슬 **바로 아래**에 놓는다
%              names of port-less subsystems; placed **right below** the chain
%     'Gap'    태그 블록과 포트 사이의 간격. 기본 45 / gap between tag block and port; default 45
%     'Wrap'   한 줄에 놓을 단계 수. 기본 inf (한 줄) / stages per row; default inf (one row)
%
%   빈 공간을 남기지 않는다 (교수 지시 2026-10-01)
%   No empty space left (instructor's instruction 2026-10-01)
%
%       *"블록을 띄엄띄엄 놓지 말고 가까이 모아 주세요. 빈 공간이 너무 많습니다."*
%       *"Do not scatter the blocks; gather them close together. There is too much empty space."*
%
%       `W05_1_vrx` 는 사슬이 x = 260, y = 400 에서 시작하고 로깅 상자가 사슬에서
%       120 px 아래 떨어져 있어 최상위 **채움 비율이 11.5 %** 였다 — 그림의 열에
%       아홉이 빈 칸이다. 도면이 넓어지면 PDF 쪽폭에 맞출 때 **글씨가 작아진다.**
%       같은 내용이면 작은 도면이 낫다.
%       In `W05_1_vrx` the chain started at x = 260, y = 400 and the logging box sat 120 px
%       below the chain, so the top-level **fill ratio was 11.5 %** — nine tenths of the picture
%       empty. A wider diagram means **smaller text** when fitted to the PDF page width.
%       For the same content, a smaller diagram is better.
%
%       그래서 시작 좌표를 원점 쪽으로 당기고(260·400 -> 40·40), 단계 간격을
%       340 -> 190 으로 줄이고, 태그 더미·줄 간격·상자 간격을 같이 좁혔다.
%       좁힌 뒤에도 일곱 항목·흐름·태그·색은 그대로 0 이어야 한다. 겹치면
%       **그만큼만** 다시 벌린다 — 눈대중이 아니라 `check_fill` 로 재서 견준다.
%       So the start was pulled toward the origin (260·400 -> 40·40), stage spacing cut from
%       340 to 190, and tag stacks, row gaps and box gaps narrowed with it. After narrowing,
%       the seven items, flow, tags and colour must still be 0. Where things overlap, widen
%       **only that much** again — measured and compared with `check_fill`, not by eye.
%
%   줄 접기 / wrapping the chain
%       다섯 칸짜리 사슬을 한 줄에 놓으면 캔버스가 2700 x 800 이 된다. 가로세로 비가
%       3:1 이 넘으면 그림을 쪽 너비에 맞추는 순간 세로가 남고, 글씨는 읽을 수
%       없을 만큼 작아진다. **같은 픽셀 예산이면 정사각형에 가까울수록 크게 보인다.**
%       A five-stage chain on one row gives a 2700 x 800 canvas. Beyond a 3:1 aspect ratio,
%       fitting the picture to page width leaves vertical space unused and the text becomes
%       too small to read. **For the same pixel budget, closer to square looks bigger.**
%
%       'Wrap' 을 주면 사슬을 여러 줄로 접는다. 줄이 바뀌는 자리의 연결은
%       Goto/From 한 쌍으로 바꾼다 — 선으로 이으면 오른쪽 끝에서 왼쪽 끝으로
%       거슬러 올라가야 해서 도면을 가로지른다.
%       With 'Wrap' the chain folds into several rows. Connections at a row change become a
%       Goto/From pair — a line would have to run back from the right end to the left end,
%       crossing the whole diagram.
%
%       측정 (W09_0_offline, 39블록) — 한 줄 3660 x 624 에서 두 줄 1900 x 1290 으로.
%       2000 px 안에 담을 때 39 dpi 에서 79 dpi 로, 글씨 크기가 두 배가 된다.
%       Measured (W09_0_offline, 39 blocks) — from one row 3660 x 624 to two rows 1900 x 1290.
%       Fitted into 2000 px, 39 dpi becomes 79 dpi: text twice the size.

p = inputParser;
p.addParameter('Row',    40);
p.addParameter('Pitch', 190);
p.addParameter('X0',     40);
p.addParameter('Boxes', {});
p.addParameter('Gap',   45);
p.addParameter('Wrap',  inf);
p.parse(varargin{:});
o = p.Results;

%  도면을 모으는 치수. 한곳에 모아 두어야 "얼마나 좁혔는가" 를 한눈에 읽는다.
%  (교수 지시 2026-10-01 — 빈 공간을 남기지 않는다)
%  Dimensions that compact the diagram. Kept in one place so "how much was narrowed" reads at a glance.
%  (Instructor's instruction 2026-10-01 — no empty space left)
TAG_DROP  = 48;    % 블록 아래 테두리에서 첫 태그까지 (전 70) / block bottom edge to first tag (was 70)
TAG_STEP  = 42;    % 태그끼리의 세로 간격       (전 45). 더 좁히면 이름표가 겹친다 / vertical tag spacing (was 45); narrower overlaps labels
TAG_LANE  = 26;    % 태그 통로끼리의 가로 간격   (그대로 — 좁히면 두 신호가 한 통로를 나눠 쓴다) / horizontal spacing of tag lanes (unchanged — narrower makes two signals share a lane)
ROW_GAP   = 36;    % 사슬 줄과 줄 사이          (전 60) / between chain rows (was 60)
BOX_DROP  = 50;    % 사슬 아래 상자 줄까지      (전 120) / chain to the box row below (was 120)
BOX_GAPX  = 70;    % 상자끼리 가로 간격         (전 170) / horizontal gap between boxes (was 170)
BOX_GAPY  = 46;    % 상자 줄끼리 세로 간격      (전 70) / vertical gap between box rows (was 70)
LANE_MIN  = 70;    % 사슬 통로가 출발 포트에서 떨어지는 최소 거리 (전 150) / minimum distance of a chain lane from its source port (was 150)

opened = false;
if ~bdIsLoaded(m), load_system(m); opened = true; end

% --- 1) 연결을 기억한다 ------------------------------------------------
% --- 1) remember the connections ------------------------------------------
conn = grab(m);

%  블록마다 태그가 몇 개 달리는지 먼저 센다. 태그 열은 통로들 오른쪽에 와야 하고,
%  단계 사이 간격은 그 열이 들어갈 만큼은 되어야 한다. 넉넉히 잡으면 사슬 한 칸이
%  그만큼 넓어져 일곱 칸짜리 도면이 8000 px 을 넘는다 (2026-09-17 측정).
%  First count how many tags hang on each block. The tag column must come right of the lanes,
%  and the stage spacing must at least fit that column. Being generous widens every chain
%  step, and a seven-stage diagram exceeds 8000 px (measured 2026-09-17).
cnt = containers.Map('KeyType','char','ValueType','double');
for i = 1:numel(conn)
    if is_type(m, conn(i).dst, 'Goto')
        if ~isKey(cnt, conn(i).src), cnt(conn(i).src) = 0; end
        cnt(conn(i).src) = cnt(conn(i).src) + 1;
    end
end




% --- 2) 선을 전부 지운다 ----------------------------------------------
% --- 2) delete all lines ---------------------------------------------------
L = find_system(m, 'FindAll','on', 'SearchDepth',1, 'Type','line');
for i = 1:numel(L)
    try, delete_line(L(i)); catch, end
end

% --- 3) 사슬 단계를 줄에 나누어 놓는다 ---------------------------------
% --- 3) place the chain stages in rows ------------------------------------
%   포트가 많은 블록은 **키운다.** 포트 간격이 52 px 보다 좁으면 옆에 붙는 태그가
%   서로 겹치고, 겹친 태그를 피하느라 선이 네 번 꺾인다 (2026-09-16 재현).
%   Blocks with many ports are **made taller.** With port pitch under 52 px the tags beside
%   them overlap, and lines bend four times avoiding them (reproduced 2026-09-16).
%  ceil(k/Inf) 은 0 이라 줄 번호가 전부 0 이 되고, 뒤의 줄 반복이 한 번도 돌지
%  않아 블록이 하나도 놓이지 않는다. 한 줄이면 단계 수를 그대로 쓴다.
%  ceil(k/Inf) is 0, so every row number would be 0, the row loop below would never run and
%  no block would be placed. For one row the number of stages is used as is.
wrap  = max(1, min(o.Wrap, numel(stages)));
rowOf = containers.Map('KeyType','char','ValueType','double');
for k = 1:numel(stages), rowOf(stages{k}) = ceil(k / wrap); end

%  줄이 바뀌는 자리의 연결은 곧 태그 한 쌍이 된다 (3.5). 태그가 늘면 그 블록 아래
%  더미도 깊어지므로, 자리를 잡기 **전에** 미리 세어 둔다.
%  A connection at a row change becomes a tag pair (3.5). More tags make the stack under that
%  block deeper, so they are counted **before** positions are set.
for i = 1:numel(conn)
    c = conn(i);
    if ~isKey(rowOf, c.src) || ~isKey(rowOf, c.dst), continue, end
    if rowOf(c.src) == rowOf(c.dst), continue, end
    if ~isKey(cnt, c.src), cnt(c.src) = 0; end
    cnt(c.src) = cnt(c.src) + 1;
end

%  크기를 먼저 정하고, 줄마다 **가장 큰 블록**에 맞춰 높이를 잡는다. 한 블록의
%  가운데에 맞춰 줄을 시작하면 그 줄의 키 큰 블록이 위아래로 삐져나와 옆 줄과
%  겹친다 (2026-09-17 W07_0 에서 InnerLoop 와 MotionModel 이 40 px 겹쳤다).
%  Sizes are set first, and each row's height follows its **largest block**. Starting a row on
%  one block's centre lets the row's tall block stick out and overlap the next row
%  (2026-09-17 in W07_0, InnerLoop and MotionModel overlapped by 40 px).
W = zeros(1,numel(stages));  H = zeros(1,numel(stages));  D = zeros(1,numel(stages));
for k = 1:numel(stages)
    r  = get_param([m '/' stages{k}], 'Position');
    ph = get_param([m '/' stages{k}], 'PortHandles');
    n  = max([numel(ph.Inport), numel(ph.Outport), 2]);
    W(k) = r(3)-r(1);
    H(k) = max(r(4)-r(2), 52*(n-1) + 40);
    nt   = 0;  if isKey(cnt, stages{k}), nt = cnt(stages{k}); end
    D(k) = TAG_DROP + TAG_STEP*max(nt-1,0) + 26;   % 블록 아래 태그 더미의 깊이 / depth of the tag stack under the block
end

%  단계마다 **다음 단계로 몇 가닥이 나가는가**. 사슬 통로는 가닥마다 제 x 가
%  있어야 하므로, 칸 너비를 정할 때 이 수가 필요하다. 같은 출발 포트에서
%  갈라지는 팬아웃은 줄기를 함께 쓰므로 한 가닥으로 센다.
%  For each stage, **how many lines go to the next stage**. Each chain lane needs its own x,
%  so this number is needed to size the step. A fan-out from the same source port shares a
%  trunk and counts as one line.
nChain = containers.Map('KeyType','char','ValueType','double');
seen   = containers.Map('KeyType','char','ValueType','logical');
for i = 1:numel(conn)
    c = conn(i);
    if ~isKey(rowOf, c.src) || ~isKey(rowOf, c.dst), continue, end
    if rowOf(c.src) ~= rowOf(c.dst), continue, end     % 줄이 바뀌면 태그가 된다 / a row change becomes a tag
    kk = sprintf('%s|%d', c.src, c.sp);
    if isKey(seen, kk), continue, end
    seen(kk) = true;
    if ~isKey(nChain, c.src), nChain(c.src) = 0; end
    nChain(c.src) = nChain(c.src) + 1;
end

nRow = 0;
for k = 1:numel(stages), nRow = max(nRow, rowOf(stages{k})); end
yTop = o.Row;
for rr = 1:nRow
    idx  = find(cellfun(@(s) rowOf(s) == rr, stages));
    hMax = max(H(idx));
    yc   = yTop + hMax/2;
    x    = o.X0;
    for k = idx
        set_param([m '/' stages{k}], 'Position', ...
                  round([x, yc-H(k)/2, x+W(k), yc+H(k)/2]));
        %  간격은 **그 칸에 실제로 들어가야 하는 것**에 맞춘다. 태그가 여덟 개인
        %  블록 하나 때문에 모든 칸을 넓히면 도면이 통째로 늘어난다 (W05_1_vrx 가
        %  그랬다). 그렇다고 한 값으로 못박으면 빽빽한 칸에서 통로가 포개진다
        %  (2026-10-01 W07·W08 의 `Guidance -> InnerLoop` 두 가닥이 그랬다).
        %
        %  한 칸에 들어가는 것은 셋이다 — 왼쪽부터
        %    1) 내는 블록 아래로 내려가는 **Goto 통로**   18 + TAG_LANE*(태그수-1)
        %    2) 다음 단계로 가는 **사슬 통로**            13 px 씩, 가닥 수만큼
        %    3) 받는 포트 왼쪽의 **From 태그 열**         Gap + 태그폭 + 이름표
        %  The spacing fits **what must actually go into that step**. Widening every step
        %  because one block has eight tags stretches the whole diagram (as W05_1_vrx did).
        %  Fixing one value, though, makes lanes pile up in crowded steps
        %  (2026-10-01, the two `Guidance -> InnerLoop` lines of W07·W08).
        %
        %  Three things go into a step — from the left
        %    1) the **Goto lanes** going down from the source block   18 + TAG_LANE*(tags-1)
        %    2) the **chain lanes** to the next stage                 13 px each, per line
        %    3) the **From tag column** left of the receiving port    Gap + tag width + label
        g  = o.Pitch;
        nt = 0;  if isKey(cnt, stages{k}),   nt = cnt(stages{k});   end
        nc = 0;  if isKey(nChain, stages{k}), nc = nChain(stages{k}); end
        need = max(18 + TAG_LANE*max(nt-1,0), LANE_MIN) + 13*nc + (o.Gap + 70 + 15) + 20;
        g = max(g, need);
        x = x + W(k) + g;
    end
    %  태그 더미는 **각 블록의 아래 테두리**에서 D 만큼 내려간다. 줄 가운데(yc)에서 재면
    %  키 큰 블록의 태그가 다음 줄에 닿아 이름표가 가려진다 (2026-09-19 W05_0 의
    %  Go_psi_ref 가 MotionModel 에 — check_lines 의 (6) 블록겹침)
    %  The tag stack goes D down from **each block's bottom edge**. Measured from the row centre
    %  (yc), a tall block's tags reach the next row and labels get covered (2026-09-19 W05_0,
    %  Go_psi_ref on MotionModel — item (6) block overlap of check_lines)
    yTop = yc + max(hMax/2, max(H(idx)/2 + D(idx))) + ROW_GAP;
end
yBot = yTop - ROW_GAP;

% --- 3.5) 줄이 바뀌는 자리는 태그 한 쌍으로 -----------------------------
% --- 3.5) a row change becomes one tag pair --------------------------------
%   오른쪽 끝에서 왼쪽 끝으로 선을 끌면 도면을 통째로 가로지른다. 사슬이 줄을
%   바꾸는 자리에서는 신호에 이름을 붙여 내보내고 받는다.
%   Dragging a line from the right end to the left end crosses the whole diagram. Where the
%   chain changes row, the signal is named, sent out and received.
isStage = @(nm) isKey(rowOf, nm);
for i = 1:numel(conn)
    c = conn(i);
    if ~isStage(c.src) || ~isStage(c.dst), continue, end
    if rowOf(c.src) == rowOf(c.dst), continue, end
    tag = matlab.lang.makeValidName(sprintf('%s_%d', c.src, c.sp));
    j   = 0;
    while ~isempty(find_system(m,'SearchDepth',1,'BlockType','Goto','GotoTag',tag))
        j = j + 1;
        tag = matlab.lang.makeValidName(sprintf('%s_%d_%d', c.src, c.sp, j));
    end
    g = add_block('simulink/Signal Routing/Goto', [m '/Go_' tag], ...
                  'MakeNameUnique','on', 'Position',[0 0 60 22], ...
                  'GotoTag', tag, 'TagVisibility','local');
    f = add_block('simulink/Signal Routing/From', [m '/Fr_' tag], ...
                  'MakeNameUnique','on', 'Position',[0 0 60 22], 'GotoTag', tag);
    gn = get_param(g, 'Name');  fn = get_param(f, 'Name');
    conn(i) = struct('src', c.src, 'sp', c.sp, 'dst', gn, 'dp', 1);
    conn(end+1) = struct('src', fn, 'sp', 1, 'dst', c.dst, 'dp', c.dp); %#ok<AGROW>
end

% --- 4) 포트 없는 상자는 맨 아래에 --------------------------------------
% --- 4) port-less boxes at the bottom --------------------------------------
%   아래 줄도 사슬 폭을 넘으면 접는다. 상자 몇 개 때문에 도면 전체가 옆으로
%   늘어나면 사슬을 접은 뜻이 없다 (2026-09-17 W09_1 이 그랬다).
%   The bottom row also wraps when wider than the chain. If a few boxes stretched the whole
%   diagram sideways, folding the chain would be pointless (as W09_1 did on 2026-09-17).
xLim = o.X0;
for k = 1:numel(stages)
    r    = get_param([m '/' stages{k}], 'Position');
    xLim = max(xLim, r(3));
end
bx = o.X0;  by = yBot + BOX_DROP;  bh = 0;
for k = 1:numel(o.Boxes)
    b = [m '/' o.Boxes{k}];
    if ~ismember(b, find_system(m,'SearchDepth',1,'Type','Block')), continue, end
    r = get_param(b, 'Position');
    w = r(3)-r(1);  h = r(4)-r(2);
    if bx > o.X0 && bx + w > xLim, bx = o.X0;  by = by + bh + BOX_GAPY;  bh = 0; end
    set_param(b, 'Position', round([bx, by, bx+w, by+h]));
    bx = bx + w + BOX_GAPX;
    bh = max(bh, h);
end

%  사슬에도 상자에도 들지 않은 블록 — 태그에만 신호를 주는 상수 따위. 자리를 정해
%  주지 않으면 빌더가 놓았던 자리에 그대로 남아 도면을 옆으로 늘린다
%  (2026-09-17 W09_1 의 ZeroEnv·ZeroThr 가 x=2440 에 남아 폭이 2685 였다).
%
%  **사슬로 신호를 주는 블록은 내리지 않는다.** 그런 블록은 받는 포트 옆에 있어야
%  선이 곧다. 아래로 내리면 화면을 가로질러 올라오며 남의 태그를 뚫는다
%  (2026-09-17 W08_1 의 URf2 가 Go_chi 를 뚫었다).
%  Blocks in neither the chain nor the boxes — e.g. constants feeding only tags. Without a set
%  position they stay where the builder put them and stretch the diagram sideways
%  (2026-09-17, ZeroEnv·ZeroThr of W09_1 left at x=2440 made the width 2685).
%
%  **Blocks feeding the chain are not moved down.** They must sit beside the receiving port for
%  the line to be straight. Moved down, the line climbs across the screen and cuts other tags
%  (2026-09-17, URf2 of W08_1 cut through Go_chi).
rest = find_system(m, 'SearchDepth',1, 'Type','Block');
for k = 1:numel(rest)
    nmk = get_param(rest{k}, 'Name');
    if strcmp(rest{k}, m), continue, end
    if ismember(nmk, stages) || ismember(nmk, o.Boxes), continue, end
    if any(strcmp(get_param(rest{k},'BlockType'), {'Goto','From'})), continue, end
    feeds = false;
    for i = 1:numel(conn)
        if strcmp(conn(i).src, nmk) && isKey(rowOf, conn(i).dst), feeds = true; break, end
    end
    if feeds, continue, end          % 사슬 먹임 — 5)에서 받는 포트 옆에 놓는다 / feeds the chain — placed beside the port in 5)

    %  **받기만 하는 종착 블록**(Terminator·Display)은 바닥으로 쓸어내리지 않는다.
    %  내는 포트 바로 오른쪽에 두면 잇는 선이 수평 한 토막, 꺾임 0회다. 바닥으로
    %  내리면 두 종착이 같은 통로로 나란히 내려가 서로 겹치고 상대를 관통한다
    %  (2026-10-01 W07_1_vrx 의 EndChi·EndU 가 그랬다).
    %  **Receive-only end blocks** (Terminator · Display) are not swept to the bottom. Placed just
    %  right of the source port, the connecting line is one horizontal segment with 0 bends. Moved
    %  to the bottom, two end blocks go down the same lane side by side, overlapping and cutting
    %  each other (as EndChi·EndU of W07_1_vrx did on 2026-10-01).
    ph = get_param(rest{k}, 'PortHandles');
    if isempty(ph.Outport) && numel(ph.Inport) == 1
        srcNm = '';  srcPt = 0;
        for i = 1:numel(conn)
            if strcmp(conn(i).dst, nmk) && isKey(rowOf, conn(i).src)
                srcNm = conn(i).src;  srcPt = conn(i).sp;  break
            end
        end
        if ~isempty(srcNm)
            q = port_xy(m, srcNm, 'Outport', srcPt);
            move_to(m, nmk, q(1) + 60, q(2), 'left');
            continue
        end
    end

    r = get_param(rest{k}, 'Position');
    w = r(3)-r(1);  h = r(4)-r(2);
    if bx > o.X0 && bx + w + 130 > xLim, bx = o.X0;  by = by + bh + BOX_GAPY;  bh = 0; end
    set_param(rest{k}, 'Position', round([bx, by, bx+w, by+h]));
    bx = bx + w + BOX_GAPX + 30;
    bh = max(bh, max(h, 100));       % 이 상수의 태그가 아래에 쌓인다 / this constant's tags stack below it
end
yBot = by + bh;

% --- 5) 태그 블록을 짝의 포트 높이로 옮긴다 ------------------------------
% --- 5) move tag blocks to the height of their partner port ----------------
%   그러면 태그로 가는 선이 **직선 한 토막**이 된다. 태그는 신호 사슬이 아니라
%   "여기서 이 신호가 나간다" 는 표시이므로, 포트 바로 옆에 있어야 읽힌다.
%   같은 자리에 둘이 겹치면 한쪽을 아래로 밀어 둔다.
%   Then the line to a tag is **one straight segment**. A tag is not part of the signal chain
%   but a mark that "this signal leaves here", so it reads only right beside the port.
%   If two land on the same spot, one is pushed down.
%   Goto 는 포트 높이에 두지 않는다. 같은 포트에서 본선도 나가기 때문에,
%   태그를 포트 옆에 두면 본선이 **태그를 관통한다** (2026-09-16 재현).
%   그래서 태그는 내는 블록 **아래**에 한 열로 쌓는다.
%   Gotos are not placed at port height. The main line also leaves the same port, so a tag
%   beside the port would have the main line **cut through it** (reproduced 2026-09-16).
%   So tags are stacked in one column **below** the source block.
%
%   통로 x 는 **도면 전체에서** 겹치지 않아야 한다. 사슬을 여러 줄로 접으면 윗줄과
%   아랫줄의 같은 칸이 x 를 나눠 쓰게 되고, 두 신호의 세로 토막이 포개진다
%   (2026-09-17 W05_0 을 두 줄로 접었을 때 재현). 높이가 겹치는 통로끼리만
%   따지면 되므로, 쓰고 있는 [x, 위, 아래] 를 적어 두고 부딪히면 옆으로 민다.
%   Lane x must not clash **across the whole diagram**. Folding the chain into rows makes the
%   same step in the upper and lower rows share x, and two signals' vertical segments pile up
%   (reproduced 2026-09-17 when W05_0 was folded into two rows). Only lanes overlapping in
%   height matter, so the used [x, top, bottom] is recorded and a clash is pushed sideways.
used  = zeros(0, 2);
busy  = zeros(0, 3);
nTag  = containers.Map('KeyType','char','ValueType','double');
lane  = containers.Map('KeyType','char','ValueType','double');
tagY  = containers.Map('KeyType','char','ValueType','double');
colOf = containers.Map('KeyType','char','ValueType','double');
edgeL = containers.Map('KeyType','char','ValueType','double');

for i = 1:numel(conn)
    c = conn(i);
    if is_type(m, c.dst, 'Goto')
        r = get_param([m '/' c.src], 'Position');
        if ~isKey(nTag, c.src), nTag(c.src) = 0; end
        j = nTag(c.src);  nTag(c.src) = j + 1;
        ty = r(4) + TAG_DROP + TAG_STEP*j;
        lx = r(3) + 18 + TAG_LANE*j;
        while ~isempty(busy) && ...
              any(abs(busy(:,1)-lx) < 1 & busy(:,2) < ty-1 & r(2) < busy(:,3)-1)
            lx = lx + TAG_LANE;
        end
        busy(end+1,:) = [lx, r(2), ty]; %#ok<AGROW>
        lane(c.dst)   = lx;
        tagY(c.dst)   = ty;
        if ~isKey(colOf, c.src), colOf(c.src) = r(3) + 150; end
        colOf(c.src) = max(colOf(c.src), lx + 40);   % 태그 열은 마지막 통로보다 오른쪽 / tag column right of the last lane
    elseif isKey(rowOf, c.dst) && ~isKey(rowOf, c.src) && ...
           ~ismember(c.src, o.Boxes) && ~is_type(m, c.src, 'Goto')
        %  From 태그와 사슬을 먹이는 상수 — 둘 다 받는 포트 **왼쪽**에 둔다.
        %  그러면 선이 곧고, 무엇이 그 포트로 들어오는지 한눈에 읽힌다.
        %  From tags and constants feeding the chain — both go **left** of the receiving port.
        %  Then the line is straight and what enters that port reads at a glance.
        q = port_xy(m, c.dst, 'Inport', c.dp);
        [x, y] = free_slot(used, q(1)-o.Gap, q(2));
        used(end+1,:) = [x y]; %#ok<AGROW>
        move_to(m, c.src, x, y, 'right');                 % 출력이 오른쪽 테두리 / output on the right border
        %  이 태그 열의 **이름표까지**가 사슬 통로의 오른쪽 한계다. 통로를 그
        %  안쪽에 두면 세로 토막이 태그 이름 위를 지나 이름을 읽을 수 없게 된다
        %  (2026-09-21 W07_0·W07_1 의 Fr_psi_b — check_lines 의 (7) 이름표 위 선).
        %  This tag column **including its labels** is the right limit for chain lanes. A lane
        %  inside it runs its vertical segment over the tag name, making it unreadable
        %  (2026-09-21, Fr_psi_b of W07_0·W07_1 — item (7) line over label of check_lines).
        rr = get_param([m '/' c.src], 'Position');
        tt = name_box([m '/' c.src]);
        xl = rr(1);
        if ~any(isnan(tt)), xl = min(xl, tt(1)); end
        if ~isKey(edgeL, c.dst), edgeL(c.dst) = inf; end
        edgeL(c.dst) = min(edgeL(c.dst), xl);
    end
end

%  통로가 다 정해진 뒤에야 태그 열의 x 를 알 수 있다. 그래서 자리 옮기기는 뒤로 미룬다
%  The tag column's x is known only after all lanes are set, so moving is deferred
for c = keys(tagY)
    move_to(m, c{1}, colOf(srcOf(conn, c{1})), tagY(c{1}), 'left');
end

% --- 6) 선을 규칙대로 다시 긋는다 ---------------------------------------
% --- 6) redraw the lines by the rules --------------------------------------
%   포트 높이가 같으면 직선(꺾임 0회).
%   다르면 **두 블록 사이의 빈 곳**에서 한 번 내려가고 한 번 나온다(꺾임 2회).
%   출발·도착 블록의 테두리 위에서 꺾으려 하면 Simulink 가 그 구간을 거부하고
%   스스로 우회해 꺾임이 넷으로 늘어난다 — 2026-09-16 에 재현.
%   Same port height: straight line (0 bends).
%   Otherwise go down once and out once **in the empty space between the two blocks** (2 bends).
%   Bending on the border of the source or destination block makes Simulink reject that segment
%   and detour on its own, raising the bends to four — reproduced 2026-09-16.
%  통로를 **먼저 다 정하고** 그 다음에 긋는다. 간격을 좁히면 통로가 들어갈
%  띠가 얇아지는데, 하나씩 그으면 뒤 통로가 앞 통로와 같은 x 에 올라앉는 것을
%  알아차릴 길이 없다 (2026-10-01 — Pitch 를 340 에서 190 으로 줄였더니
%  `Guidance -> InnerLoop` 두 가닥이 x = 345 에서 포개졌다).
%  All lanes are **decided first**, then drawn. Narrower spacing thins the band for lanes,
%  and drawing one at a time gives no way to notice a later lane landing on the same x as an
%  earlier one (2026-10-01 — cutting Pitch from 340 to 190 piled the two
%  `Guidance -> InnerLoop` lines at x = 345).
%
%  규칙은 그대로다 — 통로 x 는 출발 포트 번호로 정한다. 한 신호의 여러 갈래는
%  같은 줄기를 공유해야 하므로(팬아웃은 그것이 옳다) **같은 출발 포트면 같은
%  통로**를 쓰고, 다른 포트끼리만 비킨다.
%  The rule is unchanged — lane x is set by the source port number. Branches of one signal must
%  share a trunk (correct for a fan-out), so **the same source port uses the same lane** and
%  only different ports step aside.
busyX = zeros(0, 3);          % [x, 위, 아래] — 이미 쓰고 있는 통로 / [x, top, bottom] — lanes in use
for i = 1:numel(busy)/3
    busyX(end+1,:) = busy(i,:); %#ok<AGROW>
end
laneOf = containers.Map('KeyType','char','ValueType','double');

for i = 1:numel(conn)
    c = conn(i);
    if is_type(m, c.dst, 'Goto'), continue, end
    a = port_xy(m, c.src, 'Outport', c.sp);
    b = port_xy(m, c.dst, 'Inport',  c.dp);
    if abs(a(2)-b(2)) < 0.5, continue, end

    key = sprintf('%s|%d', c.src, c.sp);
    if isKey(laneOf, key), continue, end      % 같은 출발 포트 — 줄기를 함께 쓴다 / same source port — shares the trunk

    xm = round((a(1) + b(1))/2) + TAG_LANE*(c.sp - 1);
    %  태그로 내려가는 통로들이 출발 포트 오른쪽에 줄지어 있다 (+18 부터
    %  TAG_LANE 씩). 사슬 통로를 그 안에 두면 두 신호가 같은 x 를 나눠 쓴다.
    %  Lanes going down to tags line up right of the source port (from +18, every TAG_LANE).
    %  A chain lane among them would make two signals share the same x.
    lo = a(1) + LANE_MIN;
    if isKey(cnt, c.src), lo = max(lo, a(1) + 24 + TAG_LANE*cnt(c.src)); end
    hi = b(1) - 15;
    %  받는 포트 왼쪽에 From 태그·상수 열이 서 있으면 그 이름표 앞에서 멈춘다
    %  If a column of From tags / constants stands left of the receiving port, stop before its labels
    if isKey(edgeL, c.dst), hi = min(hi, edgeL(c.dst) - 10); end
    if hi < a(1) + 15, hi = a(1) + 15; end
    if lo > hi, lo = max(a(1) + 15, hi); end
    xm = min(max(xm, lo), hi);

    %  같은 높이를 지나는 통로와 겹치면 비킨다. 오른쪽으로 먼저, 막히면 왼쪽으로.
    %  13 px 는 통로 반 칸이다 — 이만큼이면 두 선이 따로 보인다
    %  On a clash with a lane at the same height, step aside: right first, left if blocked.
    %  13 px is half a lane — enough for two lines to look separate
    yl = min(a(2), b(2));  yh = max(a(2), b(2));
    if taken(busyX, xm, yl, yh)
        found = false;
        for d = 13:13:260
            for s = [1 -1]
                x2 = xm + s*d;
                if x2 < lo || x2 > hi, continue, end
                if ~taken(busyX, x2, yl, yh), xm = x2; found = true; break, end
            end
            if found, break, end
        end
    end
    busyX(end+1,:) = [xm, yl, yh]; %#ok<AGROW>
    laneOf(key) = xm;
end

for i = 1:numel(conn)
    c = conn(i);
    a = port_xy(m, c.src, 'Outport', c.sp);
    b = port_xy(m, c.dst, 'Inport',  c.dp);
    if abs(a(2)-b(2)) < 0.5
        add_line(m, [a; b]);
    elseif is_type(m, c.dst, 'Goto') && isKey(lane, c.dst)
        %  태그는 블록 아래에 있다. 태그마다 제 통로로 내려가 옆으로 붙인다.
        %  통로를 공유하면 서로 다른 신호가 한 선으로 겹쳐 보인다
        %  Tags are below the block. Each tag goes down its own lane and attaches from the side.
        %  Shared lanes would make different signals look like one line
        add_line(m, [a; lane(c.dst) a(2); lane(c.dst) b(2); b]);
    else
        xm = laneOf(sprintf('%s|%d', c.src, c.sp));
        add_line(m, [a; xm a(2); xm b(2); b]);
    end
end

%  직접 연 모델은 **저장하고** 닫는다. 저장하지 않으면 여기서 한 배치가
%  전부 버려진다 (2026-09-16 에 한참 헤맴).
%  A model opened here is **saved** and closed. Without saving, the whole layout done here is
%  thrown away (cost a long search on 2026-09-16).
if opened, save_system(m); close_system(m, 0); end
end

% -------------------------------------------------------------------------
function tf = taken(busyX, x, yl, yh)
%TAKEN  그 x 통로를 같은 높이에서 이미 쓰고 있는가.
%   두 통로가 8 px 안에 들어오고 세로 구간이 겹치면 한 선으로 보인다.
%       Whether lane x is already used at the same height.
%   Two lanes within 8 px with overlapping vertical spans look like one line.
if isempty(busyX), tf = false; return, end
tf = any(abs(busyX(:,1) - x) < 8 & busyX(:,2) < yh - 1 & yl < busyX(:,3) - 1);
end

% -------------------------------------------------------------------------
function conn = grab(m)
%GRAB  최상위 신호선을 (출발 블록·포트, 도착 블록·포트) 목록으로 읽는다.
%      Reads the top-level signal lines as a list of (source block·port, destination block·port).
conn = struct('src',{},'sp',{},'dst',{},'dp',{});
L = find_system(m, 'FindAll','on', 'SearchDepth',1, 'Type','line');
for i = 1:numel(L)
    sp = get_param(L(i), 'SrcPortHandle');
    dp = get_param(L(i), 'DstPortHandle');
    if sp < 0, continue, end
    for d = dp(:)'
        if d < 0, continue, end
        conn(end+1) = struct( ...
            'src', get_param(get_param(sp,'Parent'), 'Name'), ...
            'sp',  get_param(sp, 'PortNumber'), ...
            'dst', get_param(get_param(d, 'Parent'), 'Name'), ...
            'dp',  get_param(d,  'PortNumber')); %#ok<AGROW>
    end
end

%  팬아웃은 부모선과 가지선이 같은 (출발, 도착) 을 두 번 보고할 수 있다.
%  그대로 두면 같은 자리에 선을 두 번 그어 길이 0 인 토막이 남는다.
%  In a fan-out, parent and branch lines can report the same (source, destination) twice.
%  Left as is, the line would be drawn twice in the same place, leaving zero-length segments.
if ~isempty(conn)
    key = arrayfun(@(c) sprintf('%s|%d|%s|%d', c.src, c.sp, c.dst, c.dp), ...
                   conn, 'UniformOutput', false);
    [~, ia] = unique(key, 'stable');
    conn = conn(ia);
end
end

% -------------------------------------------------------------------------
function s = srcOf(conn, dst)
%SRCOF  그 태그에 신호를 주는 블록의 이름.
%       Name of the block feeding that tag.
s = '';
for i = 1:numel(conn)
    if strcmp(conn(i).dst, dst), s = conn(i).src; return, end
end
end

% -------------------------------------------------------------------------
function [x, y] = free_slot(used, x, y)
%FREE_SLOT  같은 자리에 태그가 이미 있으면 40 px 씩 아래로 비켜 준다.
%   태그가 겹치면 어느 선이 어느 태그로 가는지 알 수 없다.
%           If a tag already occupies the spot, steps down by 40 px.
%   Overlapping tags make it impossible to tell which line goes to which tag.
while ~isempty(used) && any(abs(used(:,1)-x) < 80 & abs(used(:,2)-y) < 26)
    y = y + 30;
end
end

% -------------------------------------------------------------------------
function tf = is_type(m, name, t)
try, tf = strcmp(get_param([m '/' name], 'BlockType'), t); catch, tf = false; end
end

% -------------------------------------------------------------------------
function move_to(m, name, x, y, edge)
%MOVE_TO  블록의 지정한 테두리가 (x, y) 에 오도록 옮긴다. 크기는 그대로.
%         Moves a block so its given edge lands at (x, y). Size unchanged.
b = [m '/' name];
r = get_param(b, 'Position');
w = r(3)-r(1);  h = r(4)-r(2);
if strcmp(edge, 'left')
    set_param(b, 'Position', round([x, y-h/2, x+w, y+h/2]));
else
    set_param(b, 'Position', round([x-w, y-h/2, x, y+h/2]));
end
end
