function n = square_lines(mdl)
%SQUARE_LINES  사선 토막을 없앤다. 꺾임 수는 늘리지 않는다.
%   Removes diagonal segments without increasing the number of turns.
%
%   n = square_lines('W07_0_offline')      % 최상위와 모든 서브시스템 / top level and all subsystems
%
%   왜 생기나 / where the diagonals come from
%       선을 긋고 **나서** 블록 크기가 바뀌면 (mss_style 이 Sum 을 20x20 원으로,
%       MATLAB Function 이 포트를 새로 만들며) Simulink 는 선의 **끝점만** 새 포트로
%       당기고 그 이웃 점은 그대로 둔다. 그래서 끝 토막이 2~7 px 기운 사선이 된다.
%       포트 높이가 몇 px 어긋난 두 블록을 곧게 이은 선도 사선이다.
%       When block sizes change after lines are drawn (mss_style making Sum a 20x20 circle,
%       a MATLAB Function creating new ports), Simulink pulls only the line end point to the
%       new port and leaves the neighbouring point. The end segment becomes a diagonal tilted
%       by 2-7 px. A straight line between two blocks whose port heights differ by a few px
%       is also diagonal.
%
%   어떻게 고치나 / how, in order of preference
%     1) 꺾인 선 — 사선 토막의 **안쪽 점**을 이웃 토막의 방향을 따라 옮겨 직각으로
%        만든다. 점의 수가 그대로이므로 꺾임도 그대로다.
%        Bent line: move the inner point of the diagonal segment along the neighbouring
%        segment's direction to make it square. The number of points and turns stays the same.
%     2) 곧은 선(두 점) — 배치로 푼다. 한쪽이 **잎 블록**(선이 이것 하나뿐인
%        From·Constant·Inport·Clock / Goto·Outport·Terminator·Display·To Workspace)
%        이면 그 블록을 위아래로 옮겨 포트 높이를 맞춘다. 둘 다 아니면 남겨 두고
%        check_lines 가 보고하게 한다 — 빌더의 배치를 고쳐야 하는 경우다.
%        Straight line (two points): fixed by layout. If one end is a leaf block (a
%        From/Constant/Inport/Clock or Goto/Outport/Terminator/Display/To Workspace with only
%        this line), that block is moved up or down to match the port height. Otherwise the
%        line is left for check_lines to report; the builder's layout must be fixed.
%     3) 몇 px 짜리 계단 (가로-세로-가로, 세로 15 px 이하) — 사선과 원인이 같다.
%        2) 와 같이 잎 블록을 옮겨 곧은 한 토막으로 만든다 (꺾임 2 -> 0).
%        Small steps of a few px (horizontal-vertical-horizontal, vertical <= 15 px) have the
%        same cause. As in 2), the leaf block is moved to make one straight segment (turns 2 -> 0).
%
%   돌려주는 값은 고친 선의 수다.
%   The return value is the number of lines fixed.

sys = [{mdl}; find_system(mdl, 'LookUnderMasks','all', 'BlockType','SubSystem')];
n = 0;
for s = 1:numel(sys)
    %  Stateflow 차트·MATLAB Function 의 안은 Simulink 선이 아니다
    %  The inside of Stateflow charts and MATLAB Functions is not Simulink lines
    try
        if ~strcmp(get_param(sys{s},'Type'),'block_diagram') && ...
           ~strcmp(get_param(sys{s},'SFBlockType'),'NONE'), continue, end
    catch
    end
    L = find_system(sys{s}, 'FindAll','on', 'SearchDepth',1, 'Type','line');
    for i = 1:numel(L)
        try
            n = n + fix_one(sys{s}, L(i));
        catch
        end
    end
end
end

% -------------------------------------------------------------------------
function k = fix_one(sys, h)
k = 0;
p = get_param(h, 'Points');
if ~any(is_diag(p))
    %  사선은 아니지만 몇 px 짜리 계단(가로-세로-가로, 세로 15 px 이하)도 같은 원인이다.
    %  잎 블록을 옮겨 곧은 한 토막으로 만든다 (꺾임 2 -> 0)
    %  Not diagonal, but small steps (horizontal-vertical-horizontal, vertical <= 15 px) have
    %  the same cause. Move the leaf block to make one straight segment (turns 2 -> 0)
    if size(p,1) == 4 && is_h(p(1,:),p(2,:)) && is_v(p(2,:),p(3,:)) && is_h(p(3,:),p(4,:)) ...
            && abs(p(3,2)-p(2,2)) <= 15 && p(4,1) > p(1,1)
        k = fix_straight(sys, h, p([1 4],:));
    end
    return
end

if size(p,1) == 2
    k = fix_straight(sys, h, p);
    return
end

%  첫 토막이 사선이고 출발 쪽이 **잎 블록**이면 그 블록을 옮겨 곧게 만든다.
%  아래의 "안쪽 점을 옮기는" 방식은 이 경우에 듣지 않는다 — 옮길 수 있는 점이
%  도착 포트에 붙은 끝점뿐이라, 고쳐 놓아도 Simulink 가 포트로 되snap 하면서
%  사선이 되살아난다.
%
%  2026-10-02 SB11_enabled_done/EnSub 의 In1 -> SumA 가 그랬다. 합산점의 두
%  입력이 왼쪽 가장자리로 올라오면서(add_sum) 포트가 원 바깥 ±(h/2+5) 에 서고,
%  그 높이에 맞춰 놓이지 않은 Inport 와 10 px 어긋났다.
%  If the first segment is diagonal and the source is a leaf block, move that block to make
%  it straight. The "move the inner point" method below does not work here: the only movable
%  point is the end point attached to the destination port, and even when fixed, Simulink
%  snaps it back to the port and the diagonal returns.
%
%  This happened on 2026-10-02 with In1 -> SumA in SB11_enabled_done/EnSub. The two inputs of
%  the junction moved up to the left edge (add_sum), placing the ports at ±(h/2+5) outside
%  the circle, 10 px off from an Inport not placed at that height.
if is_diag(p(1:2,:))
    k = fix_straight(sys, h, p(1:2,:));
    if k > 0, return, end
end

q = p;
for pass = 1:3
    d = find(is_diag(q), 1);
    if isempty(d), break, end
    m = size(q,1);
    %  사선 토막 d -> d+1. 끝점(포트)은 움직일 수 없으므로 안쪽 점을 옮긴다
    %  Diagonal segment d -> d+1. End points (ports) cannot move, so move the inner point
    if d > 1
        %  d 를 옮긴다. 앞 토막 d-1 -> d 의 방향을 따라 움직여야 그 토막이 곧게 남는다
        %  Move d along the direction of the previous segment d-1 -> d so that it stays straight
        if is_h(q(d-1,:), q(d,:)), q(d,1) = q(d+1,1);       % 앞이 가로 -> 이번은 세로 / previous horizontal -> this vertical
        elseif is_v(q(d-1,:), q(d,:)), q(d,2) = q(d+1,2);   % 앞이 세로 -> 이번은 가로 / previous vertical -> this horizontal
        else, break
        end
    elseif d + 1 < m
        %  첫 토막이 사선. d+1 을 뒤 토막 방향을 따라 옮긴다
        %  The first segment is diagonal; move d+1 along the direction of the next segment
        if is_h(q(d+1,:), q(d+2,:)), q(d+1,1) = q(d,1);
        elseif is_v(q(d+1,:), q(d+2,:)), q(d+1,2) = q(d,2);
        else, break
        end
    else
        break
    end
end
if isequal(q, p) || any(is_diag(q)), return, end
%  꺾임을 늘리지 않았는지, 길이 0 토막이 생겼으면 지운다
%  Make sure turns were not added; remove zero-length segments if any were created
keep = [true; any(abs(diff(q,1,1)) > 0.5, 2)];
q = q(keep,:);
set_param(h, 'Points', q);
if ~any(is_diag(get_param(h,'Points'))), k = 1; end
end

% -------------------------------------------------------------------------
function k = fix_straight(sys, h, p)
k = 0;
sb = get_param(h, 'SrcBlockHandle');
db = get_param(h, 'DstBlockHandle');
if isempty(db) || numel(db) ~= 1 || db <= 0 || sb <= 0, return, end
dy = round(p(2,2) - p(1,2));
dx = round(p(2,1) - p(1,1));
if abs(dx) < abs(dy), return, end              % 가로선이어야 할 것만 다룬다 / only lines meant to be horizontal
%  옮긴 자리가 다른 블록과 겹치면 옮기지 않는다
%  Do not move if the new position overlaps another block
if is_leaf(sb) && can_move(sys, sb, dy)
    move(sb, dy);
elseif is_leaf(db) && can_move(sys, db, -dy)
    move(db, -dy);
else
    return
end
%  옮긴 뒤 두 포트가 같은 높이면 한 토막으로 다시 놓는다 (남은 계단·사선 점을 지운다)
%  If both ports are at the same height after the move, lay one segment (removes leftover step/diagonal points)
sp = get_param(h, 'SrcPortHandle');  dp = get_param(h, 'DstPortHandle');
a = get_param(sp, 'Position');  b = get_param(dp, 'Position');
if abs(a(2)-b(2)) < 0.5, set_param(h, 'Points', [a; b]); end
if ~any(is_diag(get_param(h,'Points'))), k = 1; end
end

function tf = is_leaf(b)
%  선이 딱 하나만 붙은 블록. 옮겨도 다른 선을 건드리지 않는다
%  A block with exactly one line attached; moving it does not disturb other lines
tf = false;
t = get_param(b, 'BlockType');
if ~any(strcmp(t, {'From','Constant','Inport','Clock','DigitalClock', ...
                   'Goto','Outport','Terminator','Display','ToWorkspace','Ground'}))
    return
end
lh = get_param(b, 'LineHandles');
c = [lh.Inport(:); lh.Outport(:)];
tf = sum(c > 0) == 1;
end

function tf = can_move(sys, b, dy)
%  옮긴 사각형(이름 자리 10 px 포함)이 같은 층의 다른 블록과 겹치지 않는가
%  Does the moved rectangle (including 10 px for the name) avoid other blocks on the same level?
r = get_param(b, 'Position') + [0 dy 0 dy] + [0 -2 0 12];
o = find_system(sys, 'SearchDepth',1, 'Type','Block');
tf = true;
for i = 1:numel(o)
    h = get_param(o{i}, 'Handle');
    if h == b || strcmp(o{i}, sys), continue, end
    q = get_param(h, 'Position');
    if min(r(3),q(3)) - max(r(1),q(1)) > 0 && min(r(4),q(4)) - max(r(2),q(2)) > 0
        %  이미 겹쳐 있던 것은 탓하지 않는다 / overlaps that existed before are not blamed
        p = get_param(b, 'Position') + [0 -2 0 12];
        if min(p(3),q(3)) - max(p(1),q(1)) > 0 && min(p(4),q(4)) - max(p(2),q(2)) > 0, continue, end
        tf = false;  return
    end
end
end

function move(b, dy)
set_param(b, 'Position', get_param(b,'Position') + [0 dy 0 dy]);
end

function tf = is_diag(p)
dd = abs(diff(p,1,1));
tf = dd(:,1) > 0.5 & dd(:,2) > 0.5;
end
function tf = is_h(a, b), tf = abs(a(2)-b(2)) <= 0.5; end
function tf = is_v(a, b), tf = abs(a(1)-b(1)) <= 0.5; end
