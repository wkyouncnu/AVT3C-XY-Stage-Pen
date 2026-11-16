function blk = feed_from(sys, name, dst, dk, dy, ~)
%FEED_FROM  From 태그를 **도착 포트 바로 왼쪽**에 놓고 잇는다. 꺾임 0회, 관통 0.
%           Places a From tag **just left of the destination port** and connects it. 0 bends, 0 crossings.
%
%   feed_from(m, 'u', 'SumE', 2)              % 포트 옆 — 선은 수평 직선 / beside the port — straight horizontal line
%   feed_from(m, 'u', 'SumE', 2, 90)          % 옆자리가 차 있으면 90 px 아래로 / 90 px down if the side spot is taken
%
%   태그는 신호 바로 옆에 둔다 / a tag belongs beside its signal
%       2026-10-01 교수 지시 — "Goto 부분은 실제 신호에 최대한 가까이 모아 주고,
%       선들도 꺾이는 부분을 최소화해서 해 줄 것."
%       Instructor's instruction 2026-10-01 — "Gather the Goto parts as close as possible to
%       the actual signal, and keep the bends in the lines to a minimum."
%
%       도착 포트와 **같은 높이**, 한 칸(40 px) 왼쪽에 놓으면 잇는 선이 수평 한
%       토막이 되어 꺾임이 **0회**다. DY 는 그 자리가 이미 차 있을 때 비켜 갈
%       방향일 뿐이다.
%       Placed at **the same height** as the destination port, one step (40 px) to the left,
%       the connecting line is one horizontal segment with **0 bends**. DY is only the
%       direction to step aside when that spot is already taken.
%
%   왜 From 이 포트보다 왼쪽에 있어야 하는가 / why the From sits left of the port
%       From 의 **출력 포트는 오른쪽 테두리**에 있다. 도착 포트의 x 를 통로로 삼으면
%       선은 가로로 간 뒤 세로로 들어가며 꺾임이 한 번뿐이다. 그러려면 From 이
%       도착 포트보다 **왼쪽**에 있어야 한다. 이 함수가 그 자리를 계산한다.
%       A From's **output port is on its right border**. Using the destination port's x as
%       the lane, the line goes across then down with only one bend. For that the From must
%       be **left** of the destination port; this function computes that spot.
%
%       둥근 Sum 의 2번 입력은 원의 **아래쪽**에 있다. 되먹임이 아래에서 올라오는
%       MSS 데모의 모양이 이렇게 나온다 — 그런 포트는 같은 높이로 들어올 수 없으므로
%       가로로 간 뒤 세로로 올라간다 (꺾임 1회).
%       Input 2 of a round Sum is at the **bottom** of the circle. This gives the MSS-demo
%       look with feedback rising from below — such a port cannot be entered at the same
%       height, so the line goes across and then up (1 bend).
%
%   블록 이름 / the block name
%       'Fr_<태그>'. 같은 시스템에 이미 있으면 **번호만** 붙인다 (Fr_psi_2).
%       블록 이름을 섞지 않는다 — from_name.m 참조.
%       'Fr_<tag>'. If it already exists in the same system, **only a number** is added (Fr_psi_2).
%       Block names are not mixed in — see from_name.m.
%
%   SYS   부모 시스템 / parent system
%   NAME  태그 이름 (= 신호 이름) / tag name (= signal name)
%   DST   도착 블록 이름 / destination block name
%   DK    도착 블록의 입력 포트 번호 / input port number of the destination block
%   DY    옆자리가 찼을 때 비켜 갈 세로 거리 [px]. 생략하면 아래로 90
%         vertical distance [px] to step aside when the side spot is taken; default 90 down
%   여섯째 인자  옛 꼴의 꼬리표(SFX). **쓰이지 않는다** — 이름은 from_name 이 정한다.
%         옛 호출 feed_from(m,'noise','SumY',2,90,'1') 을 그대로 두어도 되게 받아만 둔다
%   sixth argument  the old-style suffix (SFX). **Not used** — the name is set by from_name.
%         It is only accepted so that old calls like feed_from(m,'noise','SumY',2,90,'1') still work

if nargin < 5 || isempty(dy), dy = 90; end

W = 60; H = 22; GAP = 40;

b  = port_xy(sys, dst, 'Inport', dk);
r  = get_param([sys '/' dst], 'Position');
sidePort = b(1) <= r(1) || b(1) >= r(3);

nm  = from_name(sys, name);
blk = [sys '/' nm];

%  1) 먼저 **포트와 같은 높이**를 본다. 포트가 블록 옆면에 있을 때만 가능하다 —
%     아래쪽 포트(둥근 Sum 의 2번)는 같은 높이로 들어올 수 없다.
%  1) First try **the port's own height**. Only possible when the port is on a side face —
%     a bottom port (input 2 of a round Sum) cannot be entered at the same height.
ok = false;  x2 = round(b(1) - GAP);  y = round(b(2));
if sidePort
    for g = GAP + [0 40 90 150 220]
        x2 = round(b(1) - g);
        if spot_free(sys, [x2-W, y-H/2, x2, y+H/2], nm, [x2 y; b], {dst})
            ok = true; break
        end
    end
end

%  2) 옆이 차 있으면 DY 쪽으로 비킨다. 아래위로만 훑으면 풀리지 않는 것이 있다 —
%     세로 통로의 x 는 From 의 **오른쪽 테두리**에 묶여 있어서, 그 x 위에 남의
%     이름표가 걸려 있으면 아무리 내려도 그대로 가로지른다 (2026-09-21 W04_3 의
%     Fr_Alloc_1_1 이 psi_ref_deg 의 이름을 그었다). 그래서 포트에서 얼마나
%     떨어질지(GAP)도 함께 훑어 통로를 옆으로 옮긴다.
%  2) If the side is taken, step aside towards DY. Scanning only up and down leaves some
%     cases unsolved — the x of the vertical lane is tied to the From's **right border**, so if
%     another block's label sits on that x, the line crosses it however far it goes down
%     (2026-09-21: Fr_Alloc_1_1 in W04_3 struck through the name psi_ref_deg). So the distance
%     from the port (GAP) is scanned as well, moving the lane sideways.
if ~ok
    sg = sign(dy);  if sg == 0, sg = 1; end
    for g = GAP + [0 50 100 160 220 290]
        x2 = round(b(1) - g);
        for t = 0:40
            y = round(b(2) + dy + sg*10*t);
            if sidePort, pts = [x2 y; x2 b(2); b]; else, pts = [x2 y; b(1) y; b]; end
            if spot_free(sys, [x2-W, y-H/2, x2, y+H/2], nm, pts, {dst})
                ok = true; break
            end
        end
        if ok, break, end
    end
    if ~ok                                     % 빈자리가 없다 (검사가 보고한다) / no free spot (the checks report it)
        x2 = round(b(1) - GAP);
        y  = round(b(2) + dy);
    end
end

add_block('simulink/Signal Routing/From', blk, ...
          'Position', [x2-W, y-H/2, x2, y+H/2], 'GotoTag', name);

%  포트가 블록의 **옆면**에 있으면 선은 가로로 들어가야 한다. 세로로 먼저 내려가고
%  마지막에 가로로 붙인다. 포트가 **아래쪽**에 있으면 반대다 — 가로로 간 뒤 세로로
%  올라간다. 이것을 구분하지 않으면 마지막 토막이 사선이 되어 포트에 비스듬히 붙는다.
%  If the port is on a block's **side face**, the line must enter horizontally: go down first
%  and attach horizontally at the end. If the port is at the **bottom**, the reverse — across,
%  then up. Without this distinction the last segment becomes a diagonal meeting the port at a slant.
if abs(y - b(2)) < 0.5
    pts = [x2 y; b];                           % 같은 높이 — 직선 한 토막 / same height — one straight segment
elseif sidePort
    pts = [x2 y; x2 b(2); b];
else
    pts = [x2 y; b(1) y; b];
end

%  포트로 먼저 잇는다. 점만 주면 끝점이 포트에 붙지 않고 매달릴 수 있다 —
%  도면은 이어져 보이는데 신호는 끊긴다 (drop_tag 의 같은 주석 참조).
%  Connect by port first. Given only points, the end may dangle instead of attaching —
%  the diagram looks joined but the signal is cut (see the same comment in drop_tag).
pf = get_param(blk, 'PortHandles');
pd = get_param([sys '/' dst], 'PortHandles');
draw_line(sys, pf.Outport(1), pd.Inport(dk), pts);
end
