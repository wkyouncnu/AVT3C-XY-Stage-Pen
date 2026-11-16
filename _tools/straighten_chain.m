function n = straighten_chain(mdl, chain, verbose)
%STRAIGHTEN_CHAIN  사슬의 이웃 블록을 위아래로 옮겨, 바로 잇는 선을 직선으로 만든다.
%                  Moves neighbouring chain blocks up or down so the lines directly joining them are straight.
%
%   n = straighten_chain('W06_1_point', {'Guidance','OuterLoop','InnerLoop','MotionModel'})
%   straighten_chain(mdl, chain, true)        % 무엇을 몇 px 옮겼는지 말한다 / reports what moved by how many px
%
%   왜 이 도구가 필요한가 / why this exists
%       `lay_chain` 은 사슬 블록을 열마다 세우고 **가운데 높이**를 맞춘다. 포트
%       수가 다르면 가운데를 맞춘 것으로는 포트가 맞지 않는다. 포트 간격이
%       50 px 로 같아도, 출력 4개인 블록과 입력 6개인 블록을 가운데로 맞추면
%       첫 포트끼리 50 px 어긋난다.
%       `lay_chain` stands chain blocks in columns and aligns their **centre heights**. With
%       different port counts, aligned centres do not align ports. Even with the same 50 px
%       pitch, centring a block with 4 outputs on one with 6 inputs leaves the first ports 50 px apart.
%
%           Guidance 출력  195 245 295 345        (4개, 가운데 270) / outputs (4, centre 270)
%           OuterLoop 입력 145 195 245 295 345 395 (6개, 가운데 270) / inputs (6, centre 270)
%
%       그러면 x_ref 는 195 에서 나와 145 로 **올라가고**, y_ref 는 245 에서
%       나와 195 로 올라간다. 둘 다 두 번 꺾이고, 게다가 x_ref 가 떠나는 통로
%       (y = 195) 와 y_ref 가 닿는 통로가 같은 높이라 100 px 를 함께 쓴다 —
%       `check_lines` 의 (1) 겹침이다 (2026-10-14 W06_1_point 에서 2건).
%       Then x_ref leaves at 195 and **climbs** to 145, and y_ref leaves at 245 and climbs to
%       195. Both bend twice, and the lane x_ref leaves on (y = 195) is at the same height as
%       the lane y_ref arrives on, so they share 100 px — overlap item (1) of `check_lines`
%       (2 cases in W06_1_point on 2026-10-14).
%
%       포트 간격이 같으므로 **첫 짝만 맞추면 나머지도 저절로 맞는다.** 그래서
%       첫 짝의 높이 차이만큼 위쪽 블록을 통째로 옮긴다.
%       Since the pitch is equal, **aligning the first pair aligns the rest.** So the upstream
%       block is moved as a whole by the height difference of the first pair.
%
%   어떻게 / how
%       사슬의 **뒤에서 앞으로** 돈다. 뒷블록을 먼저 제자리에 앉히고 앞블록을
%       그것에 맞추어야, 앞에서부터 돌 때처럼 맞춘 짝이 다시 어긋나지 않는다.
%       옮기는 것은 위쪽(앞) 블록이다 — 뒷블록은 이미 맞춰 둔 자리다.
%       It walks the chain **from back to front**. Seating the downstream block first and
%       fitting the upstream one to it keeps aligned pairs from coming apart again, as they
%       would when walking front to back. The upstream (front) block is the one moved — the
%       downstream block is already in place.
%
%       짝은 **직접 이은 선 가운데 도착 포트 번호가 가장 작은 것** 하나를 쓴다.
%       태그(Goto/From)로 건너가는 신호는 선이 아니므로 애초에 끼지 않는다.
%       The pair used is **the direct line with the smallest destination port number**.
%       Signals passed by tags (Goto/From) are not lines and so never take part.
%
%   무엇을 건드리지 않는가 / what does not change
%       연결·게인·이름은 그대로다. 블록의 **y 자리**만 바뀐다. 가로(x) 는 한 px
%       도 움직이지 않으므로 `lay_chain` 이 세운 단계 순서가 그대로 남는다.
%       Connections, gains and names stay. Only the block's **y position** changes. x does not
%       move by a single px, so the stage order set by `lay_chain` remains.
%
%   돌려주는 값 n 은 자리를 옮긴 블록의 수다.
%   The return value n is the number of blocks moved.

if nargin < 3, verbose = false; end

opened = false;
if ~bdIsLoaded(mdl), load_system(mdl); opened = true; end
n = 0;

for k = numel(chain)-1 : -1 : 1
    up = chain{k};  dn = chain{k+1};
    if isempty(getSimulinkBlockHandle([mdl '/' up]) > 0), continue, end
    if getSimulinkBlockHandle([mdl '/' up]) < 0 || getSimulinkBlockHandle([mdl '/' dn]) < 0
        continue
    end

    [sp, dp] = first_pair(mdl, up, dn);
    if isempty(sp), continue, end

    y0 = port_xy(mdl, up, 'Outport', sp);
    align_to(mdl, up, 'Outport', port_xy(mdl, dn, 'Inport', dp), sp);
    y1 = port_xy(mdl, up, 'Outport', sp);

    if y1(2) ~= y0(2)
        n = n + 1;
        if verbose
            fprintf('  [straighten] %s 를 %+d px — 출력 %d 을 %s 입력 %d 에 맞춤\n', ...
                    up, y1(2)-y0(2), sp, dn, dp);
        end
    end
end

if opened, close_system(mdl, 0); end
end

% ---------------------------------------------------------------------
function [sp, dp] = first_pair(mdl, up, dn)
%FIRST_PAIR  up 에서 dn 으로 바로 이은 선 가운데 도착 포트가 가장 앞인 짝.
%           Among lines directly from up to dn, the pair with the lowest destination port.
sp = [];  dp = [];
pc = get_param([mdl '/' up], 'PortConnectivity');
best = inf;
for i = 1:numel(pc)
    if isempty(pc(i).DstBlock), continue, end        % 출력 포트만 DstBlock 을 갖는다 / only outputs have DstBlock
    for j = 1:numel(pc(i).DstBlock)
        h = pc(i).DstBlock(j);
        if ~strcmp(get_param(h, 'Name'), dn), continue, end
        dpj = pc(i).DstPort(j) + 1;                  % DstPort 는 0 부터 센다 / DstPort counts from 0
        if dpj < best
            best = dpj;
            sp = str2double(pc(i).Type);             % 출력 포트는 Type 이 번호다 / for outputs Type is the number
            dp = dpj;
        end
    end
end
end
