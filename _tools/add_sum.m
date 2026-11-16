function blk = add_sum(sys, name, signs, centre)
%ADD_SUM  MSS 가 그리는 모양 그대로의 합산점 — 작은 원 하나.
%         A summing junction drawn the way MSS draws one: a small round circle.
%
%   blk = add_sum(sys, name, signs, centre)
%
%     sys      부모 시스템, 예를 들어 [mdl '/Controller']
%              the parent system, e.g. [mdl '/Controller']
%     signs    포트 순서대로의 부호. '+-', '++', '-+' 등
%              the signs, in port order
%     centre   원의 **중심** [x y]. 모서리가 아니다
%              the centre of the circle, not a corner
%
%   부호 목록은 언제나 막대로 시작한다 / the sign list always starts with a bar
%
%       교수 지시 2026-10-14 — "Simulink sum 블록은 항상 |+- 이런 부호 목록만
%       사용." 그래서 이 함수는 막대가 없는 목록을 받아도 앞에 막대를 붙인다.
%       '+-' 를 주어도 실제로는 '|+-' 가 들어간다.
%       Instructor's instruction 2026-10-14 — "Always use a sign list like |+- for
%       Simulink Sum blocks." So this function prepends a bar even when given a list
%       without one. Giving '+-' actually puts '|+-'.
%
%           '|+-'  입력1 (왼쪽)   입력2 (아래)   출력 (오른쪽)
%           '|++'  입력1 (왼쪽)   입력2 (아래)   출력 (오른쪽)
%                  input1 (left)  input2 (bottom) output (right)
%
%       막대는 포트가 아니라 **포트 자리 하나를 비우는 띄우개**다. 원의 위
%       자리를 비우면 첫 입력이 왼쪽 가장자리에 남고 둘째 입력만 아래로
%       내려간다. 그림에서는 이렇게 읽힌다.
%       The bar is not a port but a **spacer that leaves one port slot empty**. With the
%       top slot of the circle empty, the first input stays on the left edge and only
%       the second input moves to the bottom. In the diagram it reads like this.
%
%           지령 ------->(+)-----> 제어기 ...
%                         ^
%                         | (-)
%                      되먹임
%           (지령 = command, 제어기 = controller, 되먹임 = feedback)
%
%       신호가 **왼쪽에서 들어와 오른쪽으로 나가는 한 줄**이 되고, 되먹임만
%       아래에서 올라온다. 되먹임 제어 그림의 표준 모양이며, 지령선·출력선이
%       모두 수평이라 블록을 포트 높이에 맞추어 놓으면 꺾임이 없는 직선이 된다.
%       The signal becomes **one row entering on the left and leaving on the right**, and
%       only the feedback comes up from below. This is the standard shape of a feedback
%       control diagram; command and output lines are both horizontal, so placing blocks
%       at port height gives straight lines with no bends.
%
%       2026-10-02 에는 막대 없는 '+-' (지령 위·되먹임 아래)를 골랐고 이 머리글도
%       그렇게 적혀 있었다. 2026-10-14 지시로 바뀌었다 — 위에서 내려오는 지령선은
%       신호 흐름을 끊고, 합산점마다 긴 세로줄이 생겨 선이 서로 겹치기 때문이다.
%       On 2026-10-02 the bar-less '+-' (command on top, feedback at bottom) was chosen
%       and this header said so. It changed with the 2026-10-14 instruction — a command
%       line coming down from above breaks the signal flow, and each junction gets a long
%       vertical run where lines overlap.
%
%   왜 필요한가 / WHY THIS EXISTS
%
%   Simulink 기본 Sum 블록은 부호가 안에 인쇄된 25 x 40 사각형이다. 페이지 폭으로
%   읽어야 하는 도면에서는 덩어리다 — 주변 게인 블록보다 크고, 신호 사슬의 줄을
%   끊으며, 읽는 사람이 가장 먼저 찾는 오차 합산점이 루프에서 가장 무거워 보인다.
%
%   Simulink's default Sum block is a rectangle 25 x 40 with the signs printed
%   inside it. On a diagram that has to be read at page width it is a slab: it
%   is larger than the gain blocks around it, it breaks the line of the signal
%   chain, and the error junction - the one block a reader looks for first -
%   ends up the most visually heavy thing in the loop.
%
%   MSS 는 어디서나 20 x 20 원을 쓴다. demoOtterUSVHeadingControl.slx 의 Sum 블록
%   여덟 개가 모두 IconShape 'round', 20 x 20 임을 확인했다. 이 함수는 그 모양과
%   크기를 따른다.
%   MSS uses a circle 20 x 20 throughout. Checked against
%   Tools/MSS/SIMULINK/mssSimulinkDemos/demoOtterUSVHeadingControl.slx, whose
%   eight Sum blocks are all IconShape 'round' at 20 x 20. This function keeps
%   that shape and size.
%
%   모서리가 아니라 중심을 인자로 받는 것은 합산점이 자기가 끊는 신호와 줄을 맞춰
%   놓이기 때문이다. 중심을 맞추면 선이 곧고, 모서리를 맞추면 그렇지 않다.
%   The centre is the argument rather than a corner because a summing junction
%   is placed to line up with the signal it interrupts. Aligning centres keeps
%   the wire straight; aligning corners does not.
%
%   옛 차수의 빌더 / older builders
%       부호만 '+-' 로 적어 둔 빌더도 그대로 돈다 (막대는 여기서 붙는다). 다만
%       포트가 위에서 왼쪽으로 옮겨 가므로, 그 차수를 **다시 빌드할 때** 합산점
%       주변 블록 높이를 한 번 맞추어 주어야 check_lines 가 0 으로 남는다.
%       Builders that wrote signs as plain '+-' still run (the bar is added here). But
%       the port moves from the top to the left, so when that week is **rebuilt** the
%       heights of blocks around the junction must be realigned once for check_lines to stay 0.

if nargin < 4, error('add_sum:args', 'add_sum(sys, name, signs, centre)'); end

signs = strrep(signs, '|', '');            % 들어온 막대는 떼고 / strip incoming bars
signs = ['|' signs];                       % 맨 앞에 하나만 다시 붙인다 / put exactly one back in front
R     = 10;                                % MSS 20 x 20 의 절반 / half of the MSS 20 x 20

blk = [sys '/' name];
add_block('simulink/Math Operations/Sum', blk, ...
          'IconShape', 'round', ...
          'Inputs',    signs, ...
          'Position',  [centre(1)-R, centre(2)-R, centre(1)+R, centre(2)+R]);
end
