function sub = add_subsys(mdl, name, pos, ins, outs, colour)
%ADD_SUBSYS  이름 붙은 포트를 가진 빈 서브시스템. 안을 채우기만 하면 된다.
%            An empty subsystem with named ports, ready to be filled.
%
%   sub = add_subsys(mdl, name, pos, ins, outs, colour)
%
%     mdl     모델 이름 / the model name
%     name    서브시스템 이름, 예를 들어 'Heading autopilot'
%             the subsystem name, e.g. 'Heading autopilot'
%     pos     [x1 y1 x2 y2], 보통 gnc_chain 이 준다
%             [x1 y1 x2 y2], normally taken from gnc_chain
%     ins     입력 포트 이름의 셀 배열, 예: {'psi_d','x'}
%             cell array of inport names,  e.g. {'psi_d','x'}
%     outs    출력 포트 이름의 셀 배열, 예: {'tau_N'}
%             cell array of outport names, e.g. {'tau_N'}
%     colour  선택 사항, BackgroundColor 문자열
%             optional BackgroundColor string
%
%   서브시스템의 전체 경로를 돌려주므로 호출자가 그 안에 블록을 계속 넣을 수 있다.
%   Returns the full path of the subsystem, so the caller can go on adding
%   blocks inside it.
%
%   WHY EVERY STAGE IS A SUBSYSTEM / 왜 단계마다 서브시스템인가
%
%   이 과목 모델의 최상위는 신호 사슬만 보여 준다 — 한 줄로 늘어선 블록 여섯, 짝마다
%   선 하나, 되먹임 선 하나. 단계를 움직이는 것은 모두 그 단계 안에 있다. 제어기가
%   무엇을 하는지 알고 싶은 사람은 제어기를 연다.
%   The top level of a model in this course shows the signal chain and nothing
%   else: six blocks in a row, one line between each pair, and one feedback
%   line. Everything that makes a stage work lives inside that stage. A reader
%   who wants to know what the controller does opens the controller.
%
%   반대로 게인·합산·적분기를 모두 한 캔버스에 두면 한 쪽에 읽을 수도 문서로 내보낼
%   수도 없는 도면이 된다. 고치기도 더 어렵다 — 블록 하나를 옮겨도 다른 것은 따라
%   움직이지 않아 고칠 때마다 배치가 무너진다.
%   The alternative — every gain, sum and integrator on one canvas — produces a
%   diagram that cannot be read on a page and cannot be exported into a
%   document. It is also harder to change, because moving one block moves
%   nothing else and the layout decays with every edit.
%
%   PORT NAMES ARE THE INTERFACE / 포트 이름이 인터페이스다
%
%   포트에는 이름을 붙이며 In1, Out1 로 두지 않는다. 그 이름이 최상위 도면에서 선 옆에
%   보이므로, `psi_d` 와 `tau_N` 이 주석 하나 없이 모델을 설명한다.
%   Ports are named, never left as In1 and Out1. The name is what the top-level
%   diagram shows next to the line, so `psi_d` and `tau_N` document the model
%   without a single annotation.

if nargin < 6, colour = ''; end
if nargin < 5 || isempty(outs), outs = {}; end
if nargin < 4 || isempty(ins),  ins  = {}; end

sub = [mdl '/' name];
add_block('built-in/Subsystem', sub, 'Position', pos);
clear_subsystem(sub);

%  포트는 서브시스템 캔버스의 왼쪽·오른쪽 가장자리를 따라 일정한 간격으로 놓인다.
%  그래서 입력 셋인 서브시스템은 늘 같은 모양이다.
%  Ports are spread down the left and right edges of the subsystem canvas at a
%  fixed pitch, so a subsystem with three inports always looks like one.
PITCH = 60;  Y0 = 80;
for i = 1:numel(ins)
    add_block('simulink/Sources/In1', [sub '/' ins{i}], ...
              'Port', num2str(i), ...
              'Position', [60 Y0+PITCH*(i-1) 90 Y0+PITCH*(i-1)+20]);
end
for i = 1:numel(outs)
    add_block('simulink/Sinks/Out1', [sub '/' outs{i}], ...
              'Port', num2str(i), ...
              'Position', [700 Y0+PITCH*(i-1) 730 Y0+PITCH*(i-1)+20]);
end

if ~isempty(colour), set_param(sub, 'BackgroundColor', colour); end
end

% -------------------------------------------------------------------------
function clear_subsystem(sub)
b = find_system(sub, 'SearchDepth',1, 'LookUnderMasks','all', 'Type','Block');
for i = 1:numel(b)
    if ~strcmp(b{i}, sub), delete_block(b{i}); end
end
end
