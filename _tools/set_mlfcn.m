function set_mlfcn(blk, lines, outName, outSize, sampleTime)
%SET_MLFCN  MATLAB Function 블록에 코드를 넣고, 출력의 크기를 명시한다.
%           Put a script into a MATLAB Function block, and size its output.
%
%   set_mlfcn(blk, lines)
%   set_mlfcn(blk, lines, outName, outSize)
%   set_mlfcn(blk, lines, outName, outSize, 'h')
%
%     blk         이미 존재하는 MATLAB Function 블록의 전체 경로
%                 the full path of an existing MATLAB Function block
%     lines       소스 줄들의 셀 배열. 한 원소가 한 줄
%                 a cell array of source lines, one per element
%     outName     크기를 명시할 출력 변수의 이름, 예를 들어 'xdot'
%                 the output variable to give an explicit size, e.g. 'xdot'
%
%   왜 크기를 명시하는가 / why the size is stated
%       Simulink 는 출력 크기를 추론하려 하고, 추론에 실패하면 컴파일 시점에야
%       알려 준다. 그때 나오는 메시지는 블록 안의 어느 줄이 원인인지 말해 주지
%       않는다. 크기를 미리 못박아 두면 그 부류의 오류가 아예 생기지 않는다.
%
%       Simulink tries to infer the output size and reports failure only at
%       compile time, in a message that does not say which line inside the
%       block caused it. Fixing the size in advance removes that class of
%       error altogether.
%     outSize     크기 문자열, 예: '[12 1]'
%                 size string, e.g. '[12 1]'
%     sampleTime  이산 주기, 예: 'h'. 스크립트가 PERSISTENT 변수를 가지면 반드시 준다
%                 discrete rate, e.g. 'h'. Required whenever the script keeps
%                 a PERSISTENT variable
%
%   WHY A BLOCK WITH MEMORY MUST DECLARE ITS RATE / 기억을 가진 블록이 주기를 밝혀야 하는 이유
%
%   주기를 물려받게 두면 연속 운동모델에 이어진 MATLAB Function 블록은 연속 샘플
%   타임을 받고, Simulink 는 다음과 같이 스크립트를 거부한다.
%   Left to inherit, a MATLAB Function block wired to a continuous plant picks
%   up a continuous sample time, and Simulink then refuses the script:
%
%       ... uses invalid syntax when the block specifies or inherits a
%           continuous sample time ... writing to initialized persistent
%           variables ...
%
%   거부가 옳다. 솔버가 한 스텝을 밟았다 버리고 다시 밟을 수 있으면 persistent 변수는
%   뜻이 없다 — 시간 한 스텝에 상태가 여러 번 나아간다. 'h' 를 주면 블록은 스텝마다
%   한 번 갱신되는 이산 알고리즘이 되며, 안의 오일러 갱신이 이미 그것을 가정하고 있었다.
%   The refusal is right. A persistent variable has no meaning when the solver
%   may take a step, reject it and take it again: the state would advance more
%   than once for one step of time. Saying 'h' makes the block a discrete
%   algorithm updated once per step, which is what the Euler update inside it
%   already assumed.
%
%   주기는 블록 파라미터가 아니라 Stateflow 차트의 속성이므로
%   set_param(blk,'SampleTime',...) 는 "SubSystem block does not have a parameter
%   named 'SampleTime'" 으로 실패한다.
%   The rate is a property of the Stateflow chart, not a block parameter, so
%   set_param(blk,'SampleTime',...) fails with "SubSystem block does not have
%   a parameter named 'SampleTime'".
%
%   블록의 스크립트도 블록 파라미터가 아니다. Stateflow 객체 트리에 있으므로
%   set_param 이 아니라 sfroot() 를 거친다.
%   The block's script is not a block parameter. It lives in the Stateflow
%   object tree, which is why this goes through sfroot() rather than set_param.
%
%   블록이 extrinsic 함수를 부르면 출력 크기를 반드시 밝혀야 한다. Simulink 는 그런
%   호출을 컴파일하지 않으므로 돌아오는 값의 폭을 추론할 수 없다.
%   The explicit output size is needed whenever the block calls an extrinsic
%   function. Simulink does not compile such a call, so it cannot infer the
%   width of what comes back, and the width has to be declared.

n = sfroot().find('-isa','Stateflow.EMChart','-and','Path',blk);
if isempty(n)
    error('set_mlfcn:notFound','No MATLAB Function block at %s', blk);
end
fpos = get_param(blk, 'Position');
n.Script = strjoin(lines, newline);
%  포트가 생기며 블록이 멋대로 자라고, 그 포트 위치는 저장했다 다시 열면 달라진다.
%  다시 놓아 두면 지금 읽는 포트 위치가 저장 뒤와 같다 (2026-09-19 사선 검사).
%  Creating ports makes the block grow on its own, and those port positions change after
%  save and reopen. Restoring the position makes the ports read now equal those after
%  saving (2026-09-19 diagonal check).
set_param(blk, 'Position', fpos);

if nargin >= 4 && ~isempty(outName)
    d = n.find('-isa','Stateflow.Data','-and','Name',outName);
    d.DataType = 'double';
    d.Props.Array.Size = outSize;
end

if nargin >= 5 && ~isempty(sampleTime)
    n.ChartUpdate = 'DISCRETE';
    n.SampleTime  = sampleTime;
end
end
