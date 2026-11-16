function P = gnc_chain(stages, varargin)
%GNC_CHAIN  GNC 신호 사슬의 각 단계가 놓일 표준 위치를 돌려준다.
%           Standard block positions for the stages of the GNC signal chain.
%
%   P = gnc_chain({'command','controller','allocation','plant','measurement'})
%   P.controller                        -> [x1 y1 x2 y2]
%   P = gnc_chain(..., 'Height', struct('controller',150))
%
%   왜 좌표를 손으로 쓰지 않는가 / why coordinates are not typed by hand
%       매주 같은 자리에서 같은 것을 찾을 수 있어야 한다. 좌표를 빌더마다 손으로
%       적으면 주차마다 조금씩 달라지고, 그 차이는 아무 의미도 없으면서 읽는
%       사람의 눈을 매번 다시 적응시킨다.
%
%       The same thing must be in the same place every week. Coordinates typed
%       into each builder drift from week to week, and the differences carry no
%       meaning while forcing the reader's eye to re-adjust each time.
%
%   사슬 / the chain
%       이 강의의 모든 모델은 MSS 데모 모델
%       (Tools/MSS/SIMULINK/mssSimulinkDemos/demoOtterUSVHeadingControl.slx)
%       과 같은 순서로 왼쪽에서 오른쪽으로 놓인다.
%       Every model in this course is laid out left to right in the same order,
%       the order used by the MSS demonstration models:
%
%     command --> reference --> controller --> allocation --> plant --> measurement
%      what is     what is       what force     which          how the   what is
%      asked for   achievable    is needed      thrusters      vessel    recorded
%                  and how fast                 produce it     responds
%      무엇을      실현 가능한가  어떤 힘이      어느 추진기가  배가 어떻게  무엇을
%      요구하는가  얼마나 빨리   필요한가       내는가         반응하는가   기록하는가
%
%   어떤 단계가 필요 없는 주차는 그 단계를 목록에서 빼기만 하면 되고, 남은 단계가
%   붙어서 놓인다. 계약은 좌표가 아니라 **순서**다. 모델 하나를 본 독자는 다른
%   모든 모델에서 어디를 보아야 할지 안다.
%   A week that does not need a stage simply leaves it out of the list, and the
%   remaining stages close up. The ORDER is the contract, not the coordinates:
%   a reader who has seen one model knows where to look in every other one.
%
%   단계 이름, 사슬 순서대로 / STAGE NAMES, in chain order
%
%     'command'      constants and step blocks — the setpoint / 상수·스텝 블록 — 설정값
%     'reference'    reference model, when there is one / 기준 모델 (있을 때)
%     'controller'   the control law / 제어 법칙
%     'allocation'   generalised force to shaft speed / 일반화 힘 -> 축 회전수
%     'plant'        the vessel, add_otter_plant / 선체 운동모델, add_otter_plant
%     'measurement'  selectors, scopes, logging, live view / 선택기·스코프·로깅·실시간 화면
%
%   NAME/VALUE
%
%     'Height'  scalar, or a struct with a field per stage. Default 90
%               스칼라 또는 단계별 필드를 가진 구조체. 기본 90
%     'Width'   scalar, or a struct with a field per stage. Default 110
%               스칼라 또는 단계별 필드를 가진 구조체. 기본 110
%     'Y'       top edge of the chain. Default 140 / 사슬의 위쪽 끝. 기본 140
%     'Pitch'   left-edge spacing between stages. Give it and every column is
%               spaced the same. Leave it out and the spacing follows the width
%               단계 사이의 왼쪽 끝 간격. 주면 모든 열이 같은 간격으로 놓이고,
%               빼면 간격이 폭을 따른다
%     'Gap'     empty space between one column and the next when 'Pitch' is not
%               given. Default 60
%               'Pitch' 를 주지 않았을 때 한 열과 다음 열 사이의 빈 공간. 기본 60
%
%   단계 별칭 / STAGE ALIASES  guidance·mission -> reference, control -> controller,
%                  thruster -> allocation. gnc_roles 의 낱말을 그대로 써도 된다
%                  The words of gnc_roles may be used as they are.
%
%   위치를 고정하는 이유 / WHY THE POSITIONS ARE FIXED
%
%   매주의 블록도는 강의 노트로 내보내진다. 위치를 고정하면 그 PNG 가 문서가
%   보여 줄 수 있는 폭 2000 px 아래로 유지되고, 블록을 손으로 더할 때마다
%   배치가 무너지는 일도 막는다.
%   The block diagram of every week is exported into the lecture note. Fixed
%   positions keep that PNG under the 2000 px width a document can display,
%   and they stop the layout decaying every time a block is added by hand.

ORDER = {'command','reference','controller','allocation','plant','measurement'};

%  같은 뜻의 다른 이름. gnc_roles·gnc_colour 가 쓰는 낱말을 그대로 받는다 —
%  빌더가 표 하나만 외우면 되게. model-layout.md §2 의 예제가 이 이름을 쓴다
%  Other names with the same meaning. The words used by gnc_roles·gnc_colour are accepted
%  as they are — so a builder needs to remember only one table. The examples in model-layout.md §2 use these names
ALIAS = {'guidance','reference'; 'mission','reference'; ...
         'control','controller'; 'thruster','allocation'};

p = inputParser;
p.addParameter('Height', 90);
p.addParameter('Width',  110);
p.addParameter('Y',      140);
p.addParameter('Pitch',  160);
p.addParameter('Gap',     60);
p.parse(varargin{:});
o = p.Results;

%  'Pitch' 를 주지 않으면 열 간격을 **그 단계의 폭 + Gap** 으로 잡는다.
%  단계마다 상자 수가 다르기 때문이다 — 제어기 하나뿐인 주차와, 제어기 옆에
%  개루프 스위치까지 선 주차를 같은 간격으로 놓으면 한쪽은 비고 한쪽은 겹친다.
%  Without 'Pitch', column spacing is **that stage's width + Gap**, because stages hold
%  different numbers of boxes — spacing a week with a single controller and a week with an
%  open-loop switch beside the controller equally leaves one empty and the other overlapping.
autoPitch = ismember('Pitch', p.UsingDefaults);

if ischar(stages) || isstring(stages), stages = {char(stages)}; end
stages = cellfun(@(s) lower(strtrim(s)), stages, 'UniformOutput', false);
for i = 1:numel(stages)
    k = find(strcmp(ALIAS(:,1), stages{i}), 1);
    if ~isempty(k), stages{i} = ALIAS{k,2}; end
end
stages = unique(stages, 'stable');

bad = setdiff(stages, ORDER);
if ~isempty(bad)
    error('gnc_chain:stage', ...
          'Unknown stage ''%s''. Valid stages are: %s.', bad{1}, strjoin(ORDER, ', '));
end

%  Order the requested stages by the chain, never by the order given. A builder
%  that lists them out of order still produces a diagram that reads correctly.
%  요청된 단계를 주어진 순서가 아니라 항상 사슬 순서로 정렬한다. 순서를 틀리게
%  적은 빌더도 바르게 읽히는 도면을 만든다.
[~, k] = ismember(stages, ORDER);
[~, s] = sort(k);
stages = stages(s);

x = 40;
P  = struct();
for i = 1:numel(stages)
    st = stages{i};
    w  = pick(o.Width,  st, 110);
    hh = pick(o.Height, st, 90);
    P.(st) = [x, o.Y, x + w, o.Y + hh];
    if autoPitch
        x = x + w + o.Gap;
    else
        x = x + o.Pitch;
    end
end
end

% -------------------------------------------------------------------------
function v = pick(spec, name, dflt)
%PICK  A scalar option, or the field of a per-stage struct, or the default.
%      스칼라 옵션, 또는 단계별 구조체의 필드, 또는 기본값.
if isstruct(spec)
    if isfield(spec, name), v = spec.(name); else, v = dflt; end
else
    v = spec;
end
end
