function n = mss_style(mdl, verbose)
%MSS_STYLE  모델 안의 모든 블록 크기를 MSS 가 쓰는 치수로 맞춘다.
%           Resize every block in a model to the sizes MSS uses.
%
%   mss_style(mdl)              조용히 적용한다 / apply, silently
%   n = mss_style(mdl, true)    적용하고 무엇이 바뀌었는지 나열한 뒤 개수를 돌려준다
%                               apply, list what changed, return the count
%
%   빌더의 끝에서, save_system 직전에 한 번 부른다.
%   Call it once at the end of a builder, just before save_system.
%
%   숫자의 출처 / WHERE THE NUMBERS COME FROM
%
%   지어낸 값이 아니다. Tools/MSS/SIMULINK/mssSimulinkDemos 의 모든 데모 모델에서
%   (Sum 211 개, Gain 242 개, Inport 354 개 등) 블록 종류마다 잰 치수의 중앙값이다.
%   아래 표가 그 측정이다. MSS 가 갱신되면 이 파일 끝의 조사 코드로 다시 만든다.
%   Not invented. Measured across every demo model in
%   Tools/MSS/SIMULINK/mssSimulinkDemos - 211 Sum blocks, 242 Gains, 354
%   Inports and so on - and the median of each block type taken. The table
%   below is that measurement. Regenerate it with the survey at the bottom of
%   this file if MSS is ever updated.
%
%   왜 중요한가 / WHY IT MATTERS
%
%   Simulink 기본 크기는 거의 모든 경우 MSS 보다 크다. 기본 Gain 은 폭 40 (MSS 50 x 36),
%   기본 Sum 은 25 x 40 사각형 (MSS 20 x 20 원), 기본 Transfer Fcn 은 퍼져 있다. PDF 의
%   쪽폭으로 읽어야 하는 도면에서 큰 블록은 사슬을 벌려 놓아 신호의 줄을 놓치게 한다.
%   MSS 에 맞추면 MSS 데모를 이 모델 옆에 연 학생이 양쪽에서 같은 작도 관례를 본다.
%   Simulink's defaults are larger than MSS's in almost every case: a default
%   Gain is 40 wide against 50 x 36, a default Sum is a 25 x 40 rectangle
%   against a 20 x 20 circle, a default Transfer Fcn sprawls. On a diagram
%   that has to be read at page width in a PDF, oversized blocks push the
%   chain apart, and the reader loses the line of the signal. Matching MSS
%   also means a student who opens an MSS demo next to one of these models
%   sees the same drawing conventions in both.
%
%   중심은 그대로 둔다 / THE CENTRE IS PRESERVED
%
%   블록은 자기 중심을 기준으로 크기를 바꾸므로, 지나가는 신호와 줄을 맞춘 블록은
%   그대로 맞아 있다. Simulink 가 선을 붙여 둔 채 다시 그린다.
%   Blocks are resized about their own centre, so a block that was lined up
%   with the signal passing through it stays lined up. Simulink keeps the
%   lines attached and redraws them.
%
%   게인 블록은 식에 맞춰 넓힌다 / GAIN BLOCKS ARE WIDENED TO FIT THEIR EXPRESSION
%   (교수 지시 2026-10-01 / instructor's instruction 2026-10-01)
%
%   MSS 치수 50 x 36 은 숫자 게인에는 맞지만 `Kp_psi` 같은 **변수 이름**은 그
%   폭에 들어가지 않는다. Simulink 는 식이 블록보다 넓으면 식을 숨기고 `-K-`
%   를 그린다. 그러면 도면만 보고는 **어느 게인인지 알 수 없다.** 가독성이 MSS
%   치수보다 앞서므로 Gain 만 예외로 두고 식 길이에 맞춰 폭을 늘린다.
%   글자당 GAIN_PX_PER_CHAR, 좌우 여백 GAIN_PAD, 최소 폭은 MSS 의 50 px 그대로.
%   숫자 게인(`2` · `0.5`)은 식이 짧아 50 px 에서 멈추므로 달라지지 않는다.
%   중심은 그대로 두므로 배선이 틀어지지 않는다.
%   The MSS size 50 x 36 fits numeric gains, but a **variable name** such as `Kp_psi`
%   does not fit that width. When the expression is wider than the block, Simulink
%   hides it and draws `-K-`, and the diagram alone **no longer tells which gain it is**.
%   Readability comes before MSS sizes, so Gain alone is an exception and its width
%   grows with the expression length: GAIN_PX_PER_CHAR per character, side margin
%   GAIN_PAD, minimum width the MSS 50 px. Numeric gains (`2`, `0.5`) are short and stop
%   at 50 px, so they do not change. The centre is kept, so wiring is not disturbed.
%
%   상수도 같다. Constant 는 값이 블록보다 넓으면 `-C-` 를 그린다 — `-K-` 와
%   똑같이 **어느 값인지 알 수 없는** 상태다. 상수 값은 모두 작업공간 변수
%   이름이므로(sl_const 규칙) 이름이 길수록 가려진다. 2026-10-15 에 W05·W06 의
%   `F_plugin_max` 가 그렇게 가려졌다. 같은 자로 Constant 도 넓힌다.
%   Constants are the same. A Constant whose value is wider than the block draws `-C-`
%   — just like `-K-`, **the value cannot be identified**. Constant values are all
%   workspace variable names (sl_const rule), so longer names get hidden. On 2026-10-15
%   `F_plugin_max` in W05 and W06 was hidden this way. Constants are widened by the same rule.
%
%   건드리지 않는 것 / WHAT IS LEFT ALONE
%
%   SubSystem 블록. 신호 사슬의 여섯 단계이며 크기는 gnc_chain 이 일부러 정한다.
%   Mux · Demux 도 높이를 그대로 둔다 — 높이가 나르는 신호 수를 나타내므로,
%   억지로 맞추면 모델을 잘못 보여 준다.
%   SubSystem blocks. Those are the six stages of the signal chain and their
%   sizes come from gnc_chain, which places them deliberately. Mux and Demux
%   keep their height too - the height is how many signals they carry, and
%   forcing it would misrepresent the model.

if nargin < 2, verbose = false; end

%  블록 종류 -> [폭 높이], MSS 데모 모델의 중앙값 / block type -> [width height], median over the MSS demo models
S = { 'Sum',           20  20
      'Gain',          50  36
      'Constant',      55  30
      'Integrator',    30  30
      'TransferFcn',   60  36
      'Saturate',      30  30
      'Scope',         30  30
      'Step',          30  30
      'Clock',         20  20
      'Inport',        30  14
      'Outport',       30  14
      'ToWorkspace',   60  30
      'Product',       45  35
      'Terminator',    20  20
      'Math',          30  30
      'Trigonometry',  30  30
      'Signum',        30  30
      'Switch',        30  30
      'Rounding',      30  30
      'RandomNumber',  30  28
      'Fcn',           60  22 };

want = containers.Map(S(:,1), num2cell(cell2mat(S(:,2:3)), 2));

blocks = find_system(mdl, 'LookUnderMasks','all', 'Type','Block');
n = 0;
for i = 1:numel(blocks)
    b = blocks{i};
    try, t = get_param(b, 'BlockType'); catch, continue; end
    if ~isKey(want, t), continue; end

    %  입력이 여럿인 Scope 는 높이가 곧 포트 간격이다. 30 px 로 줄이면 포트가
    %  겹치고 들어오는 선이 전부 얕은 사선이 된다. 크기를 그대로 둔다.
    %  For a Scope with several inputs, the height is the port spacing. Shrunk to 30 px the
    %  ports overlap and every incoming line becomes a shallow diagonal. Its size is kept.
    if strcmp(t, 'Scope')
        try
            if numel(get_param(b,'PortHandles').Inport) > 1, continue; end
        catch
        end
    end

    wh = want(t);

    %  Gain 은 식이, Constant 는 값이 보여야 한다. MSS 폭을 **최소값**으로 두고
    %  글자 길이에 맞춰 늘린다. 안 그러면 Simulink 가 `-K-` · `-C-` 를 그린다.
    %  A Gain must show its expression, a Constant its value. The MSS width is the **minimum**,
    %  widened to the text length. Otherwise Simulink draws `-K-` / `-C-`.
    if strcmp(t, 'Gain')
        wh(1) = max(wh(1), text_width(b, 'Gain'));
    elseif strcmp(t, 'Constant')
        wh(1) = max(wh(1), text_width(b, 'Value'));
    end

    p  = get_param(b, 'Position');
    if (p(3)-p(1)) == wh(1) && (p(4)-p(2)) == wh(2), continue; end

    cx = (p(1)+p(3))/2;  cy = (p(2)+p(4))/2;
    q  = round([cx-wh(1)/2, cy-wh(2)/2, cx+wh(1)/2, cy+wh(2)/2]);
    set_param(b, 'Position', q);
    n = n + 1;
    if verbose
        fprintf('    %-38s %s  %dx%d -> %dx%d\n', get_param(b,'Name'), t, ...
                p(3)-p(1), p(4)-p(2), wh(1), wh(2));
    end
end

%  A round summing junction is the MSS signature. Any Sum still drawn as a
%  rectangle is one a builder created without add_sum; fix it here rather than
%  leaving one slab in an otherwise consistent diagram.
%  둥근 합산점은 MSS 의 표식이다. 아직 사각형으로 그려진 Sum 은 빌더가 add_sum 없이
%  만든 것이다. 다른 데는 일관된 도면에 덩어리 하나를 남기지 않고 여기서 고친다.
%
%  부호 문자열은 **막대('|') 하나로 시작한다** — 교수 지시 2026-10-14.
%  막대는 포트가 아니라 원의 위 자리를 비우는 띄우개다. 비워 두면 첫 입력이
%  왼쪽 가장자리에 남고 둘째 입력만 아래로 내려가, 지령은 왼쪽에서 들어오고
%  되먹임만 밑에서 올라온다. 지령선과 출력선이 한 줄로 수평이 되어 꺾임이 없다.
%  2026-10-02 에는 거꾸로 **떼어 내고** 있었다. 그래서 `add_sum` 이 붙인 막대가
%  빌더 끝에서 다시 지워져 모델 71 개가 전부 막대 없이 저장되어 있었다 (2026-10-15).
%  자세한 이유는 add_sum 의 머리글에 적어 두었다. 둥근 Sum 은 전부 여기를 지난다.
%  The sign string **starts with one bar ('|')** — instructor's instruction 2026-10-14.
%  The bar is not a port but a spacer that leaves the top slot of the circle empty. Then the
%  first input stays on the left edge and only the second input moves to the bottom: the
%  command enters from the left and only the feedback comes up from below. Command and
%  output lines form one horizontal row with no bends. On 2026-10-02 the bar was instead
%  being **stripped**, so the bar added by `add_sum` was removed again at the end of the
%  builder and all 71 models were saved without it (2026-10-15). The full reason is in the
%  header of add_sum. Every round Sum passes through here.
sums = find_system(mdl, 'LookUnderMasks','all', 'BlockType','Sum');
for i = 1:numel(sums)
    changed = false;
    if ~strcmp(get_param(sums{i}, 'IconShape'), 'round')
        set_param(sums{i}, 'IconShape', 'round');
        changed = true;
    end
    sg   = get_param(sums{i}, 'Inputs');
    want = ['|' strrep(sg, '|', '')];
    if ~strcmp(sg, want)
        set_param(sums{i}, 'Inputs', want);
        changed = true;
    end
    if changed, n = n + 1; end
end

%  크기를 바꾸면 Simulink 는 선의 끝점만 새 포트로 당기고 이웃 점은 두어 끝 토막이
%  2~7 px 기운 사선이 된다 (2026-09-19). 이웃 점을 따라 옮겨 직각으로 되돌린다.
%  On resize, Simulink pulls only the line end point to the new port and leaves the neighbour
%  point, so the end segment tilts by 2~7 px (2026-09-19). The neighbour point is moved along
%  to restore right angles.
square_lines(mdl);

if verbose, fprintf('    %d blocks restyled in %s\n', n, mdl); end
end

% -------------------------------------------------------------------------
function w = text_width(b, prm)
%TEXT_WIDTH  블록의 글자(Gain 의 식, Constant 의 값)가 그대로 보이는 폭 [px].
%            Width [px] at which a block's text (Gain expression, Constant value) shows in full.
%
%   Simulink 는 글자가 블록보다 넓으면 `-K-`(Gain) · `-C-`(Constant) 를
%   그린다. 블록을 글자가 들어갈 만큼 키우면 변수 이름이 그대로 보인다.
%   폭만 늘리고 높이는 그대로 둔다.
%   When the text is wider than the block, Simulink draws `-K-` (Gain) / `-C-` (Constant).
%   Enlarging the block to fit the text shows the variable name as is.
%   Only the width grows; the height stays.
%
%   7 px/글자는 Simulink 의 기본 도면 글꼴(Helvetica 10 pt)을 도면에서 재
%   얻은 값이다. 여백 16 px 는 좌우 8 px 씩이다.
%   7 px/character was measured on a diagram in Simulink's default font (Helvetica
%   10 pt). The 16 px margin is 8 px on each side.
GAIN_PX_PER_CHAR = 7;
GAIN_PAD         = 16;
GAIN_MAX         = 200;     % 이보다 길면 식이 아니라 수식이다. 거기서 멈춘다 / longer than this is a formula, not a name; stop here

try
    ex = get_param(b, prm);
catch
    w = 0; return
end
if ~ischar(ex) && ~isstring(ex), w = 0; return; end
ex = char(ex);

%  행렬 게인·행렬 상수는 여러 줄로 그려진다 — 가장 긴 줄로 잰다
%  Matrix gains and matrix constants are drawn on several lines — measure the longest line
lines = strsplit(ex, {char(10), char(13), ';'});
len   = max(cellfun(@(s) numel(strtrim(s)), lines));

w = min(GAIN_MAX, len*GAIN_PX_PER_CHAR + GAIN_PAD);
end

% =========================================================================
%  The survey that produced the table above. Kept so the numbers can be
%  checked rather than trusted.
%  위 표를 만든 조사 코드. 숫자를 믿지 않고 확인할 수 있도록 남겨 둔다.
%
%   root = <...>/Tools/MSS;  addpath(genpath(root));
%   dd = dir(fullfile(root,'SIMULINK','mssSimulinkDemos','demo*.slx'));
%   S = containers.Map('KeyType','char','ValueType','any');
%   for k = 1:numel(dd)
%       [~,m] = fileparts(dd(k).name);
%       evalc(sprintf('load_system(''%s'')', fullfile(dd(k).folder, dd(k).name)));
%       b = find_system(m,'LookUnderMasks','all','FollowLinks','on','Type','Block');
%       for i = 1:numel(b)
%           t = get_param(b{i},'BlockType');  p = get_param(b{i},'Position');
%           if ~isKey(S,t), S(t) = []; end
%           S(t) = [S(t); p(3)-p(1) p(4)-p(2)];
%       end
%       evalc(sprintf('close_system(''%s'',0)', m));
%   end
%   for k = sort(S.keys), v = S(k{1});
%       fprintf('%-16s %4d  %dx%d\n', k{1}, size(v,1), median(v(:,1)), median(v(:,2)));
%   end
