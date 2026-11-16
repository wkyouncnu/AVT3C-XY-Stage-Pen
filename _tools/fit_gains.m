function n = fit_gains(m, verbose)
%FIT_GAINS  식이 숨은 게인을 넓히고 그 옆 선만 다시 손질한다. 남은 것 0 이 합격선.
%           Widens gains whose expression is hidden and retouches only the nearby lines.
%           Zero remaining is the pass threshold.
%
%   n = fit_gains('SB9_continuous_done')
%   fit_gains(m, true)        % 무엇을 넓혔는지 나열한다 / lists what was widened
%
%   무엇을 하는가 / what it does
%       Gain 블록의 폭이 식보다 좁으면 Simulink 는 식을 숨기고 `-K-` 를 그린다.
%       교수 지시 2026-10-01 — *"게인은 변수명이 보이게 크게 할 것."*
%       이 도구는 좁은 게인이 **하나라도 있을 때만** 손을 댄다. 없으면 아무것도
%       바꾸지 않고 0 을 돌려준다 (이미 깨끗한 도면을 흔들지 않는다).
%       If a Gain block is narrower than its expression, Simulink hides it and draws `-K-`.
%       Instructor's instruction 2026-10-01 — *"Make gains large enough to show the
%       variable name."* This tool acts **only if at least one** narrow gain exists.
%       Otherwise it changes nothing and returns 0 (a clean diagram is not disturbed).
%
%   왜 따로 있는가 / why this is not just mss_style
%       `tidy_model` 은 배치를 여러 방식으로 해 보고 **지적이 적은 쪽**을 저장하며,
%       나빠지면 파일째 되돌린다. 그 되돌림이 `mss_style` 이 넓혀 둔 폭까지
%       되돌려 놓는다 — 배치는 좋아졌는데 게인은 다시 `-K-` 가 된다
%       (2026-10-01 SB2_mfcn_todo · SB9_continuous_done 이 그랬다).
%       `tidy_model` tries several layouts, saves the one with **fewer findings**, and
%       reverts the whole file if things get worse. That revert also undoes the widths
%       set by `mss_style` — the layout improves but the gains are `-K-` again
%       (as happened with SB2_mfcn_todo and SB9_continuous_done on 2026-10-01).
%
%       폭은 **고르는 것이 아니라 지켜야 하는 것**이다. 그래서 되돌림 뒤에 한 번
%       더 걸고, 넓히느라 생긴 선 문제만 그 자리에서 손질한다. 손질하는 것은
%       선과 태그뿐이고 블록 자리는 건드리지 않으므로 단계 순서(check_flow)는
%       그대로다.
%       Width is **not something to choose but something to keep**. So it is applied
%       once more after the revert, and only the line problems caused by widening are
%       fixed in place. Only lines and tags are retouched and block positions are not
%       touched, so the stage order (check_flow) stays the same.
%
%   돌려주는 것 / Returns
%       n   넓힌 게인의 수. 0 이면 손대지 않았다는 뜻이다
%           number of gains widened. 0 means nothing was touched

if nargin < 2, verbose = false; end
load_system(m);
if ~ischar(m) && ~isstring(m), m = bdroot(m); end
[~, m] = fileparts(char(m));

%  --- 좁은 게인이 있는가. 없으면 그대로 둔다 ----------------------------
%  --- Any narrow gains? If none, leave the model as is ---------------------
g = find_system(m, 'LookUnderMasks','all', 'BlockType','Gain');
tight = {};
for i = 1:numel(g)
    p = get_param(g{i}, 'Position');
    if (p(3)-p(1)) < need_width(g{i}), tight{end+1} = g{i}; end %#ok<AGROW>
end
n = numel(tight);
if n == 0, return, end

if verbose
    for i = 1:n
        p = get_param(tight{i}, 'Position');
        fprintf('    %-34s "%s"  폭 %d -> %d\n', ...
                get_param(tight{i},'Name'), get_param(tight{i},'Gain'), ...
                p(3)-p(1), need_width(tight{i}));
    end
end

%  --- 넓힌 뒤 선과 태그만 다시 손질한다 --------------------------------
%  --- After widening, retouch only lines and tags --------------------------
mss_style(m);                   % 폭은 mss_style 한 곳에서만 정한다 (표가 두 벌이 되면 어긋난다) / widths are set only in mss_style (two tables would drift apart)
sys = [{m}; find_system(m, 'LookUnderMasks','all', 'BlockType','SubSystem')];
for s = 1:numel(sys)
    try, lay_links(sys{s}); catch, end
    try, snug_tags(sys{s}); catch, end
end
square_lines(m);
save_system(m);

%  넓어진 블록의 이름표가 길어져 남의 선이 그 위를 지날 수 있다 (일곱째 항목).
%  nudge_labels 가 그 블록을 옆으로 비킨다. 이 도구는 모델을 닫고 돌아오므로
%  파일 이름으로 부르고, 끝나면 다시 열어 둔다 (호출자가 닫는다).
%  A widened block's name label gets longer and other lines may pass over it (item 7).
%  nudge_labels moves that block aside. That tool closes the model and comes back, so it is
%  called with the file name, and the model is reopened afterwards (the caller closes it).
close_system(m, 0);
try, nudge_labels(m); catch, end
load_system(m);
end

% -------------------------------------------------------------------------
function w = need_width(b)
%  식이 글자 그대로 보이는 데 필요한 폭. mss_style 의 gain_width 와 같은 셈이다.
%  Width needed to show the expression verbatim. Same calculation as gain_width in mss_style.
try, ex = get_param(b, 'Gain'); catch, ex = ''; end
w = max(50, numel(char(ex))*7 + 16);
end
