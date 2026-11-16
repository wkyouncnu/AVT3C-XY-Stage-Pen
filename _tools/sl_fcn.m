function sl_fcn(sys, path, code)
%SL_FCN  MATLAB Function 블록 안에 코드를 넣는다.
%        Puts code into a MATLAB Function block.
%
%   sl_fcn([m '/Guidance'], 'TaskRef', strjoin({...}, newline))
%
%   왜 위치를 다시 쓰는가 / Why the position is written back
%     코드를 넣으면 포트가 생기면서 블록이 제멋대로 자란다. 그대로 저장하면
%     다시 열었을 때 크기가 달라져 선이 사선이 된다. 넣기 전 위치를 기억했다가
%     그대로 돌려놓는다.
%     Inserting code creates ports and the block grows on its own. Saved as is,
%     its size differs on reopening and lines become diagonal. The position before
%     insertion is remembered and restored.
    S    = sfroot;
    fpos = get_param([sys '/' path], 'Position');
    ch   = S.find('-isa','Stateflow.EMChart','Path',[sys '/' path]);
    ch.Script = code;
    set_param([sys '/' path], 'Position', fpos);
end
