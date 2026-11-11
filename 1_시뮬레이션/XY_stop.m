% XY_STOP  XY_1_pen_sim · XY_2_pen_ros2 의 StopFcn — Run 버튼으로 돌렸을 때 결과를 바로 보여 준다
%   StopFcn of both models: show the results right after a Run-button simulation.
%
%   모델은 ReturnWorkspaceOutputs = off 라서 To Workspace 'log' 가 base workspace 에 바로 생긴다.
%   그것을 out.log 로 옮겨 스크립트와 같은 꼴로 만든 뒤 (>> XY_plot(out) 가능)
%   가상 실험 모델이면 XY_plot 을 부른다.
%   스크립트 (XY_1_pen_sim_run, XY_compare, XY_run_all, XY_record_live) 는 XY_batch = 1 로
%   이것을 끄고 sim(..., 'ReturnWorkspaceOutputs','on') 의 반환값을 쓴다.

if ~(exist('XY_batch','var') && XY_batch) && exist('log','var') && isstruct(log)
    out = struct('log', log);
    clear log
    if isfield(out.log, 'pen_true')
        XY_plot(out);
    else
        fprintf('XY_2_pen_ros2 종료: 기록은 out.log 에 있다\n');
    end
end
