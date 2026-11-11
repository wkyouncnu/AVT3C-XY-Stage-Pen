% XY_1_PEN_SIM_RUN  가상 실험 한 번: 설정 -> sim (ROS 2 루프백) -> 그림 -> 동영상
%
%   >> XY_1_pen_sim_run
%
%   봐야 할 숫자: 펜 횡오차 RMS (보상 / 보상 없음), 스테이지 사용 범위

here = fileparts(mfilename('fullpath'));  cd(here);
XY_setup;
if ~isfile('XY_1_pen_sim.slx'), build_xy_models(1); end

XY_batch = 1;  animate = 0;
out = sim('XY_1_pen_sim', 'ReturnWorkspaceOutputs','on');
XY_batch = 0;  animate = 1;
M = XY_plot(out);
if ~exist('make_video','var') || make_video
    XY_animate_video(out);
end
