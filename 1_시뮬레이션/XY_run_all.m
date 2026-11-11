% XY_RUN_ALL  모델 생성 -> 시뮬레이션 -> 그림 -> 동영상을 한 번에 돌리고 기록을 남긴다
%
%   >> run('C:\...\MC_Handover\5_XY스테이지\1_시뮬레이션\XY_run_all.m')
%
%   기록: logs/run_log.txt (명령창 출력 전체), logs/status.txt (단계별 OK / FAIL)

here = fileparts(mfilename('fullpath'));  cd(here);
if ~isfolder('logs'), mkdir('logs'); end
diary off;  if isfile(fullfile('logs','run_log.txt')), delete(fullfile('logs','run_log.txt')); end
diary(fullfile('logs','run_log.txt'));
status = {};
fprintf('XY_run_all 시작 %s  (MATLAB %s)\n', datestr(now), version);

try
    XY_setup;  status{end+1} = 'setup OK';
catch e
    status{end+1} = ['setup FAIL: ' e.message];  disp(getReport(e,'extended'));
end

try
    build_xy_models([1 2]);  status{end+1} = 'build OK';
catch e
    status{end+1} = ['build FAIL: ' e.message];  disp(getReport(e,'extended'));
end

try
    out = sim('XY_1_pen_sim');  status{end+1} = 'sim OK';
catch e
    status{end+1} = ['sim FAIL: ' e.message];  disp(getReport(e,'extended'));
end

try
    M = XY_plot(out);  status{end+1} = sprintf('plot OK  e_rms %.2f mm (no comp %.2f mm)', M.e_rms_mm, M.enc_rms_mm);
catch e
    status{end+1} = ['plot FAIL: ' e.message];  disp(getReport(e,'extended'));
end

try
    XY_animate_video(out);  status{end+1} = 'video OK';
catch e
    status{end+1} = ['video FAIL: ' e.message];  disp(getReport(e,'extended'));
end

fid = fopen(fullfile('logs','status.txt'), 'w', 'n', 'UTF-8');
fprintf(fid, '%s\n', status{:});  fclose(fid);
fprintf('\n'); fprintf('%s\n', status{:});
diary off;
