function XY_compare_plot()
%XY_COMPARE_PLOT  XY_compare 결과 그림 두 장 (logs/compare.mat -> img/)
%   그림 1  XY_compare_bar.png   경우별 펜 횡오차 RMS · 최대, 로봇만 RMS
%   그림 2  XY_compare_time.png  펜 횡오차 시계열 (기준 · 스테이지 느슨 · 센서 이상적) + 스테이지 Y (기준 · 중앙 유지)
%   Plot XY_compare results: per-case bar chart and selected time histories.

here = fileparts(mfilename('fullpath'));
load(fullfile(here,'logs','compare.mat'), 'C', 'S');
imgdir = fullfile(here,'img');
navy = [0.12 0.23 0.37];  grey = [0.62 0.62 0.62];  lgrey = [0.85 0.85 0.85];

ord = {'base','v15','v60','pred','v60pred','loose','center','all','zk05','ideal'};
lab = {'기준 30 mm/s','15 mm/s','60 mm/s','지연 보상','60 mm/s+지연 보상','스테이지 느슨', ...
       '중앙 유지','지연+중앙','ZeroKey 0.5 mm','센서 이상적'};
k = cellfun(@(n) find(strcmp({C.name}, n)), ord);
R = [[C(k).e_rms]' [C(k).e_max]'];
N = [C(k).n_rms]';

f = figure('Color','w','Position',[60 60 1200 520]);
tl = tiledlayout(f,1,2,'TileSpacing','compact');
a = nexttile(tl); b = barh(a, R, 'grouped', 'BarWidth', 0.85);
b(1).FaceColor = navy;  b(2).FaceColor = lgrey;  b(2).EdgeColor = grey;
set(a,'YTick',1:numel(ord),'YTickLabel',lab,'YDir','reverse','FontSize',10); grid(a,'on');
xlabel(a,'펜 횡오차 [mm]'); legend(a,{'RMS','최대'},'Location','southeast');
title(a,'스테이지 보상 후 펜 횡오차','FontWeight','normal');
for i = 1:numel(ord), text(a, R(i,1)+0.05, i-0.18, sprintf('%.2f', R(i,1)), 'FontSize',9, 'Color',navy); end
a2 = nexttile(tl); bb = barh(a2, N, 'BarWidth', 0.5, 'FaceColor', grey, 'EdgeColor','none');
set(a2,'YTick',1:numel(ord),'YTickLabel',[],'YDir','reverse','FontSize',10); grid(a2,'on');
xlabel(a2,'펜 횡오차 RMS [mm]'); title(a2,'스테이지 없이 로봇만 (같은 주행)','FontWeight','normal');
for i = 1:numel(ord), text(a2, N(i)+0.2, i, sprintf('%.1f', N(i)), 'FontSize',9); end
bb.FaceColor = grey;
exportgraphics(f, fullfile(imgdir,'XY_compare_bar.png'), 'Resolution',150);

g = figure('Color','w','Position',[80 80 1200 620]);
tl = tiledlayout(g,2,1,'TileSpacing','compact');
a = nexttile(tl); hold(a,'on'); grid(a,'on');
sel = {'loose','base','ideal'};  col = {grey, navy, [0.75 0.35 0.2]};
nm = {'스테이지 느슨 (2 Hz, 명령 5 Hz)','기준','센서 잡음 0'};
for i = 1:3
    s = S(strcmp({S.name}, sel{i}));
    plot(a, s.t, s.e, 'Color', col{i}, 'LineWidth', 1.1, 'DisplayName', nm{i});
end
ylabel(a,'펜 횡오차 [mm]'); legend(a,'Location','northeast','Orientation','horizontal'); ylim(a,[-5 5]);
a = nexttile(tl); hold(a,'on'); grid(a,'on');
yline(a, 50.8, 'k:', '중앙 50.8 mm', 'LabelHorizontalAlignment','left');
yline(a, [0 101.6], 'Color', [0.8 0.3 0.3]);
s = S(strcmp({S.name},'base'));   plot(a, s.t, s.sy, 'Color', grey, 'LineWidth', 1.2, 'DisplayName','기준');
s = S(strcmp({S.name},'center')); plot(a, s.t, s.sy, 'Color', navy, 'LineWidth', 1.2, 'DisplayName','중앙 유지');
ylim(a,[0 101.6]); ylabel(a,'스테이지 Y [mm]'); xlabel(a,'펜 다운 후 시간 [s]');
legend(a,{'','','','기준','중앙 유지'},'Location','northeast','Orientation','horizontal');
exportgraphics(g, fullfile(imgdir,'XY_compare_time.png'), 'Resolution',150);
end
