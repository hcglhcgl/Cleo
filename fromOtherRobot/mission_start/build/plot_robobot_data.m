% robot odo path
close all
clear
%%
%% new format window (4xADC)
ww002 = load('log_irdist_20191227_095545.054.txt'); % filter 23.5k || 330nF
% robobot mission heartbeat logfile
%%
figure(100)
data = ww002;
n = 1:300;
hold off
plot(data(n,1) - data(1,1), data(n,2))
hold on
plot(data(n,1) - data(1,1), data(n,3))
grid on
%legend('battery','message cnt', 'REGBOT load', 'bridge load')
legend('IR1', 'IR2')



%%
 data = load('log_irdist_20191227_095545.054.txt'); % filter 23.5k || 330nF
 h = figure(200)
 hold off
 plot(data(:,1) - data(1,1), data(:,2))
 hold on
 plot(data(:,1) - data(1,1), data(:,3))
 grid on
 legend('IR1', 'IR2')
 xlabel('sec')
 ylabel('distance [m]')
 saveas(h, 'ir_plot.png')