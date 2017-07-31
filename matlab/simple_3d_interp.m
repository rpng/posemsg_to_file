% 3D pose plot with interpolated ground truth
% This script will take a ground truth and estimated path
% And will take the timestamps of the estimate and find the
% Interpolated pose of the ground truth. It then gets plotted

% Close all old plots
close all
clear all

% Include setting file
aa_settings

% Read in the files
data_g = importdata(path_groundtruth,delimiterIn,headerlinesIn);
data_e = importdata(path_estimate,delimiterIn,headerlinesIn);

% Interpolate the ground truth pose (time_g, pos_g, time_e)
% https://www.mathworks.com/help/matlab/ref/interp1.html
inter_gx = interp1(data_g.data(1:skip_num:end,1),data_g.data(1:skip_num:end,2),data_e.data(1:skip_num:end,1),'spline');
inter_gy = interp1(data_g.data(1:skip_num:end,1),data_g.data(1:skip_num:end,3),data_e.data(1:skip_num:end,1),'spline');
inter_gz = interp1(data_g.data(1:skip_num:end,1),data_g.data(1:skip_num:end,4),data_e.data(1:skip_num:end,1),'spline');

% Subtract out the ground truth starting value (so that they both start in
% the same place at the origin (todo: is this logic right?)
inter_gx = bsxfun(@minus,inter_gx,inter_gx(1,1)-data_e.data(1,2));
inter_gy = bsxfun(@minus,inter_gy,inter_gy(1,1)-data_e.data(1,3));
inter_gz = bsxfun(@minus,inter_gz,inter_gz(1,1)-data_e.data(1,4));

% Plot the 3d spacecurve
figure(1)
set(gcf,'defaultuicontrolfontname','Times');
set(gcf,'defaultuicontrolfontsize',fontsize);
set(gcf,'defaultaxesfontname','Times');
set(gcf,'defaultaxesfontsize',fontsize);
set(gcf,'defaulttextfontname','Times');
set(gcf,'defaulttextfontsize',fontsize);
%plot3(inter_gx,inter_gz,inter_gy,'-ok'); hold on;
plot3(inter_gx,inter_gy,inter_gz,'--k'); hold on;
plot3(data_e.data(1:skip_num:end,2),data_e.data(1:skip_num:end,3),data_e.data(1:skip_num:end,4),'-b'); hold on;


grid on
xlabel('x-distance (m)');
ylabel('y-distance (m)');
zlabel('z-distance (m)');
%legend('ground truth','estimated path', 'Location','northwest');
legend('ground truth','estimated path', 'Location','southeast');
%view([-200 35])
view([0 90])
axis equal
set(gcf,'Position',[0 0 900 600])
%saveas(gcf,'plot_3d_interpolated.png');
%print('-dpng','-r900','plot_3d_interpolated.png')

