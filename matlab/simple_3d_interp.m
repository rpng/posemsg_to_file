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

% Plot the 3d spacecurve
figure(1)
plot3(inter_gx,inter_gy,inter_gz,'-ok'); hold on;
plot3(data_e.data(1:skip_num:end,2),data_e.data(1:skip_num:end,3),data_e.data(1:skip_num:end,4),'-*b'); hold on;

grid on
xlabel('x-distance (m)');
ylabel('y-distance (m)');
zlabel('z-distance (m)');
legend('ground truth','estimated path');
view([35 35])
axis equal
