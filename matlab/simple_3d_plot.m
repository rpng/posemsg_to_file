% 3D plot of position
% This script will take a ground truth and estimated path
% Displays a plot with both displayed
% Note: This does not use interpolation so it is prone to
% Note: Having more points for higher frequency sensors

% Close all old plots
close all
clear all

% Include setting file
aa_settings

% Read in the files
data_g = importdata(path_groundtruth,delimiterIn,headerlinesIn);
data_e = importdata(path_estimate,delimiterIn,headerlinesIn);

% Plot the 3d spacecurve
figure(1)
plot3(data_g.data(1:skip_num:end,2),data_g.data(1:skip_num:end,3),data_g.data(1:skip_num:end,4),'-ok'); hold on;
plot3(data_e.data(1:skip_num:end,2),data_e.data(1:skip_num:end,3),data_e.data(1:skip_num:end,4),'-*b'); hold on;

grid on
xlabel('x-distance (m)');
ylabel('y-distance (m)');
zlabel('z-distance (m)');
legend('ground truth','estimated path');
view([35 35])
axis equal



