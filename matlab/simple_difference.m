% 3D error plot of position without bounds
% This script will take a ground truth and estimated path
% Displays a plot with 3 subplots with the error of
% Each 3d position compared to the ground truth

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

% Create the array
inter_g = [inter_gx,inter_gy,inter_gz];

% Subtract out the first timestamp
% This will give us relative time to the first recorded pose
timestamps = bsxfun(@minus,data_e.data(1:skip_num:end,1),data_e.data(1,1));


% =================================================
% FINALLY, LETS PLOT THESE FIGURES!!!!!!!!!!!!
% =================================================
figure('name','position errors')
yNames = {'x-pos (m)','y-pos (m)','z-pos (m)'};
set(gcf,'defaultuicontrolfontname','Times');
set(gcf,'defaultuicontrolfontsize',fontsize);
set(gcf,'defaultaxesfontname','Times');
set(gcf,'defaultaxesfontsize',fontsize);
set(gcf,'defaulttextfontname','Times');
set(gcf,'defaulttextfontsize',fontsize);
for i = 1:3
    subplot(3,1,i);
    plot(timestamps,abs(data_e.data(1:skip_num:end,1+i)-inter_g(:,i)),'r'); hold on;
    %plot(timestamps,sigma_bounds*[sqrt(data_e.data(1:skip_num:end,8+i))';-sqrt(data_e.data(1:skip_num:end,8+i))'],'r');
    %ylim([-yLimits{i}-0.25*yLimits{i},yLimits{i}+0.25*yLimits{i}]);
    ylabel(yNames{i});
end
xlabel('time (sec)')
%legend('error',[num2str(sigma_bounds),'\sigma error']); %saveas(gcf,'position_errors')




