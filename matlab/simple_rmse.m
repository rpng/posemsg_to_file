% Calculates the RMSE traveled along a path

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

% Subtract out the first timestamp
% This will give us relative time to the first recorded pose
timestamps = bsxfun(@minus,data_e.data(1:skip_num:end,1),data_e.data(1,1));


diff_x = inter_gx-data_e.data(1:skip_num:end,2);
diff_y = inter_gy-data_e.data(1:skip_num:end,3);
diff_z = inter_gz-data_e.data(1:skip_num:end,4);

% Calculate the square
diff = diff_x.^2 + diff_y.^2 + diff_z.^2;
diff = sqrt(diff);

% Divide by amount of nodes, and sqrt
%error = error / size(inter_gx(1:end),1);
%error = sqrt(error);

% Print it out
fprintf('RMSE sum (m) = %.4f\n',sum(diff));
fprintf('RMSE average (m) = %.4f\n',mean(diff));




% Plot it
figure(1);
set(gcf,'defaultuicontrolfontname','Times');
set(gcf,'defaultuicontrolfontsize',fontsize);
set(gcf,'defaultaxesfontname','Times');
set(gcf,'defaultaxesfontsize',fontsize);
set(gcf,'defaulttextfontname','Times');
set(gcf,'defaulttextfontsize',fontsize);
plot(timestamps,diff,'r'); hold on;
ylabel('RMSE Error (m)');
xlabel('time (sec)');





