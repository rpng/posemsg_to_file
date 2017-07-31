% Calculates the distance traveled along a path
% Will do both ground truth and estimated

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


% Loop through and calculate distance
dist = 0;
for index = 1:size(inter_gx(1:end-1),1)
    temp = (inter_gx(index) - inter_gx(index+1))^2 ...
            + (inter_gy(index) - inter_gy(index+1))^2 ...
            + (inter_gz(index) - inter_gz(index+1))^2;
     dist = dist + sqrt(temp);
end
% Print it out
fprintf('Disance Traveled (ground) = %.4f\n',dist);


% Loop through and calculate distance
dist = 0;
for index = 1:size(data_e.data(1:end-1,1),1)
    temp = (data_e.data(index,2) - data_e.data(index+1,2))^2 ...
            + (data_e.data(index,3) - data_e.data(index+1,3))^2 ...
            + (data_e.data(index,4) - data_e.data(index+1,4))^2;
     dist = dist + sqrt(temp);
end
% Print it out
fprintf('Disance Traveled (estimate) = %.4f\n',dist);







