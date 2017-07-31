% 3D plot of bounds
% Just shows the bounds of the system
% Nothing else usefull really..

% Close all old plots
close all
clear all

% Include setting file
aa_settings

% Read in the files
data_g = importdata(path_groundtruth,delimiterIn,headerlinesIn);
data_e = importdata(path_estimate,delimiterIn,headerlinesIn);

% For now, since our timestamps are wrong
% Create a temp index array
%sizevec = size(data_e.data(1:skip_num:end,1),1);
%timestamps = [1:sizevec];
timestamps = bsxfun(@minus,data_e.data(1:skip_num:end,1),data_e.data(1,1));

% =================================================
% FINALLY, LETS PLOT THESE FIGURES!!!!!!!!!!!!
% =================================================
figure('name','position errors')
yNames = {'x-pos','y-pos','z-pos'};
for i = 1:3
    subplot(3,1,i);
    plot(timestamps,sigma_bounds*[sqrt(data_e.data(1:skip_num:end,8+i))';-sqrt(data_e.data(1:skip_num:end,8+i))'],'r');
    ylabel(yNames{i}); xlabel('time (sec)')
end
legend('error',[num2str(sigma_bounds),'\sigma error']); %saveas(gcf,'position_errors')

figure('name','rotation errors')
yNames = {'x-rot','y-rot','z-rot'};
for i = 1:3
    subplot(3,1,i);
    plot(timestamps,sigma_bounds*[sqrt(data_e.data(1:skip_num:end,11+i))';-sqrt(data_e.data(1:skip_num:end,11+i))']*180/pi,'r');
    ylabel(yNames{i}); xlabel('time (sec)')
end
legend('error',[num2str(sigma_bounds),'\sigma error']); %saveas(gcf,'rotation_errors')



