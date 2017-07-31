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
yNames = {'x-pos (m)','y-pos (m)','z-pos (m)'};
set(gcf,'defaultuicontrolfontname','Times');
set(gcf,'defaultuicontrolfontsize',fontsize);
set(gcf,'defaultaxesfontname','Times');
set(gcf,'defaultaxesfontsize',fontsize);
set(gcf,'defaulttextfontname','Times');
set(gcf,'defaulttextfontsize',fontsize);
for i = 1:3
    subplot(3,1,i);
    plot(timestamps,sigma_bounds*sqrt(data_e.data(1:skip_num:end,8+i))','r');
    ylabel(yNames{i});
end
xlabel('time (sec)');
legend([num2str(sigma_bounds),'\sigma error']);
%saveas(gcf,'errors_position.png');
%print('-dpng','-r900','errors_position.png')

% =========================================================================
% figure('name','rotation errors')
% yNames = {'x-rot (deg)','y-rot (deg)','z-rot (deg)'};
% set(gcf,'defaultuicontrolfontname','Times');
% set(gcf,'defaultuicontrolfontsize',fontsize);
% set(gcf,'defaultaxesfontname','Times');
% set(gcf,'defaultaxesfontsize',fontsize);
% set(gcf,'defaulttextfontname','Times');
% set(gcf,'defaulttextfontsize',fontsize);
% for i = 1:3
%     subplot(3,1,i);
%     plot(timestamps,sigma_bounds*sqrt(data_e.data(1:skip_num:end,11+i))'*180/pi,'r');
%     ylabel(yNames{i});
% end
% xlabel('time (sec)');
% legend([num2str(sigma_bounds),'\sigma error']);
% set(gca,'FontSize',fontsize);
%saveas(gcf,'errors_rotation.png');
%print('-dpng','-r900','errors_rotation.png')



