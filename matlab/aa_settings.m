% Settings file for all scrips
% Change the values here, and it will effect all the scripts
% If you write a new script, include this file

% Add robotics3d library and utils
% addpath('./robotics3D/')
% addpath('./functions/')

% Number of clone states we want to skip
% This prevents clutter in the exported images
skip_num = 5;

% Bounds that we want to have on our plot
% Normally we either do 3 sigma, or 1 sigma
sigma_bounds = 3;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Path to files
% path_groundtruth = '../logs/1489799715_797_kitti_player_oxts_gps.txt';
% path_estimate = '../logs/1489799715_655_toto_posewithcovariance.txt';

% LOAM + ORB + Prior (run 1)
%path_groundtruth = '../logs/1489854714_513_kitti_player_oxts_gps.txt';
%path_estimate = '../logs/1489854714_611_toto_posewithcovariance.txt';

% LOAM + Prior (run 2)
%path_groundtruth = '../logs/1489856512_798_kitti_player_oxts_gps.txt';
%path_estimate = '../logs/1489856512_411_toto_posewithcovariance.txt';

% LOAM (run 3)
%path_groundtruth = '../logs/1489857800_486_kitti_player_oxts_gps.txt';
%path_estimate = '../logs/1489857800_656_toto_posewithcovariance.txt';

% LOAM + ORB (run 4)
%path_groundtruth = '../logs/1489858934_345_kitti_player_oxts_gps.txt';
%path_estimate = '../logs/1489858934_869_toto_posewithcovariance.txt';


path_groundtruth = '../logs/1490772754_328_kitti_player_oxts_gps.txt';
path_estimate = '../logs/1490772754_563_toto_posewithcovariance.txt';

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Details about file we are going to read in
delimiterIn = ' ';
headerlinesIn = 20;

% Font size on the plots
fontsize = 15;


