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

% Path to files
%path_groundtruth = '../logs/1487298794_novatel_gps_fix.txt';
%path_estimate = '../logs/1487298794_toto_posewithcovariance.txt';
path_groundtruth = '../logs/1487449713_kitti_player_oxts_gps.txt';
path_estimate = '../logs/1487449713_toto_posewithcovariance.txt';

% Details about file we are going to read in
delimiterIn = ' ';
headerlinesIn = 3;