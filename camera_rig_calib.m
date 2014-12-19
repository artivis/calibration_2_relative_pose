%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%
%%%%%%%%%%%% Script Example of the SpheriCam calibration
%%%%%%%%%%%% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%
%%%%%%%%%%%% Author : Deray Jeremie 
%%%%%%%%%%%% Date : 01/7/2014

clc; clear all; close all;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%  Define some var %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

path_to_calib_cam1 = 'data/camera1/new_corner/Calib_Results.mat';

path_to_calib_camn{1} = 'data/camera2/new_corner/Calib_Results.mat';
%path_to_calib_camn{2} = 'data/camera3/Calib_Results.mat';

path_to_save = 'Results/';

if exist(path_to_calib_cam1,'file') ~= 2 
    display(['Could not find : ', path_to_calib_cam1]);
    return;
end

for i = 1 : size(path_to_calib_camn, 2)
    if exist(path_to_calib_camn{i}, 'file') ~= 2 
        display(['Could not find : ', path_to_calib_camn{i}]);
        return;
    end
end

% Load calibrations
calib_cam1 = load(path_to_calib_cam1);
calib_cam2 = load(path_to_calib_camn{1});

% Compute baseline
[T, R_T, t_T] = calcBaseLineBouget(calib_cam1, calib_cam2);

% Retrieve cameras calibration information
image_cam1_size = [calib_cam1.ny calib_cam1.nx];

image_cam2_size = [calib_cam2.ny calib_cam2.nx];

K_cam1 = calib_cam1.KK;

K_cam2 = calib_cam2.KK;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Display Relative rotation only %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Cam 1 is master and so origin
Func_CameraDisplay(eye(3), [0,0,0]', 1, 1, 'b');
% Cam 2 - apply only rotation
Func_CameraDisplay(R_T, [0,0,0]', 1, 1, 'r');

title(' b Z axis / g Y axis / r X axis - Cam1 blue / Cam2 red');

% Rearrange data
system_calib = struct('K1', K_cam1, 'K2', K_cam2, ...
                      'baseline', T, ...
                      'image_cam1_size', image_cam1_size, ...
                      'image_cam2_size', image_cam2_size);

camera1.type = 'pinhole';
camera1.xi = 0;
camera1.K = K_cam1;
camera1.pose = eye(4);
camera1.image_size = image_cam1_size;

camera2.type = 'pinhole';
camera2.xi = 0;
camera2.K = K_cam2;
camera2.pose = T;
camera2.image_size = image_cam2_size;

system.extrinsicParam = T;
system.camera1 = camera1;
system.camera2 = camera2;

% Export to .mat
%save([path_to_save, 'Calib_int_ext.mat'], 'system_calib');

% Export to .yaml
exportCalib2Yaml(camera1, path_to_save, 'intrinsicParam_cam1_newcorner');
exportCalib2Yaml(camera2, path_to_save, 'intrinsicParam_cam2_newcorner');
%exportBaseline2Yaml(system, path_to_save, 'extrinsicParam');


