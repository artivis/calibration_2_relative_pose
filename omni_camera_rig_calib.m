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

path_to_calib_cam1 = 'path/to/Calib_Results.mat';
path_to_calib_cam2 = 'path/to/Calib_Results.mat';

path_to_save = 'Results/';

if exist(path_to_calib_cam1,'file') ~= 2 
    
    display(['Could not find : ', path_to_calib_cam1]);
    return;
end
    
if exist(path_to_calib_cam2,'file') ~= 2 
    
    display(['Could not find : ', path_to_calib_cam2]);
    return;
end

% Load calibrations
calib_cam1 = load(path_to_calib_cam1); %Load Mei calib results
calib_cam2 = load(path_to_calib_cam2);

% Compute baseline
[T, R_T, t_T] = calcBaseLineMei(calib_cam1, calib_cam2);

% Retrieve cameras calibration information
image_cam1_size = [calib_cam1.images_without_I.ny calib_cam1.images_without_I.nx];

image_cam2_size = [calib_cam2.images_without_I.ny calib_cam2.images_without_I.nx];

xi_cam1 = calib_cam1.paramEst.xi;

K_cam1 = [calib_cam1.paramEst.gammac(1) 0 calib_cam1.paramEst.cc(1);...
          0 calib_cam1.paramEst.gammac(2) calib_cam1.paramEst.cc(2);...
          0 0 1];

xi_cam2 = calib_cam2.paramEst.xi;

K_cam2 = [calib_cam2.paramEst.gammac(1) 0 calib_cam2.paramEst.cc(1);...
          0 calib_cam2.paramEst.gammac(2) calib_cam2.paramEst.cc(2);...
          0 0 1];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Display Relative rotation only %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Cam 1 is master and so origin
Func_CameraDisplay(eye(3), [0,0,0]', 1, 1, 'b');
% Cam 2 - apply only rotation
Func_CameraDisplay(R_T, [0,0,0]', 1, 1, 'r');

% Rearrange data
system_calib = struct('K1', K_cam1, 'xi1', xi_cam1, ...
                      'K2', K_cam2, 'xi2', xi_cam2, ...
                      'baseline', T, ...
                      'image_cam1_size', image_cam1_size, ...
                      'image_cam2_size', image_cam2_size);

camera1.type = 'FishEye';
camera1.xi = xi_cam1;
camera1.K = K_cam1;
camera1.pose = eye(4);
camera1.image_size = image_cam1_size;

camera2.type = 'FishEye';
camera2.xi = xi_cam2;
camera2.K = K_cam2;
camera2.pose = T;
camera2.image_size = image_cam2_size;

system.extrinsicParam = T;
system.camera1 = camera1;
system.camera2 = camera2;

% Export to .mat
%save([path_to_save, 'Calib_int_ext.mat'], 'system_calib');

% Export to .yaml
exportCalib2Yaml(camera1, path_to_save, 'intrinsicParam_cam1');
exportCalib2Yaml(camera2, path_to_save, 'intrinsicParam_cam2');
%exportBaseline2Yaml(system, path_to_save, 'extrinsicParam');
