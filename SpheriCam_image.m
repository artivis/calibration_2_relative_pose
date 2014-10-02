%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%% 
%%%%%%%%%%%% Merges the two images of the SpheriCam
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
path_to_calib_system = 'Results/Calib_int_ext.mat';

path_to_image1 = 'path/to/image_cam1.jpg';
path_to_image2 = 'path/to/image_cam2.jpg';

path_to_mask1 = 'path/to/image_mask_cam1.jpg';
path_to_mask2 = 'path/to/image_mask_cam2.jpg';

size_pano = [1200 400];

unwrapping_type = 'plattecarre';
% unwrapping_type = 'healpix';

pre_processing = false;

inpaint = false;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%  Load calib data %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if exist(path_to_calib_system,'file') == 2
    
    system = load(path_to_calib_system);
    system = system.system_calib;
    
    K_cam1 = system.K1;
    K_cam2 = system.K2;
    
    xi_cam1 = system.xi1;
    xi_cam2 = system.xi2;
    
    baseline_est = system.baseline;
    
else
    
    disp(['Could not find : ', path_to_calib]);
    
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%    Read image   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

image_cam1 = imread(path_to_image1);
image_cam1 = double(image_cam1);

image_cam2 = imread(path_to_image2);
image_cam2 = double(image_cam2);

%%%%%% Mask %%%%%%

mask1 = imread(path_to_mask1);
mask1 = double(mask1);
mask1(mask1 <= 240) = 0;
mask1(mask1 > 0) = 1;

mask2 = imread(path_to_mask2);
mask2 = double(mask2);
mask2(mask2 <= 240) = 0;
mask2(mask2 > 0) = 1;


%%%%%% Display %%%%%%

% image_cam1(:,:,1) = image_cam1(:,:,1).*mask1;
% image_cam1(:,:,2) = image_cam1(:,:,2).*mask1;
% image_cam1(:,:,3) = image_cam1(:,:,3).*mask1;
% 
% image_cam2(:,:,1) = image_cam2(:,:,1).*mask2;
% image_cam2(:,:,2) = image_cam2(:,:,2).*mask2;
% image_cam2(:,:,3) = image_cam2(:,:,3).*mask2;
% 
% figure;
% imshow(uint8(image_cam1));
% 
% figure;
% imshow(uint8(image_cam2));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%     Get pano    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Rotation matrix to apply to the spherical image (both hemi-sphere)
% only for displaying convenience

rotation_mat = getRotationMat(0,90,0);

%%%%%% Merge images %%%%%%

if ( strcmp(unwrapping_type, 'healpix') )

    % 'Healpix' unwrapping
    image_merge = getImage_Heal(image_cam1, image_cam2, system, ...
                                mask1, mask2, size_pano, rotation_mat);
else
    
	% 'Plate-carre' unwrapping
    image_merge = getImage(image_cam1, image_cam2, system, ...
                           mask1, mask2, size_pano, rotation_mat);
end

%%%%%% light pre-processing %%%%%%

if (pre_processing)

    image_merge(:,:,1) = preProcessing(image_merge(:,:,1));
    image_merge(:,:,2) = preProcessing(image_merge(:,:,2));
    image_merge(:,:,3) = preProcessing(image_merge(:,:,3));

end

%%%%%% Inpaint un-filled pixels %%%%%%

if (inpaint)

    % This Inpaint fct takes a lot of memory,
    % it might fail for big size panoramic img
    
    mask_inpaint = repmat(image_merge(:,:,1) == 0, [1 1 3]);

    image_merge(mask_inpaint) = NaN;

    image_merge(:,:,1) = inpaint_nans(image_merge(:,:,1));
    image_merge(:,:,2) = inpaint_nans(image_merge(:,:,2));
    image_merge(:,:,3) = inpaint_nans(image_merge(:,:,3));

end

%%%%%% Display %%%%%%

figure;
imshow(uint8(image_merge),[]);

%%% To use the following display (which is nice)
%%% you need to install YAWTB (Yes Another Wavelet ToolBox)

% figure;
% yashow(uint8(image_merge),'spheric');


