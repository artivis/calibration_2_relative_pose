%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%% Assuming a stereo system with rigid transformation we have
%%%%%%%%%%%%    - cam1_i*baseline = cam2_i
%%%%%%%%%%%%    - baseline*cam2_i = cam1_i
%%%%%%%%%%%% this function computes :
%%%%%%%%%%%% 
%%%%%%%%%%%%    x = sum( norm2(cam1_i*baseline - baseline*cam2_i) )
%%%%%%%%%%%% 
%%%%%%%%%%%% Based on results from Mei's calibration Toolbox &
%%%%%%%%%%%% 'Calibration of Non-overlapping Cameras - Application to
%%%%%%%%%%%%  Vision-Based Robotics' - Lebraly & al. Universite Blaise-
%%%%%%%%%%%%  Pascal, Clermont-Ferrant [1]
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%
%%%%%%%%%%%%    Input : baseline - estimation to be refined of the baseline 
%%%%%%%%%%%%                       6*1 vector with baseline(1:3) =
%%%%%%%%%%%%                       rodrigues angles and baseline(4:6) =
%%%%%%%%%%%%                       estimated translation
%%%%%%%%%%%%
%%%%%%%%%%%%            cam1 / cam2 - 4*4 matrix [ R t ; 0 1 ] 
%%%%%%%%%%%%                          for each pose of each camera
%%%%%%%%%%%%                          where pose_i_cam1 is 
%%%%%%%%%%%%                          relative to pose_1_cam1
%%%%%%%%%%%%                          and pose_i_cam2 is 
%%%%%%%%%%%%                          relative to pose_1_cam2
%%%%%%%%%%%%
%%%%%%%%%%%%    Output : out - refined baseline (6*1 vector)
%%%%%%%%%%%%                       
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%
%%%%%%%%%%%% Author : Deray Jeremie 
%%%%%%%%%%%% Date : 01/7/2014

function [out] = optimBaseline(baseline, cam1, cam2)

n_pose = size(cam1,3);

rotation_mat = rodrigues(baseline(1:3));
baseline = [rotation_mat baseline(4:end); 0 0 0 1];

out = 0;

for i = 2 : n_pose
   
    sys1 = cam1(:,:,i)*baseline;
    sys2 = baseline*cam2(:,:,i);
    
    out = out + norm( sys1 - sys2 , 2);
    
end
end