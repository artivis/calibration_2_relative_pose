%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%
%%%%%%%%%%%% Define the equation system to estimate the baseline rotation
%%%%%%%%%%%%
%%%%%%%%%%%% From equation 6 in :
%%%%%%%%%%%%
%%%%%%%%%%%% 'Calibration of Non-overlapping Cameras - Application to
%%%%%%%%%%%% Vision-Based Robotics' - Lebraly & al. Universite Blaise-
%%%%%%%%%%%% Pascal, Clermont-Ferrant [1]
%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%
%%%%%%%%%%%%    Input : R1 / R2 - 3*3*n rotation matrix
%%%%%%%%%%%%                      for n pose of each camera
%%%%%%%%%%%%                      where R1_i is relative to R1_1
%%%%%%%%%%%%                      and   R2_i is relative to R2_1
%%%%%%%%%%%%
%%%%%%%%%%%%    Output : A - equation system
%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%
%%%%%%%%%%%% Author : Deray Jeremie 
%%%%%%%%%%%% Date : 01/7/2014

function [A] = DefEqSysRot(R1, R2)

nb_ = size(R1,3);

A = [];

for i = 2 : nb_
    
    A = [A; eye(9) - kron(R1(:,:,i),R2(:,:,i)) ];

end

end