%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%
%%%%%%%%%%%% Define the equation system to estimate the baseline
%%%%%%%%%%%% translation
%%%%%%%%%%%%
%%%%%%%%%%%% From equation 8 in :
%%%%%%%%%%%%
%%%%%%%%%%%% 'Calibration of Non-overlapping Cameras - Application to
%%%%%%%%%%%% Vision-Based Robotics' - Lebraly & al. Universite Blaise-
%%%%%%%%%%%% Pascal, Clermont-Ferrant [1]
%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%
%%%%%%%%%%%%    Input : R1 / R2 - 4*4 matrix [ R t ; 0 1 ] 
%%%%%%%%%%%%                      for each pose of each camera
%%%%%%%%%%%%                      where R1_i is relative to R1_1
%%%%%%%%%%%%                      and   R2_i is relative to R2_1
%%%%%%%%%%%%            dR - previously estimated R
%%%%%%%%%%%%
%%%%%%%%%%%%    Output : A - left side equation system
%%%%%%%%%%%%             B - right side equation system
%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%
%%%%%%%%%%%% Author : Deray Jeremie 
%%%%%%%%%%%% Date : 01/7/2014

function [A, B] = DefEqSysTr(T1, T2, dR)

nb_ = size(T1,3);

A = [];
B = [];

for i = 2 : nb_
    
    A = [A; eye(3) - T1(1:3,1:3,i)];
    
    B = [B; T1(1:3,4,i) - dR*T2(1:3,4,i)];

end

end