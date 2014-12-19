%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%
%%%%%%%%%%%% Extrinsic parameters calibration of a 2 cameras system
%%%%%%%%%%%% Based on results from Bouget's calibration Toolbox &
%%%%%%%%%%%% 'Calibration of Non-overlapping Cameras - Application to
%%%%%%%%%%%% Vision-Based Robotics' - Lebraly & al. Universite Blaise-
%%%%%%%%%%%% Pascal, Clermont-Ferrant [1]
%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%
%%%%%%%%%%%%    Input : calib_cam1 / calib_cam2 - struct() : 
%%%%%%%%%%%%                    from Bouget's calibration toolbox
%%%%%%%%%%%%                    default name : 'Calib_Results.mat'
%%%%%%%%%%%%
%%%%%%%%%%%%    Output : T - camera baseline. 4*4 matrice [ R t ; 0 1 ] 
%%%%%%%%%%%%                 relative to cam1
%%%%%%%%%%%%             R_t - above R
%%%%%%%%%%%%             t_T - above t
%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%
%%%%%%%%%%%% Author : Deray Jeremie 
%%%%%%%%%%%% Date : 01/7/2014

function [T, R_T, t_T] = calcBaseLineBouget(calib_cam1, calib_cam2)

if (sum(calib_cam1.active_images) ~= sum(calib_cam2.active_images))
    error('Must have the same number of calibration images for each camera');
else
    nb_im = sum(calib_cam1.active_images);
end

for i = 1 : nb_im
    
    tc_ind = ['Tc_' int2str(i)];
    qw_ind = ['Rc_' int2str(i)];
    
    Tw_cam1{i} = getfield(calib_cam1, tc_ind);
    Qw_cam1{i} = getfield(calib_cam1, qw_ind);
    
    Tw_cam2{i} = getfield(calib_cam2, tc_ind);
    Qw_cam2{i} = getfield(calib_cam2, qw_ind);
end

M_r_cam1 = zeros(4,4,nb_im);
M_wf_cam1 = zeros(4,4,nb_im);

M_r_cam2 = zeros(4,4,nb_im);
M_wf_cam2 = zeros(4,4,nb_im);

for i = 1 : nb_im
    
    %Retrieve 4*4 matrix [ R t ; 0 1 ] for each pose of each camera
    %M_r_i are poses relative to first pose of camera i
    %M_wf_i are poses in world coordinate frame of camera i
    
    M_wf_cam1(:,:,i) = inv([Qw_cam1{i}, Tw_cam1{i}; 0 0 0 1]);
    
    M_wf_cam2(:,:,i) = inv([Qw_cam2{i}, Tw_cam2{i}; 0 0 0 1]);
    
    M_r_cam1(:,:,i) = M_wf_cam1(:,:,1) \ M_wf_cam1(:,:,i);
    
    M_r_cam2(:,:,i) = M_wf_cam2(:,:,1) \ M_wf_cam2(:,:,i);    
    
end

%Build linear equations system (6) [1]
rotation_sys = DefEqSysRot(M_r_cam1(1:3,1:3,:), M_r_cam2(1:3,1:3,:));

%Solve system
[~,~,v] = svd(rotation_sys);
x_r = v(:,end);
rotation_est = reshape(x_r,3,3)';

%Force orthogonality paper [1] way
% rotation_est = rotation_est * ( ( det(rotation_est) / abs(det(rotation_est)) ) ...
%                         * (abs(det(rotation_est)))^(-1/3) );

%Force orthogonality
[ur,~,vr] = svd(rotation_est);
rotation_est = ur * vr' *  ( det(rotation_est) / abs(det(rotation_est)) );

%Build linear equations system (8) [1]
[Atr, Btr] = DefEqSysTr(M_r_cam1, M_r_cam2, rotation_est);

%Solve system
[~, ~, vtr] = svd([Atr -Btr]);
translation_est = [vtr(1,end)./vtr(end,end)...
                   vtr(2,end)./vtr(end,end)...
                   vtr(3,end)./vtr(end,end)]';

%Handle to ghost function
f = @(x)optimBaseline(x, M_r_cam1, M_r_cam2);

%Set non-linear optimization options
lsqopts = optimset('Algorithm', {'levenberg-marquardt',.005}, ...
                   'MaxIter', 800, 'ScaleProblem', 'Jacobian',...
                   'TolFun', 1*10^-13, 'TolX', 1*10^-13, ...
                   'MaxFunEvals', 2400); %,'Display','iter' 
                   
% Using Rodrigues rotation vector
rotation_est = rodrigues(rotation_est);

%Non-linear optimization
[x, ~] = lsqnonlin(f, [rotation_est; translation_est],...
                      [], [], lsqopts);

%%Forcing orthogonality paper [1] way
% x(1:3,1:3) = x(1:3,1:3) * ( ( det(x(1:3,1:3)) / abs(det(x(1:3,1:3))) ) ...
%                         * (abs(det(x(1:3,1:3))))^(-1/3) );

R_T = rodrigues(x(1:3));
t_T = x(4:end);

%%Retrieving transformation matrix cam2 -> cam1
x = [R_T t_T];

%%Retrieving transformation matrix cam2 -> cam1
%%Homogeneous transformation
T = [x; 0 0 0 1];
disp('Estimated baseline : ');
disp(T);

end
