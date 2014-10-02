%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%% 
%%%%%%%%%%%% Merges the two images of the SpheriCam
%%%%%%%%%%%% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%
%%%%%%%%%%%% Author : Deray Jeremie 
%%%%%%%%%%%% Date : 01/7/2014

function [image_merge] = getImage_Heal(img1, img2, system, ...
                                       mask1, mask2, size_pano, ...
                                       rotation_mat)
            
K_cam1 = system.K1;
K_cam2 = system.K2;

xi_cam1 = system.xi1;
xi_cam2 = system.xi2;

baseline_est = system.baseline;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%     meshgrid    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

grid_x_dl = meshgrid(1:size(img1,2),1:size(img1,1));
grid_y_dl = meshgrid(1:size(img1,1),1:size(img1,2))';
grid_z_dl = ones(size(img1,1),size(img1,2));

points_cam1 = [reshape(grid_x_dl,[],1) reshape(grid_y_dl,[],1)...
               reshape(grid_z_dl,[],1)]';
           
grid_x_dl = meshgrid(1:size(img2,2),1:size(img2,1));
grid_y_dl = meshgrid(1:size(img2,1),1:size(img2,2))';
grid_z_dl = ones(size(img2,1),size(img2,2));

points_cam2 = [reshape(grid_x_dl,[],1) reshape(grid_y_dl,[],1)...
               reshape(grid_z_dl,[],1)]';
         
% Free some memory
clear grid_*_dl;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%  Proj on mirror %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

pts_k_cam1 = K_cam1 \ points_cam1;

pts_k_cam2 = K_cam2 \ points_cam2;

% K_cam1 = K_cam1 .* 2; K_cam1(end,end) = 1;
% K_cam2 = K_cam2 .* 2; K_cam2(end,end) = 1;

% Free some memory
clear points_cam1 points_cam2;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%  Proj on sphere %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

alpha_cam1 = ( xi_cam1.*pts_k_cam1(3,:) + sqrt( pts_k_cam1(3,:).^2 + ...
             ( (1-xi_cam1^2).*(pts_k_cam1(1,:).^2 + pts_k_cam1(2,:).^2) ) ) ) ...
             ./ (pts_k_cam1(1,:).^2 + pts_k_cam1(2,:).^2 + pts_k_cam1(3,:).^2);
        
pts_sph_cam1 = [pts_k_cam1(1,:).*alpha_cam1; pts_k_cam1(2,:).*alpha_cam1;...
               (pts_k_cam1(3,:).*alpha_cam1) - xi_cam1];

pts_sph_cam1 = real(pts_sph_cam1);

alpha_cam2 = ( xi_cam2.*pts_k_cam2(3,:) + sqrt( pts_k_cam2(3,:).^2 + ...
             ( (1-xi_cam2^2).*(pts_k_cam2(1,:).^2 + pts_k_cam2(2,:).^2) ) ) ) ...
             ./ (pts_k_cam2(1,:).^2 + pts_k_cam2(2,:).^2 + pts_k_cam2(3,:).^2);
        
pts_sph_cam2 = [pts_k_cam2(1,:).*alpha_cam2; pts_k_cam2(2,:).*alpha_cam2;...
               (pts_k_cam2(3,:).*alpha_cam2) - xi_cam2];

pts_sph_cam2 = real(pts_sph_cam2);

% Apply baseline to pts on sphere from cam2
pts_sph_cam2 = baseline_est(1:3,1:3) * pts_sph_cam2;

% Display the points on the sphere
% scatter3(pts_sph_cam1(1,1:100:end)', pts_sph_cam1(2,1:100:end)',...
%          pts_sph_cam1(3,1:100:end)');


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%   Get spherical coord   %%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Rotate both hemisphere
% This is for image display convenience
pts_sph_cam1 = rotation_mat * pts_sph_cam1;
pts_sph_cam2 = rotation_mat * pts_sph_cam2;

% Convert to sphere coordinates - azimuth / elevation / radius
[azi_cam1, ele_cam1, ~] = cart2sph(pts_sph_cam1(1,:),...
                                   pts_sph_cam1(2,:),...
                                   pts_sph_cam1(3,:));

[azi_cam2, ele_cam2, ~] = cart2sph(pts_sph_cam2(1,:),...
                                   pts_sph_cam2(2,:),...
                                   pts_sph_cam2(3,:));

% sph_pts = [azi_cam1 azi_cam2; ele_cam1 ele_cam2; ones(1,2*length(azi_cam1))];

sph_pts = unwrapHealpix([azi_cam1 azi_cam2 ; ele_cam1 ele_cam2], 4, 3);

% Free some memory
clear pts_sph_cam1 pts_sph_cam2 azi_cam1 azi_cam2 ele_cam1 ele_cam2;

col_r_dl = img1(:,:,1);
col_g_dl = img1(:,:,2);
col_b_dl = img1(:,:,3);

% Free some memory
clear img1;

% Num of pixel in img1
pix_dl = size(col_r_dl, 1) * size(col_r_dl, 2);

% Rescale points accordingly to size_pano
sph_pts(1,:) = sph_pts(1,:) + abs(min(sph_pts(1,:)));
sph_pts(1,:) = sph_pts(1,:) / abs(max(sph_pts(1,:)));

sph_pts(2,:) = sph_pts(2,:) + abs(min(sph_pts(2,:)));
sph_pts(2,:) = sph_pts(2,:) / abs(max(sph_pts(2,:)));

sph_pts(1,:) = round(sph_pts(1,:) * size_pano(1));
sph_pts(1,:) = sph_pts(1,:) + 1;

sph_pts(2,:) = round(sph_pts(2,:) * size_pano(2));
sph_pts(2,:) = sph_pts(2,:) + 1;

% Re-distribution of the color
% This is not optimal
ind_dl = 1;

image_merge = zeros(size_pano(2)+1, size_pano(1)+1, 3);

inpaint_mask = mask1;

for i_dl = 1 : length(sph_pts)
    
    if ( inpaint_mask(ind_dl) == 1 )
            
        image_merge(sph_pts(2,i_dl), sph_pts(1,i_dl),1) = col_r_dl(ind_dl);
        image_merge(sph_pts(2,i_dl), sph_pts(1,i_dl),2) = col_g_dl(ind_dl);
        image_merge(sph_pts(2,i_dl), sph_pts(1,i_dl),3) = col_b_dl(ind_dl);
        
    end
    
    if (ind_dl == pix_dl)
        
        col_r_dl = img2(:,:,1);
        col_g_dl = img2(:,:,2);
        col_b_dl = img2(:,:,3);
        
        ind_dl = 0;
        
        inpaint_mask = mask2;
        
    end
    
    ind_dl = ind_dl + 1;

end

end