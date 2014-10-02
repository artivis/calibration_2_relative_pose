%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%% 
%%%%%%%%%%%% Compute the 2D coodinates (image plan) of spherical points
%%%%%%%%%%%% according to HealPix unwrapping scheme
%%%%%%%%%%%% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%
%%%%%%%%%%%% Author : Deray Jeremie 
%%%%%%%%%%%% Date : 01/7/2014

function [output] = unwrapHealpix(input, H, K)

H = 4;

K = 3;

theta_c = asin(2/3);

output = zeros(size(input,1),size(input,2));

for i = 1 : length(input)
    
    if (abs(input(2,i)) < theta_c)
    
        output(:,i) = [input(1,i); ((3*pi)/(2*H))*sin(input(2,i))];
    
    elseif (input(2,i) > theta_c && input(2,i)>0)
        
        xi = sqrt(3*(1-abs(sin(input(2,i)))));
        
        phi_c = -pi + ( 2 * floor( ((input(1,i)+pi)*H)/(2*pi) ) + 1 )*(pi/H) ;
        
        output(:,i) = [(phi_c + (input(1,i)-phi_c)*xi); (pi/H)*(2-xi)];
        
    else
        
        xi = sqrt(3*(1-abs(sin(input(2,i)))));
        
        phi_c = -pi + ( 2 * floor( ((input(1,i)+pi)*H)/(2*pi) ) + 1 )*(pi/H) ;
        
        output(:,i) = [(phi_c + (input(1,i)-phi_c)*xi); -(pi/H)*(2-xi)];

    end
    
end

end
