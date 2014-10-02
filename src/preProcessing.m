%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%% 
%%%%%%%%%%%% Apply a pre-processing to smooth a bit the difference in
%%%%%%%%%%%% brightness between dark and bright image area
%%%%%%%%%%%% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%
%%%%%%%%%%%% Author : Deray Jeremie 
%%%%%%%%%%%% Date : 01/7/2014

function [output] = preProcessing(input)
    
    input = log(  ( input ./ max(max(input)) ) + 1 );
    
    mean_val = mean(mean(input));
    
    output = input - mean_val;
        
    output = ( output ./ norm(output) );
    
    output = output - min(min(output));
    output = output ./ max(max(output));
    output = output .* 255;
    
end
