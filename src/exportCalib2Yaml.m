%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%% 
%%%%%%%%%%%% Export camera calibration to yaml file
%%%%%%%%%%%%    Opencv readable
%%%%%%%%%%%% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%
%%%%%%%%%%%%    Input : camera - struct() : 
%%%%%%%%%%%%                        camera.type = 'FishEye';
%%%%%%%%%%%%                        camera.xi = xi;
%%%%%%%%%%%%                        camera.K = K;
%%%%%%%%%%%%                        camera.image_size = image_size;
%%%%%%%%%%%%            filePath - Path to save yaml file
%%%%%%%%%%%%            fileName - Name of the file - no extension
%%%%%%%%%%%%
%%%%%%%%%%%%    Output : message - Message from fopen. see fopen()
%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%
%%%%%%%%%%%% Author : Deray Jeremie 
%%%%%%%%%%%% Date : 01/7/2014

function [message] = exportCalib2Yaml(camera, filePath, fileName)

[fid, message] = fopen(strcat(filePath ,fileName,'.yaml'), 'w');

yaml_header = '%YAML:1.0';

fprintf(fid, '%s\n\n',yaml_header);

fprintf(fid, '# Camera parameters\n\n\n' );

fprintf(fid,'type: %s\n\n',camera.type);

fprintf(fid,'xi: %.8f\n\n',camera.xi);

fprintf(fid,'K: !!opencv-matrix\n');

fprintf(fid,'   rows: %i\n',size(camera.K,1));

fprintf(fid,'   cols: %i\n',size(camera.K,2));

fprintf(fid,'   dt: d\n   data: [');

K = sprintf(' %.8g, ',camera.K');

K(end-1) = [];

fprintf(fid,K);

fprintf(fid,']\n\n');

fprintf(fid,'image_size:\n');

fprintf(fid,'   rows: %i\n',camera.image_size(1));

fprintf(fid,'   cols: %i',camera.image_size(2));

fprintf(fid,'\n');

fclose(fid);

end