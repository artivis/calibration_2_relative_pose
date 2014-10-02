%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%% 
%%%%%%%%%%%% Export baseline calibration to yaml file
%%%%%%%%%%%%    Opencv readable
%%%%%%%%%%%% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%
%%%%%%%%%%%%    Input : system - struct() : 
%%%%%%%%%%%%                        camera1.type = 'FishEye';
%%%%%%%%%%%%                        camera1.xi = xi2;
%%%%%%%%%%%%                        camera1.K = K2;
%%%%%%%%%%%%                        camera1.image_size = image_size2;
%%%%%%%%%%%%
%%%%%%%%%%%%                        camera2.type = 'FishEye';
%%%%%%%%%%%%                        camera2.xi = xi2;
%%%%%%%%%%%%                        camera2.K = K2;
%%%%%%%%%%%%                        camera2.image_size = image_size2;
%%%%%%%%%%%%
%%%%%%%%%%%%                        system.extrinsicParam = T;
%%%%%%%%%%%%                        system.camera1 = camera1;
%%%%%%%%%%%%                        system.camera2 = camera2;
%%%%%%%%%%%%
%%%%%%%%%%%%            filePath - Path to save yaml file
%%%%%%%%%%%%            fileName - Name of the file - no extension
%%%%%%%%%%%%
%%%%%%%%%%%%    Output : message - Message from fopen. see fopen()
%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%
%%%%%%%%%%%% Author : Deray Jeremie 
%%%%%%%%%%%% Date : 01/7/2014

function [message] = exportBaseline2Yaml(system, filePath, fileName)

[fid, message] = fopen(strcat(filePath ,fileName,'.yaml'), 'w');

yaml_header = '%YAML:1.0';

fprintf(fid, '%s\n\n',yaml_header);

fprintf(fid, '# System extrinsic parameters \n\n\n' );

fprintf(fid,'extrinsicParam: !!opencv-matrix\n');

fprintf(fid,'   rows: %i\n',size(system.extrinsicParam,1));

fprintf(fid,'   cols: %i\n',size(system.extrinsicParam,2));

fprintf(fid,'   dt: d\n   data: [');

K = sprintf(' %.8g, ',system.extrinsicParam');

K(end-1) = [];

fprintf(fid,K);

fprintf(fid,' ]\n');

fclose(fid);

end