%% Extract 3D Point cloud
%%                                
% Inputs:
%                               
% ID - the depth image
%                               
% I - the RGB image
%                               
% calib_file - calibration data path (.mat)
%                               
%              ex) './param/calib_xtion.mat'
%                                            
%
%                              
% Outputs:
%                               
% pcx, pcy, pcz    - point cloud (valid points only, in RGB camera
% coordinate frame)             
% r,g,b            - color of {pcx, pcy, pcz}
%                               
% D_               - registered z image (NaN for invalid pixel)
%                               
% X,Y              - registered x and y image (NaN for invalid
% pixel)                            
% validInd     - indices of pixels that are not NaN or zero
%                               
% NOTE:
%                               
% - pcz equals to D_(validInd)
%                               
% - pcx equals to X(validInd)
%                               
% - pcy equals to Y(validInd)
%                               

Path = '../Data/SingleObject/';
scenes = [0, 1, 2, 6, 8, 12, 22, 23];
% note: the frames are named frame_num_(rgb/depth).png starting from 0
% subtract 2 for the . and .. directories and

scene = 8;
numframes = length(dir()) / 2 - 2;

name = strcat(Path,'scene_',sprintf('%0.3d',scenes(scene)), '/frames/frame_', int2str(1));
I = imread([name,'_rgb.png']);
ID = imread([name,'_depth.png']);
[pcx, pcy, pcz, r, g, b, D_, X, Y,validInd] = depthToCloud_full_RGB(ID, I, './params/calib_xtion.mat');

Pts1 = [pcx pcy pcz];
colors1 = [r g b] / 255;
[Pts2, colors2] = RANSAC(Pts1, colors1);
[Pts3, colors3] = denoise(Pts2, colors2);
figure, pcshow(Pts3, colors3);

%{
normals = pcnormals(pointCloud(Pts));
ii = 2;
name = strcat(Path,'scene_',sprintf('%0.3d',scenes(scene)), '/frames/frame_', int2str(ii));
[Pts2, colors2] = RANSAC(name);

% mapping Pts2 onto Pts1
while true
  normals = pcnormals(pointCloud(Pts));
  M = transf_mat(normals, Pts, Pts2);
  Pts2 = bsxfun(@times, Pts2, M);
  pause
end
  %}  
%end
