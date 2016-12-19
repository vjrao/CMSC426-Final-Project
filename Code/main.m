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

scene = 1;
numframes = length(dir()) / 2 - 2;

name = strcat(Path,'scene_',sprintf('%0.3d',scenes(scene)), '/frames/frame_', int2str(1));
I = imread([name,'_rgb.png']);
ID = imread([name,'_depth.png']);
[pcx, pcy, pcz, r, g, b, D_, X, Y,validInd] = depthToCloud_full_RGB(ID, I, './params/calib_xtion.mat');
Pts = [pcx pcy pcz];
colors = [r g b] / 255;
[Pts, colors] = RANSAC(Pts, colors);
[Pts, colors] = denoise(Pts, colors);
%figure, pcshow(Pts, colors);

name = strcat(Path,'scene_',sprintf('%0.3d',scenes(scene)), '/frames/frame_', int2str(2));
I = imread([name,'_rgb.png']);
ID = imread([name,'_depth.png']);
[pcx, pcy, pcz, r, g, b, D_, X, Y,validInd] = depthToCloud_full_RGB(ID, I, './params/calib_xtion.mat');
Pts2 = [pcx pcy pcz];
colors2 = [r g b] / 255;
[Pts2, colors2] = RANSAC(Pts2, colors2);
[Pts2, colors2] = denoise(Pts2, colors2);
%figure, pcshow(Pts2, colors2);

numpts = min([size(Pts, 1), size(Pts2, 1)]);
Pts = Pts(1:numpts, :);
Pts2 = Pts2(1:numpts, :);

Pts3 = ICP(Pts, Pts2, 0);
figure, pcshow(Pts3);

%{
ptCloud = pointCloud(Pts);
normals = pcnormals(ptCloud, 10);
M = transf_mat(normals, Pts, Pts2);
Pts = [Pts, ones(numpts, 1)];
Pts2 = [Pts2, ones(numpts, 1)];
Pts3 = zeros(size(Pts));
for ii=1:numpts
    Pts3(ii,:) = M * Pts2(ii, :)';
end
%}

%{
dists = zeros((numpts-1)*(numpts-2)/2, 1);
ctr = 1;
for ii=1:numpts
    for jj=(ii+1):numpts
	dists(ctr) = sum((Pts(ii,:)-Pts(jj,:)).^2);
	ctr = ctr + 1;
    end
end
mindist = min(dists);
%}

%{
k = boundary(Pts);
b1 = unique(k(:));
bound = Pts(b1, :);
k2 = boundary(Pts2);
b2 = unique(k2(:));
bound2 = Pts2(b2, :);
idx = knnsearch(bound, bound2);
%}

%{
x = ptCloud.Location(1:50:end,1);
y = ptCloud.Location(1:50:end,2);
z = ptCloud.Location(1:50:end,3);
u = normals(1:50:end,1);
v = normals(1:50:end,2);
w = normals(1:50:end,3);
figure, pcshow(Pts, colors); hold on;
quiver3(x,y,z,u,v,w);
hold off;
%}



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
