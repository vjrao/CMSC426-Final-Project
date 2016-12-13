%% Uses RANSAC to remove the table and wall planes
% Setup Paths and Read RGB and Depth Images
Path = '../Data/SingleObject/'; 
SceneNum = 1;
SceneName = sprintf('%0.3d', SceneNum);
FrameNum = num2str(1);

I = imread([Path,'scene_',SceneName,'/frames/image_',FrameNum,'_rgb.png']);
ID = imread([Path,'scene_',SceneName,'/frames/image_',FrameNum,'_depth.png']);

%% Extract 3D Point cloud
% Inputs:
% ID - the depth image
% I - the RGB image
% calib_file - calibration data path (.mat) 
%              ex) './param/calib_xtion.mat'
%              
% Outputs:
% pcx, pcy, pcz    - point cloud (valid points only, in RGB camera coordinate frame)
% r,g,b            - color of {pcx, pcy, pcz}
% D_               - registered z image (NaN for invalid pixel) 
% X,Y              - registered x and y image (NaN for invalid pixel)
% validInd	   - indices of pixels that are not NaN or zero
% NOTE:
% - pcz equals to D_(validInd)
% - pcx equals to X(validInd)
% - pcy equals to Y(validInd)

[pcx, pcy, pcz, r, g, b, D_, X, Y,validInd] = depthToCloud_full_RGB(ID, I, './params/calib_xtion.mat');
Pts = [pcx pcy pcz];

maxIterations = 3000;
inliers = cell(maxIterations, 1); %Max iterations possible for inliers

%Every iteration will pick 4 points and create a homography H from them
%It will apply H to ALL of the corresponding points found in
%featureMatch. It will then score how well H did on these matching
%points with SD. If that score is good enough for a certain point p, it
%will be added to the inlier of that iteration i. 
for i = 1:maxIterations
    %Get random values/points
    inliers{i,1} = zeros(size(Pts,1),3);
    pt1 = Pts(randi(size(Pts,1)),:);
    pt2 = Pts(randi(size(Pts,1)),:);
    pt3 = Pts(randi(size(Pts,1)),:);
    
    
    
    inliers{i,1} = temp(any(temp,2),:);
    %If 90% of points liked this H, we are done
    if size(inliers{i,1},1) >= 0.9*size(nZp1,1)
        break;
    end
end
max = 0;
index = 1;
%Find inlier set who casted most votes
for j = 1:i
    curr = size(inliers{j,1},1);
    if curr > max
        max = curr;
        index = j;
    end
end
%Return new homography from that set of inliers
maxLier = inliers{index,1};
%ret = max;
ret = est_homography(maxLier(:,3), maxLier(:,4), maxLier(:,1), maxLier(:,2));