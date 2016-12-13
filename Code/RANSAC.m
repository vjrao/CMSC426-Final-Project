%% Uses RANSAC to remove the table and wall planes
% Setup Paths and Read RGB and Depth Images
Path = '../Data/SingleObject/'; 
SceneNum = 0;
SceneName = sprintf('%0.3d', SceneNum);
FrameNum = num2str(1);

I = imread([Path,'scene_',SceneName,'/frames/frame_',FrameNum,'_rgb.png']);
ID = imread([Path,'scene_',SceneName,'/frames/frame_',FrameNum,'_depth.png']);

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

% iteration stuff
iter = 0;
maxIterations = 3000;
% number of total points
numpts = size(Pts, 1);
% threshold for number of inliers to likely be an irrelevant surface
num_inlier_threshold = 0.3;
% threshold for whether a point's distance is consider inlying
inlier_threshold = 2;
% keeps track of the largest possible set of points to remove (e.g.
% the largest plane)
bestinliers = 0;
bestnuminliers = 0;
%Every iteration will pick 4 points and find the equation for a plane
while iter < maxIterations && (bestnuminliers / numpts) < num_inlier_threshold
    %Get random values/points
    inds = randi(numpts, 4, 1);
    p1 = Pts(inds(1),:);
    p2 = Pts(inds(2),:);
    p3 = Pts(inds(3),:);
    % calculate plane equation
    n = cross(p2-p1, p3-p1);
    n = n / norm(n);
    d = -dot(n, p1);
    plane = [n(:); d];
    
    inliers = false(numpts, 1);
    % precompute divisor for efficiency
    % note: if necessary, norm squared (n'*n) is much faster
    divisor = norm(n);
    % this for loop could possibly be vectorized out
    for ii = 1:numpts
	% add the last 1 for the d coefficient
	p = [Pts(ii,:), 1];
	inliers(ii) = (abs(dot(plane, p)) / divisor) < inlier_threshold;
    end
    
    numinliers = sum(inliers);
    if numinliers > bestnuminliers
       bestnuminliers = numinliers;
       bestinliers = inliers;
    end
    
    if mod(iter, 10) == 0
       disp(iter)
    end

    iter = iter + 1;
end

colors = [r g b] / 255;
%%
figure;
pcshow(Pts, colors);
drawnow;
title('Before RANSAC');

% New way
[n, ~, p] = affine_fit(Pts(bestinliers,:));
d = -dot(n, p);
plane = [n(:);d];
inliers = false(numpts, 1);
divisor = norm(n);
for ii = 1:numpts
	% add the last 1 for the d coefficient
	p = [Pts(ii,:), 1];
    inliers(ii) = (abs(dot(plane, p)) / divisor) < inlier_threshold;
end
Pts = Pts(~inliers,:);
colors = colors(~inliers,:);
figure;
pcshow(Pts, colors);
drawnow;
title('After RANSAC');
