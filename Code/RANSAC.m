function [Pts, colors] = RANSAC(imageName)
% Uses RANSAC to remove the table and wall planes
% Setup Paths and Read RGB and Depth Images

I = imread([imageName,'_rgb.png']);
ID = imread([imageName,'_depth.png']);

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

%% Thresholding
ratio = 1/3;
x_size = size(pcx);
x_start = ceil(ratio*x_size);
x_end = ceil((1-ratio)*x_size);

y_size = size(pcy);
y_start = ceil(ratio*y_size);
y_end = ceil((1-ratio)*y_size);

z_size = size(pcz);
z_start = ceil(ratio*z_size);
z_end = ceil((1-ratio)*z_size);


Pts = [pcx(x_start:x_end) pcy(y_start:y_end) pcz(z_start:z_end)];
colors = [r(x_start:x_end) g(y_start:y_end) b(z_start:z_end)] / 255;


%% RANSACing to get rid of wall and table
numTimes = 2;
for q = 1:numTimes
    iter = 0;
    maxIterations = 3000;
    % number of total points
    numpts = size(Pts, 1);
    % threshold for number of inliers to likely be an irrelevant surface
    num_inlier_threshold = 0.3;
    % threshold for whether a point's distance is consider inlying
    inlier_threshold = 5;
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
            inliers(ii) = (abs(p*plane) / divisor) < inlier_threshold;
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
    
    % Printing results
    figure;
    pcshow(Pts, colors);
    drawnow;
    title(strcat('Before RANSAC', int2str(q)));
    
    % Create best fit plane of bestinliers and then remove that plane
    [n, ~, p] = affine_fit(Pts(bestinliers,:));
    d = -dot(n, p);
    plane = [n(:);d];
    inliers = false(numpts, 1);
    divisor = norm(n);
    for ii = 1:numpts
        p = [Pts(ii,:), 1];
        inliers(ii) = (abs(p*plane) / divisor) < inlier_threshold;
    end
    
    Pts = Pts(~inliers,:);
    colors = colors(~inliers,:);
    
    figure;
    pcshow(Pts, colors);
    drawnow;
    title(strcat('After RANSAC', int2str(q)));
end
end
