function [Pts, colors] = RANSAC(Pts, colors, varargin)
    %% Uses RANSAC to remove the table and wall planes
    % Setup Paths and Read RGB and Depth Images
	 
    if nargin > 2
        showImages = varargin{1};
    else
        showImages = false;
    end


    %% Hard center thresholding by constant ratio
    ratio = 1/3;

    numpts = size(Pts, 1);
    start = floor(numpts * ratio);
    stop = ceil(numpts * (1 - ratio));
    Pts = Pts(start:stop, :);
    colors = colors(start:stop, :);

    % keeps all the points in a central cube of size ratio x ratio x ratio
%{
    x_min = min(Pts(:,1));
    x_max = max(Pts(:,1));
    x_size = x_max - x_min;
    x_start = x_min + ratio * x_size;
    x_end = x_max - ratio * x_size;
    x_box = (x_start < Pts(:, 1)) & (Pts(:, 1) < x_end);

    y_min = min(Pts(:,1));
    y_max = max(Pts(:,1));
    y_size = y_max - y_min;
    y_start = y_min + ratio * y_size;
    y_end = y_max - ratio * y_size;
    y_box = (y_start < Pts(:, 2)) & (Pts(:, 2) < y_end);

    z_min = min(Pts(:,1));
    z_max = max(Pts(:,1));
    z_size = z_max - z_min;
    z_start = z_min + ratio * z_size;
    z_end = z_max - ratio * z_size;
    z_box = (z_start < Pts(:, 3)) & (Pts(:, 3) < z_end);

    box = x_box & y_box & z_box;
    Pts = Pts(box, :);
    colors = colors(box, :);
%}

    %% RANSACing to get rid of wall and table

    % add a 1 for the d coefficient
    Pts = [Pts, ones(size(Pts, 1), 1)];

    Total_RANSAC_iters = 2;
    for q = 1:Total_RANSAC_iters
        iter = 0;
        MaxIterations = 50;
        % number of total points
        numpts = size(Pts, 1);
        % threshold for number of inliers to likely be an irrelevant surface
        num_inlier_threshold = 0.55;
        % threshold for whether a point's distance is consider inlying
        inlier_threshold = 6;
        % keeps track of the largest possible set of points to remove (e.g.
        % the largest plane)
        bestinliers = 0;
        bestnuminliers = 0;
        %Every iteration will pick 4 points and find the equation for a plane
        while iter < MaxIterations && (bestnuminliers / numpts) < num_inlier_threshold
            %Get random values/points
            inds = randi(numpts, 4, 1);
            p1 = Pts(inds(1),1:3);
            p2 = Pts(inds(2),1:3);
            p3 = Pts(inds(3),1:3);
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
%	    f = @(p, pln) p * plane;
%	    inliers = bsxfun(f, Pts, plane);

            for ii = 1:numpts
                p = Pts(ii,:);
                inliers(ii) = (abs(p*plane) / divisor) < inlier_threshold;
            end


            numinliers = sum(inliers);
            if numinliers > bestnuminliers
                bestnuminliers = numinliers;
                bestinliers = inliers;
            end
            
            iter = iter + 1;
        end

	if iter == MaxIterations
	    disp('Warning: RANSAC failed to converge in 50 iterations');
	else
	    disp([int2str(iter), ' iterations']);
	end

        % Printing results
	if showImages
            figure;
            pcshow(Pts(:, 1:3), colors);
            drawnow;
            title(strcat('Before RANSAC', int2str(q)));
	end
        
        % Create best fit plane of bestinliers and then remove that plane
        [n, ~, p] = affine_fit(Pts(bestinliers,1:3));
        d = -dot(n, p);
        plane = [n(:);d];
        inliers = false(numpts, 1);
        divisor = norm(n);
        for ii = 1:numpts
	    p = Pts(ii, :);
            inliers(ii) = (abs(p*plane) / divisor) < inlier_threshold;
        end
        
        Pts = Pts(~inliers,:);
        colors = colors(~inliers,:);
        
	if showImages
            figure;
            pcshow(Pts(:, 1:3), colors);
            drawnow;
            title(strcat('After RANSAC', int2str(q)));
	end
    end
    Pts = Pts(:, 1:3);

end
