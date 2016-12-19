function [Pts] = ICP (Pts, Pts2, varargin)
	 
    if nargin >= 3
        iters = varargin{1};
    else
	iters = 10;
    end

    numpts = size(Pts, 1);
    ptCloud = pointCloud(Pts);
    normals = pcnormals(ptCloud, 20);
    Pts2 = [Pts2, ones(numpts, 1)];
    
    for iter=1:iters
        M = transf_mat(normals, Pts, Pts2(:, 1:3));
        for ii=1:numpts
            Pts2(ii,:) = M * Pts2(ii, :)';
	end
    end
    
    Pts = pcmerge(ptCloud, pointCloud(Pts2(:,1:3)), 2);	 

end
