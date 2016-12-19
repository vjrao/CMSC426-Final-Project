function [Pts, colors] = ICP (Pts, Pts2)

    ptCloud = pointCloud(Pts);
    normals = pcnormals(ptCloud, 10);
    
    while 
    M = transf_mat(normals, Pts, Pts2);
    Pts = [Pts, ones(numpts, 1)];
    Pts2 = [Pts2, ones(numpts, 1)];
    Pts3 = zeros(size(Pts));
    for ii=1:numpts
        Pts3(ii,:) = M * Pts2(ii, :)';
    end
    
    Pts = pcmerge(ptCloud, pointCloud(Pts3(:,1:3)), 2);	 

end
