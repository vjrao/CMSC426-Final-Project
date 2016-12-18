function [Pts, colors] = denoise(Pts, colors, varargin)

    if nargin > 2
        showImages = varargin{1};
    else
        showImages = false;
    end

    if showImages
        figure;
	pcshow(Pts, colors);
	drawnow;
	title('Before remove noise');
    end
    
    m = mean(Pts);
    s = std(Pts) * 3;
    xbox = (m(1) - s(1) < Pts(:, 1)) & (Pts(:, 1) < m(1) + s(1));
    ybox = (m(2) - s(2) < Pts(:, 2)) & (Pts(:, 2) < m(2) + s(2));
    zbox = (m(3) - s(3) < Pts(:, 3)) & (Pts(:, 3) < m(3) + s(3));
    inbox = xbox & ybox & zbox;
    Pts = Pts(inbox, :);
    colors = colors(inbox, :);

    ptCloud = pointCloud(Pts);
    [Pts,in,~] = pcdenoise(ptCloud, 'Threshold', 1, 'NumNeighbors', 200);
    Pts = Pts.Location;
    colors = colors(in, :);


    if max(max(Pts) - min(Pts)) > 300
        disp('running kmeans for additional noise removal');
        idx = kmeans(Pts, 2);
	if sum(idx == 1) > numel(idx) / 2
	    keep = 1;
	else
	    keep = 2;
	end
	keep = idx == keep;
	Pts = Pts(keep, :);
	colors = colors(keep, :);
    end
    
    if showImages
        figure;
	pcshow(Pts, colors);
	drawnow;
	title('After remove noise');
    end
end
