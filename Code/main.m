scenes = {'000'; '001'; '002'; '006'; '008'; '012'; '022'; '023'};
% note: the frames are named frame_num_(rgb/depth).png starting from 0
% subtract 2 for the . and .. directories and 

scene = scenes{1};
numframes = length(dir(strcat('../Data/SingleObject/scene_', scenes{2}, '/frames'))) / 2 - 2;
for ii=0:numframes
    % the filename "image" is specific to scene 001
    name = strcat('../Data/SingleObject/scene_', scenes{2}, '/frames/image_', int2str(ii));
    [Pts, colors] = RANSAC(name);
    
end
