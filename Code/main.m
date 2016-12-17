Path = '../Data/SingleObject/';
scenes = [0, 1, 2, 6, 8, 12, 22, 23];
% note: the frames are named frame_num_(rgb/depth).png starting from 0
% subtract 2 for the . and .. directories and

scene = 1;
numframes = length(dir()) / 2 - 2;
for ii=0:numframes
    % the filename "image" is specific to scene 001
    name = strcat(Path,'scene_',sprintf('%0.3d',scenes(scene)), '/frames/image_', int2str(ii));
    [Pts, colors] = RANSAC(name);
end
