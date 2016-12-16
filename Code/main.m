scenes = {'000'; '001'; '002'; '006'; '008'; '012'; '022'; '023'};
% note: the frames are named frame_num_(rgb/depth).png starting from 0
% subtract 2 for the . and .. directories and 
numframes = length(dir(strcat('../Data/SingleObject/scene_', scenes{2}, '/frames'))) / 2 - 2;
