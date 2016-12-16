% Setup Paths and Read RGB and Depth Images
Path = '../Data/SingleObject/'; 
SceneNum = 0;
SceneName = sprintf('%0.3d', SceneNum);
FrameNum = num2str(1);

[Pts, colors] = RANSAC([Path,'scene_',SceneName,'/frames/frame_',FrameNum]);