%% Segmentation
%Take normals and filter over it (say 3x3) and average the dot product of
%all the unit vectors within the filter with the center point. If the end
%result is below a certain threshold, classify as a concavity