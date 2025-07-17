function [normals, points_fixed, points_moving] = findPlanarFeatures(fixed, moving, movingOrig, n_max_features)
%FINDPLANARFEATURES Dummy planar feature matcher for LOAM registration.
%   For demonstration, randomly samples points and assigns dummy normals.
%   Replace with real planar feature extraction for production use.

maxIdx = min(size(fixed,1), size(moving,1));
N = min([maxIdx, n_max_features]);
if N < 1
    normals = [];
    points_fixed = [];
    points_moving = [];
    return;
end
idx = randperm(maxIdx, N);
points_fixed = fixed(idx, :);
points_moving = moving(idx, :);
normals = repmat([0 0 1], N, 1); % Dummy normals (z-up)
end 