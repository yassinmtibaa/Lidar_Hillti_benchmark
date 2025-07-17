%--------------------------------------------------------------------
% File        : algorithms/loam/+internal/buildTrajRow.m
% Purpose     : Build a trajectory row from pose
% Author      : P2 <your-email>
% Created     : 2025-07-17
% Dependencies: None
%--------------------------------------------------------------------

function row = buildTrajRow(T, t)
arguments
    T rigidtform3d
    t double
end
% TODO: Implement trajectory row building
row = [t, T.Translation, T.Rotation];
end 