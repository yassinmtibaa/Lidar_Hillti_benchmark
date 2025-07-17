%--------------------------------------------------------------------
% File        : algorithms/loam/+internal/registerPair.m
% Purpose     : Register two point clouds using LOAM
% Author      : P2 <your-email>
% Created     : 2025-07-17
% Dependencies: lidarToolbox R2024b
%--------------------------------------------------------------------

%% registerPair
%  Align two organised pointCloud scans using edge-&-plane cost.
%
%  T = internal.registerPair(srcCloud, tgtCloud, initT, gridStep)
%
%  Inputs
%  ------
%  srcCloud  pointCloud   » current frame
%  tgtCloud  pointCloud   » previous frame
%  initT     rigidtform3d » initial estimate
%  gridStep  double       » down-sample size [m]
%
%  Output
%  ------
%  T         rigidtform3d  optimal SE(3) transform
%
%  Example
%  -------
%     T = internal.registerPair(pc2, pc1, rigidtform3d, 0.4);
%
function T = registerPair(srcCloud, tgtCloud, initT, gridStep)
// ... existing code from pcregisterloam.m, replacing function name and references ... 