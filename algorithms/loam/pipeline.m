%--------------------------------------------------------------------
% File        : algorithms/loam/pipeline.m
% Purpose     : End-to-end LOAM odometry on a ROS bag
% Author      : P2 <your-email>
% Created     : 2025-07-17
% Dependencies: utils/readHiltiBag, lidarToolbox R2024b
%--------------------------------------------------------------------

function traj = pipeline(bagFile, topic, cfg)
arguments
    bagFile (1,:) char
    topic   (1,:) char = "/hesai/pandar"
    cfg.GridStep   double = 0.4   % down-sample size
    cfg.InitGuess  rigidtform3d = rigidtform3d
end
%#codegen  <-- remove if not compiling

tic
scans = readHiltiBag(bagFile, topic);
N     = numel(scans);

traj        = zeros(N,8);
traj(1,5:8) = [1 0 0 0];                % identity quaternion
Tprev       = rigidtform3d;

for k = 2:N
    T = internal.registerPair(scans{k}, scans{k-1}, ...
            "InitialTransform",Tprev,"GridStep",0.4);
    traj(k,:) = accumulatePose(traj(k-1,:), T);
    Tprev = T;
    if mod(k,300)==0
        fprintf("Frame %d / %d (%.1f%%)\n",k,N,100*k/N);
    end
end

traj(:,1) = (0:N-1).' * 0.1;            % assume 10 Hz

outFile = fullfile("results", ...
          replace(bagFile,["\",".bag",".db3"],["_traj_loam",".tum",".tum"]));
saveTraj(outFile,traj);
fprintf("LOAM finished → %s\n",outFile);

fprintf("LOAM Hz = %.1f\n", numel(scans)/toc);
end
