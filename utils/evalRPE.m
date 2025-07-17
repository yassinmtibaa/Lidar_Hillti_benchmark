function rpe = evalRPE(traj, segLen)
/% traj : N×8 array OR path to tum file
% segLen : segment length in metres (default 10)
%/
if ischar(traj), traj = readmatrix(traj); end
if nargin<2, segLen = 10; end
dist = vecnorm(diff(traj(:,2:4)),2,2);
cum  = cumsum([0; dist]);
idx  = find(cum >= segLen);
err  = traj(idx,2:4) - traj(1:end-idx,2:4);
rpe  = sqrt(mean(sum(err.^2,2))) / segLen;
end 