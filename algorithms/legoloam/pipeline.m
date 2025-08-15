function traj = pipeline(bagFile, topic)

if nargin<2, topic="/hesai/pandar"; end
scans = readHiltiBag(bagFile, topic);
N     = numel(scans);

if isempty(scans) || N < 2
    error('Not enough scans for registration.');
end

traj        = zeros(N,8);
traj(1,5:8) = [1 0 0 0];                % identity quaternion
Tprev       = rigidtform3d;

for k = 2:N
    try
        T = pcregisterlegoloam(scans{k}, scans{k-1}, ...
                "InitialTransform",Tprev,"GridStep",0.4);
        traj(k,:) = accumulatePose(traj(k-1,:), T);
        Tprev = T;
    catch ME
        warning('LeGO-LOAM registration failed at frame %d: %s', k, ME.message);
        traj(k,:) = traj(k-1,:); % Repeat previous pose
        Tprev = rigidtform3d;    % Reset transform
    end
    if mod(k,300)==0
        fprintf("[LeGO-LOAM] Frame %d / %d (%.1f%%)\n",k,N,100*k/N);
    end
end

traj(:,1) = (0:N-1).' * 0.1;            % assume 10 Hz

outFile = fullfile("results", ...
          replace(bagFile,["\\",".bag",".db3"],["_traj_legoloam",".tum",".tum"]));
saveTraj(outFile,traj);
fprintf("LeGO-LOAM finished → %s\n",outFile);
end


