function scans = readHiltiBag(bagPath, topic)
% READHILTIBAG  Convert ROS 1/2 bag to organised pointCloud cells.
%   scans = readHiltiBag("data/site3_handheld_1.bag","/hesai/pandar")
%   Returns {N×1 cell} of organised pointCloud for LOAM-style use.
%   Uses Pandar64 (64 rings) by default. Adjust 'rings' for other models.

arguments
    bagPath (1,:) char
    topic   (1,:) char = "/hesai/pandar"
end

% --- choose reader automatically ---------------------------------------
[~,~,ext] = fileparts(bagPath);
if strcmpi(ext, ".bag")
    bag = rosbagreader(bagPath);   % ROS 1
else
    bag = ros2bagreader(bagPath);  % ROS 2 (.db3/.mcap)
end

sel  = select(bag, "Topic", topic);
msgs = readMessages(sel, "DataFormat", "struct");
assert(~isempty(msgs), "Topic not found or empty bag");

% --- sensor model (Pandar64 default) -----------------------------------
rings = 64;  % Change to 40 for Pandar40
verticalBeamAngles = [15 -16];  % [min max] degrees
lp = lidarParameters(rings, verticalBeamAngles, 360);  % (verticalResolution, verticalFoV, horizontalResolution)

% --- convert all scans efficiently -------------------------------------
n = numel(msgs);
scans = cell(1, n);
parfor k = 1:n
    xyz = rosReadXYZ(msgs{k});
    scans{k} = pcorganize(pointCloud(xyz), lp);
end
end
