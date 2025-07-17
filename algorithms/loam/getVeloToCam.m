function T = getVeloToCam()
%GETVELOTOCAM Returns the LiDAR-to-camera calibration transform.
%   By default, returns the identity matrix (no transform).
%   Replace with actual calibration if available.
T = eye(4);
end 