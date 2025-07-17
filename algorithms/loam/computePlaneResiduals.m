function [J, residuals] = computePlaneResiduals(normals, points_fixed, points_moving, theta)
%COMPUTEPLANERESIDUALS Dummy residual/Jacobian for LOAM registration.
%   For demonstration, computes point-to-plane residuals and a dummy Jacobian.
%   Replace with real Jacobian for production use.

if isempty(normals) || isempty(points_fixed) || isempty(points_moving)
    J = zeros(0, 6);
    residuals = zeros(0, 1);
    return;
end
N = size(points_fixed,1);
residuals = sum((points_moving - points_fixed) .* normals, 2);
J = zeros(N, 6); % Dummy Jacobian (should be replaced with real one)
end 