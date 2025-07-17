function R = getR(rx, ry, rz)
%GETR Returns a rotation matrix from Euler angles (ZYX order)
%   rx, ry, rz: rotation angles in radians

Rx = [1 0 0; 0 cos(rx) -sin(rx); 0 sin(rx) cos(rx)];
Ry = [cos(ry) 0 sin(ry); 0 1 0; -sin(ry) 0 cos(ry)];
Rz = [cos(rz) -sin(rz) 0; sin(rz) cos(rz) 0; 0 0 1];
R = Rz * Ry * Rx;
end 