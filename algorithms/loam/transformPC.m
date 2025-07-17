function ptsOut = transformPC(pts, T)
%TRANSFORMPC Applies a 4x4 transformation to a set of 3D points.
%   pts: 3xN or N×3 array of points
%   T:   4x4 transformation matrix
%   ptsOut: transformed points, same shape as input

if size(pts,1) == 3  % 3xN
    ptsH = [pts; ones(1, size(pts,2))];
    ptsOutH = T * ptsH;
    ptsOut = ptsOutH(1:3, :);
elseif size(pts,2) == 3  % N×3
    ptsH = [pts, ones(size(pts,1),1)]';
    ptsOutH = T * ptsH;
    ptsOut = ptsOutH(1:3, :)';
else
    error('Input must be 3xN or N×3');
end
end 