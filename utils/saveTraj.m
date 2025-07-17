%--------------------------------------------------------------------
% File        : utils/saveTraj.m
% Purpose     : Save trajectory to file
% Author      : P2 <your-email>
% Created     : 2025-07-17
% Dependencies: None
%--------------------------------------------------------------------
function saveTraj(fname, traj)
/% traj = N×8  [t x y z qw qx qy qz] %/
[fpath,~,~] = fileparts(fname);
if ~isfolder(fpath), mkdir(fpath); end
writematrix(traj,fname,"Delimiter"," ");
end
