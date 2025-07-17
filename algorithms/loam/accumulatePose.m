function row = accumulatePose(prevRow, T)
%ACCUMULATEPOSE Updates trajectory row given previous pose and rigidtform3d.
%   prevRow: [t x y z qw qx qy qz]
%   T: rigidtform3d object

% Convert previous pose to rigidtform3d
qPrev = prevRow(5:8);
pPrev = prevRow(2:4);
Rprev = quat2rotm(qPrev);
Tprev = rigidtform3d(Rprev, pPrev);

% Compose transforms using transformation matrices
TnewMat = Tprev.A * T.A;
Rnew = TnewMat(1:3,1:3);
pNew = TnewMat(1:3,4)';
qNew = rotm2quat(Rnew);

row = [0, pNew, qNew]; % t will be set outside
end 