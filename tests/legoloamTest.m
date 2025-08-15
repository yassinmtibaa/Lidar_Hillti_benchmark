classdef legoloamTest < matlab.unittest.TestCase
   methods (Test)
      function testIdentity(tc)
         pc = pointCloud(rand(1000,3));
         T  = pcregisterlegoloam(pc, pc, 'InitialTransform', rigidtform3d);
         tc.verifyLessThan(norm(T.Translation), 1e-2);
      end
   end
end


