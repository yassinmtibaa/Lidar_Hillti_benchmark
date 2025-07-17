classdef loamTest < matlab.unittest.TestCase
   methods (Test)
      function testRegisterPairIdentity(tc)
         pc = pointCloud(rand(1000,3));
         T  = internal.registerPair(pc, pc, rigidtform3d, 0.4);
         tc.verifyLessThan(norm(T.Translation), 1e-3);
      end
   end
end 