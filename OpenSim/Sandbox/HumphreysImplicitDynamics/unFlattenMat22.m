function [matOut,muscle]=unFlattenMat22(matIn,muscle)

import org.opensim.modeling.*

b=muscle.flattenMat22(matIn);

for i=0:3
   matOut(i+1)=b.get(i); 
end

matOut=reshape(matOut,2,2)';