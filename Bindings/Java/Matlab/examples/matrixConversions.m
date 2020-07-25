% ----------------------------------------------------------------------- %
% The OpenSim API is a toolkit for musculoskeletal modeling and           %
% simulation. See http://opensim.stanford.edu and the NOTICE file         %
% for more information. OpenSim is developed at Stanford University       %
% and supported by the US National Institutes of Health (U54 GM072970,    %
% R24 HD065690) and by DARPA through the Warrior Web program.             %
%                                                                         %
% Copyright (c) 2005-2020 Stanford University and the Authors             %
% Author(s): Christopher Dembia                                           %
%                                                                         %
% Licensed under the Apache License, Version 2.0 (the "License");         %
% you may not use this file except in compliance with the License.        %
% You may obtain a copy of the License at                                 %
% http://www.apache.org/licenses/LICENSE-2.0.                             %
%                                                                         %
% Unless required by applicable law or agreed to in writing, software     %
% distributed under the License is distributed on an "AS IS" BASIS,       %
% WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or         %
% implied. See the License for the specific language governing            %
% permissions and limitations under the License.                          %
% ----------------------------------------------------------------------- %

% This example shows how to convert Matlab matrices to and from OpenSim's Vector
% and Matrix classes.

import org.opensim.modeling.*;

% Create an OpenSim Vec3 from a Matlab matrix.
matrix = [5, 3, 6];
osimVec3 = Vec3.createFromMat(matrix);
disp(osimVec3);
% Convert the OpenSim Vec3 back to a Matlab matrix.
matrix2 = osimVec3.getAsMat();
disp(matrix2);

% Create an OpenSim Vector from a Matlab matrix.
matrix = [5, 3, 6, 2, 9];
osimVector = Vector.createFromMat(matrix);
disp(osimVector);
% Convert the OpenSim Vector back to a Matlab matrix.
matrix3 = osimVector.getAsMat();
disp(matrix3);

% Same for RowVector.
osimRowVector = RowVector.createFromMat(matrix);
disp(osimRowVector);
matrix4 = osimRowVector.getAsMat()
disp(matrix4);

% VectorVec3.
mat = [1, 2, 3; 4, 5, 6; 7, 8, 9; 10, 11, 12];
osimVV3 = VectorVec3.createFromMat(mat);
disp(osimVV3);
matrix5 = osimVV3.getAsMat();
disp(matrix5);

% RowVectorVec3.
osimRVV3 = RowVectorVec3.createFromMat(mat');
disp(osimRVV3);
matrix6 = osimRVV3.getAsMat();
disp(matrix6);

% Matrix.
matrix = [5, 3; 3, 6; 8, 1];
osimMatrix = Matrix.createFromMat(matrix);
disp(osimMatrix);
matrix7 = osimMatrix.getAsMat();
disp(matrix7);

