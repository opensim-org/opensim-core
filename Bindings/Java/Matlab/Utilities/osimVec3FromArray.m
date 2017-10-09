%% osimVec3FromArray converts 1x3 Matlab vector to OpenSim Vec3()
% Input  = 1x3 Matlab matrix
% Output = OpenSim Vec3()

% ----------------------------------------------------------------------- %
% The OpenSim API is a toolkit for musculoskeletal modeling and           %
% simulation. See http://opensim.stanford.edu and the NOTICE file         %
% for more information. OpenSim is developed at Stanford University       %
% and supported by the US National Institutes of Health (U54 GM072970,    %
% R24 HD065690) and by DARPA through the Warrior Web program.             %
%                                                                         %
% Copyright (c) 2005-2016 Stanford University and the Authors             %
% Author(s): James Dunne                                                  %
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
function vec = osimVec3FromArray(p)
% import Java Libraries
import org.opensim.modeling.*
if strcmp(class(p),'double')
    % Check the size of the input vector is 1x3.
    if size(p,1) ~= 1 | size(p,2) ~= 3
        error('Input vector needs to be a 1x3 sized vector')
    end
    % Convert the input vector to an OpenSim Vec3.
    vec = Vec3( p(1), p(2), p(3) );
else
    error('Incorrect class input. Must be type double') 
end
