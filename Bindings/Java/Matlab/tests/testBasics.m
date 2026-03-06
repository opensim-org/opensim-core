% -------------------------------------------------------------------------- %
%                          OpenSim:  testBasics.m                            %
% -------------------------------------------------------------------------- %
% The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  %
% See http://opensim.stanford.edu and the NOTICE file for more information.  %
% OpenSim is developed at Stanford University and supported by the US        %
% National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    %
% through the Warrior Web program.                                           %
%                                                                            %
% Copyright (c) 2026 Stanford University and the Authors                     %
% Author(s): Nicholas Bianco                                                 %
%                                                                            %
% Licensed under the Apache License, Version 2.0 (the "License"); you may    %
% not use this file except in compliance with the License. You may obtain a  %
% copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         %
%                                                                            %
% Unless required by applicable law or agreed to in writing, software        %
% distributed under the License is distributed on an "AS IS" BASIS,          %
% WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   %
% See the License for the specific language governing permissions and        %
% limitations under the License.                                             %
% -------------------------------------------------------------------------- %
import org.opensim.modeling.*;

% Test STL typemaps.
% ------------------
vec = StdVectorDouble();
for i = 0:4
    vec.add(i);
end
assert(vec.size() == 5);
for i = 0:vec.size() - 1
    value = vec.get(i);
    assert(value == i);
    assert(string(class(value)) == 'double');
end

% Test StatesTrajectory and StatesDocument.
% -----------------------------------------
model = ModelFactory.createDoublePendulum();
state = model.initSystem();

stateTraj = StatesTrajectory();
stateTraj.append(state);
assert(stateTraj.getSize() == 1);

doc = stateTraj.exportToStatesDocument(model);
doc.serialize('test.ostates');