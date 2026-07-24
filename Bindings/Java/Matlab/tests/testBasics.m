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

% Run a forward simulation and retreive the StatesTrajectory.
manager = Manager(model);
manager.setRecordStatesTrajectory(true);
manager.initialize(state);
manager.integrate(5.0);
statesTraj = manager.getStatesTrajectory();

% Check initial time.
initialState = statesTraj.get(0);
assert(initialState.getTime() == 0);
assert(initialState.getQ().size() == 2);
assert(initialState.getU().size() == 2);

% Check final time.
finalState = statesTraj.get(statesTraj.getSize()-1);
assert(finalState.getTime() == 5.0);
assert(finalState.getQ().size() == 2);
assert(finalState.getU().size() == 2);

% Create a StatesDocument to serialize the trajectory.
doc = statesTraj.exportToStatesDocument(model);
doc.serialize('pendulum.ostates');

% Deserialize and check equality.
statesTrajDeserialized = ...
    StatesTrajectory.createFromStatesDocument(model, 'pendulum.ostates');
assert(statesTraj.getSize() == statesTrajDeserialized.getSize());
for i = 0:statesTraj.getSize()-1
    state = statesTraj.get(i);
    stateDeserialized = statesTrajDeserialized.get(i);
    assert(state.getTime() == stateDeserialized.getTime());
end
