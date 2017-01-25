% The OpenSim API is a toolkit for musculoskeletal modeling and
% simulation. See http://opensim.stanford.edu and the NOTICE file
% for more information. OpenSim is developed at Stanford University
% and supported by the US National Institutes of Health (U54 GM072970,
% R24 HD065690) and by DARPA through the Warrior Web program.
%
% Copyright (c) 2005-2016 Stanford University and the Authors
% Author(s): James Dunne
%
% Licensed under the Apache License, Version 2.0 (the "License");
% you may not use this file except in compliance with the License.
% You may obtain a copy of the License at
% http://www.apache.org/licenses/LICENSE-2.0.
%
% Unless required by applicable law or agreed to in writing, software
% distributed under the License is distributed on an "AS IS" BASIS,
% WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
% implied. See the License for the specific language governing
% permissions and limitations under the License.

function J = DesignMainStarter_Manager(initial_states)
% This script opens a Model, edits its initial state, runs a forward
% simulation. 
% initial_states = a State object written as Matlab vector whose entries... 
%                  are in a particular order

% Set a list of ordered state names that match the state values
coordinateNames = [{'LHIP'} {'RHIP'} {'LKnee'} {'Rknee'}];

% Import
import org.opensim.modeling.*

% Open a Model by name
model = Model('../Model/WalkerModel.osim');

% Get a reference to the underlying computational system
model_states = model.initSystem();

% Set the coordinate values from the initial_states
coordSet = model.getCoordinateSet();

for i = 1:length(coordinateNames)
    idx = coordSet.getIndex( coordinateNames{idx} );
    if (idx<0), error(['Coordiantes, ' coordinateNames{i} ', do not exist in the Model']); end
    coordSet.get(idx).setValue(model_states, initial_states(i));
end

% add a reporter
reporter = TableReporter();
reporter.set_report_time_interval(0.1)

% Add inputs to the reporter
reporter.updInput('inputs').connect( model.getCoordinateSet().get(1).getOutput('value'), 'Pelvis_tx')
reporter.updInput('inputs').connect( model.getCoordinateSet().get(3).getOutput('value'), 'Hip_Angle_L')
reporter.updInput('inputs').connect( model.getCoordinateSet().get(4).getOutput('value'), 'Hip_Angle_R')
reporter.updInput('inputs').connect( model.getCoordinateSet().get(5).getOutput('value'), 'Knee_Angle_L')
reporter.updInput('inputs').connect( model.getCoordinateSet().get(6).getOutput('value'), 'Knee_Angle_R')

% add the reporter to the model
model.addComponent(reporter)

% Simulate.
manager = Manager(model);
manager.setInitialTime(0);
manager.setFinalTime(3);
manager.integrate(model_states);

% get the table last value of pelvis X (distance down the run way)
table = reporter.getTable();
nRows = table.getNumRows();

% need to get close to 3 meters
J = abs(3 - table.getDependentColumnAtIndex(0).getElt(nRows-1,0)) ;

end
