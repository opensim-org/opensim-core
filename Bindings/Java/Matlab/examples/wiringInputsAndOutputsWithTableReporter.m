% ----------------------------------------------------------------------- %
% The OpenSim API is a toolkit for musculoskeletal modeling and           %
% simulation. See http://opensim.stanford.edu and the NOTICE file         %
% for more information. OpenSim is developed at Stanford University       %
% and supported by the US National Institutes of Health (U54 GM072970,    %
% R24 HD065690) and by DARPA through the Warrior Web program.             %
%                                                                         %
% Copyright (c) 2005-2017 Stanford University and the Authors             %
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

% This example shows how to wire inputs and outputs by reporting the position
% of the system's center of mass. We also illustrate that input-output
% connections are stored in model (.osim) files.
% The model contains just one body, a free joint, and the table reporter.

import org.opensim.modeling.*;


modelFilename = 'wiring_inputs_and_outputs_with_TableReporter.osim';

%% Create and print the model to a .osim file.
% --------------------------------------------
model = Model();
model.setName('model');

% Create a body with name 'body', mass of 1 kg, center of mass at the
% origin of the body, and unit inertia (Ixx = Iyy = Izz = 1 kg-m^2).
body = Body('body', 1.0, Vec3(0), Inertia(1));

% Create a free joint (all 6 degrees of freedom) with Ground as the parent
% body and 'body' as the child body.
joint = FreeJoint('joint', model.getGround(), body);

% Add the body and joint to the model.
model.addComponent(body);
model.addComponent(joint);

% Create a TableReporter to save quantities to a file after simulating.
reporter = TableReporterVec3();
reporter.setName('reporter');
reporter.set_report_time_interval(0.1);

% Report the position of the origin of the body.
reporter.addToReport(body.getOutput('position'));
% For comparison, we will also get the center of mass position from the
% Model, and we can check that the two outputs are the same for our
% one-body system. The (optional) second argument is an alias for the name
% of the output; it is used as the column label in the table.
reporter.addToReport(model.getOutput('com_position'), 'com_pos');
% Display what input-output connections look like in XML (in .osim files).
disp('Reporter input-output connections in XML:');
disp(reporter.dump());

model.addComponent(reporter);

% We must finalize connections to save the input-output connections in the
% model file.
model.finalizeConnections();
model.print(modelFilename);


%% Load the model file and simulate.
% ----------------------------------
deserializedModel = Model(modelFilename);
state = deserializedModel.initSystem();

% We can fetch the TableReporter from within the deserialized model.
reporter = TableReporterVec3.safeDownCast(...
        deserializedModel.getComponent('reporter'));
% We can access the names of the outputs that the reporter is connected to.
disp('Outputs connected to the reporter:');
for i = 0:(reporter.getInput('inputs').getNumConnectees() - 1)
    disp(reporter.getInput('inputs').getConnecteePath(i));
end

% Simulate the model.
manager = Manager(deserializedModel);
state.setTime(0);
manager.initialize(state);
state = manager.integrate(1.0);

% Now that the simulation is done, get the table from the TableReporter and
% write it to a file.
% This returns the TimeSeriesTableVec3 that holds the history of positions.
table = reporter.getTable();
% Create a FileAdapter, which handles writing to (and reading from) .sto files.
sto = STOFileAdapterVec3();
sto.write(table, 'wiring_inputs_and_outputs_with_TableReporter.sto');
% You can open the .sto file in a text editor and see that both outputs
% (position of body's origin, and position of system mass center) are the same.

