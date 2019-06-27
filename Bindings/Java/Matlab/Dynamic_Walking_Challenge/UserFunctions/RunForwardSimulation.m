% -----------------------------------------------------------------------
% The OpenSim API is a toolkit for musculoskeletal modeling and
% simulation. See http://opensim.stanford.edu and the NOTICE file
% for more information. OpenSim is developed at Stanford University
% and supported by the US National Institutes of Health (U54 GM072970,
% R24 HD065690) and by DARPA through the Warrior Web program.
%
% Copyright (c) 2005-2019 Stanford University and the Authors
% Author(s): James Dunne, Daniel A. Jacobs.
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
% -----------------------------------------------------------------------
% This script demonstrates calling the Forward Tool from the OpenSim library from Matlab.
% ----------------------------------------------------------------------- 

import org.opensim.modeling.*

% Open Model
model= Model('../Model/WalkerModel.osim');

% Add Analyses to the Model
forceReporter = ForceReporter();
model.addAnalysis(forceReporter);

% Initialize the underlying computational system
state = model.initSystem();

% Run a fwd simulation using the manager
state = model.initSystem();
manager = Manager(model);
state.setTime(0);
manager.initialize(state);
state = manager.integrate(2);

% Get the states table from the manager and print the results.
sTable = manager.getStatesTable();
stofiles = STOFileAdapter();
stofiles.write(sTable, '../Results/fwd/simulation_states.sto');

% Print the force reporter results to file (_ForceReporter_Forces.sto)
forceReporter.printResults('','../Results/fwd',-1, '.sto');

% Cleanup
% clearvars walkerModel forceReporter tool state statusVal
display('Forward Tool Finished.');
display('Output files were written to the /Results/FWD directory:')

