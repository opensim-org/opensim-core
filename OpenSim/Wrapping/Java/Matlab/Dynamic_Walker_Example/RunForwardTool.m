% ----------------------------------------------------------------------- 
% The OpenSim API is a toolkit for musculoskeletal modeling and           
% simulation. See http://opensim.stanford.edu and the NOTICE file         
% for more information. OpenSim is developed at Stanford University       
% and supported by the US National Institutes of Health (U54 GM072970,    
% R24 HD065690) and by DARPA through the Warrior Web program.             
%                                                                         
% Copyright (c) 2005-2013 Stanford University and the Authors             
% Author(s): Daniel A. Jacobs                                             
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
% This script demonstrates calls the Forward Tool from the OpenSim library from Matlab.
% ----------------------------------------------------------------------- 
import org.opensim.modeling.*

% Open Model
walkerModel = Model('../Model/DW2013_WalkerModelTerrain.osim');

% Add Analyses to the Model
forceReporter = ForceReporter();
walkerModel.addAnalysis(forceReporter);

% Initialize the underlying computational system
state = walkerModel.initSystem();

% Create the Forward Tool
tool = ForwardTool();

% Set the model for the forward tool
tool.setModel(walkerModel);

% Define the start and finish times 
tool.setStartTime(0);
tool.setFinalTime(2);

% Define the prefix for the result files
tool.setName('DW2013_WalkerModelTerrain');

% Set Input States File
tool.setStatesFileName('../Model/DW2013_WalkerModelTerrain_Initial_states.sto');

% Set Results Directory (will create without prompt)
tool.setResultsDir('../Results/FWD');

% Run the simulation
statusVal = tool.run();

% Cleanup
% clearvars walkerModel forceReporter tool state statusVal
display('Forward Tool Finished.');
display('The following files were written to the /Results/FWD directory:')
display('DW2013_WalkerModelTerrain_states.sto')
display('DW2013_WalkerModelTerrain_controls.sto');
display('DW2013_WalkerModelTerrain_states_degrees.mot');
display('DW2013_WalkerModelTerrain_ForceReporter_forces.sto');