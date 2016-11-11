% ----------------------------------------------------------------------- 
% The OpenSim API is a toolkit for musculoskeletal modeling and           
% simulation. See http://opensim.stanford.edu and the NOTICE file         
% for more information. OpenSim is developed at Stanford University       
% and supported by the US National Institutes of Health (U54 GM072970,    
% R24 HD065690) and by DARPA through the Warrior Web program.             
%                                                                         
% Copyright (c) 2005-2014 Stanford University and the Authors             
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
function status = CallForwardTool(modelDirectory, ...
    modelFile, resultsDirectory, initialStatesFile)
import org.opensim.modeling.*

    % Open Model
    walkerModel = Model([modelDirectory, modelFile]);

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
    resultFilePrefix = walkerModel.getName();
    tool.setName(resultFilePrefix);

    % Set Input States File
    tool.setStatesFileName([modelDirectory, initialStatesFile]);

    % Set Results Directory (will create without prompt)
    tool.setResultsDir(resultsDirectory);

    % Run the simulation
    status = tool.run();

    fprintf('The following files were written to %s:\n', resultsDirectory);
    fprintf('\t%s_states.sto\n',resultFilePrefix.toCharArray());
    fprintf('\t%s_controls.sto\n',resultFilePrefix.toCharArray());
    fprintf('\t%s_states_degrees.mot\n',resultFilePrefix.toCharArray());
    fprintf('\t%s_ForceReporter_forces.sto\n',resultFilePrefix.toCharArray());
    fprintf('Forward Tool Finished.\n');
    
end