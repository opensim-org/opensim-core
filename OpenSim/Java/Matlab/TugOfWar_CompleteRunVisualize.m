% ----------------------------------------------------------------------- %
% The OpenSim API is a toolkit for musculoskeletal modeling and           %
% simulation. See http://opensim.stanford.edu and the NOTICE file         %
% for more information. OpenSim is developed at Stanford University       %
% and supported by the US National Institutes of Health (U54 GM072970,    %
% R24 HD065690) and by DARPA through the Warrior Web program.             %
%                                                                         %   
% Copyright (c) 2005-2012 Stanford University and the Authors             %
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

% This script runs a forward simulation with the tug of war model created
% in the script OpenSimCreateTugOfWarModel.

% Rotate the view window to see the simulation run.

% First, import the classes from the jar file so that these can be called
% directly
import org.opensim.modeling.*

% Generate a new model object by loading the tug of war model from file
osimModel = Model('tug_of_war_muscles_controller.osim');

% Set up the visualizer to show the model and simulation
osimModel.setUseVisualizer(true);

% Initializing the model
osimModel.initSystem();

%% DEFINE SOME PARAMETERS FOR THE SIMULATION

% Define the new tool object which will be run
tool = ForwardTool(); %--> to see all properties which can be set type 'methodsview(tool)'

% Define the model which the forward tool will operate on
tool.setModel(osimModel);

% Define the start and finish times for simulation
tool.setStartTime(0);
tool.setFinalTime(3);
tool.setSolveForEquilibrium(true);

% Define the name of the forward analysis
tool.setName('tugOfWar');

% Run the simulation
tool.run();  %--> rotate the view to see the tug of war simulation

