% DesignMainStarter
% This script demonstrates running a Forward Simulation using the OpenSim
% Manager Class. The OpenSim Manager is available from OpenSim 4.0, onwards.

% -----------------------------------------------------------------------
% The OpenSim API is a toolkit for musculoskeletal modeling and
% simulation. See http://opensim.stanford.edu and the NOTICE file
% for more information. OpenSim is developed at Stanford University
% and supported by the US National Institutes of Health (U54 GM072970,
% R24 HD065690) and by DARPA through the Warrior Web program.
%
% Copyright (c) 2005-2019 Stanford University and the Authors
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
% -----------------------------------------------------------------------

%% Set some properties for the simulation
endTime   = 2;
visualize = true;
plotResults = true;

%% Define the intial coordinate values and speeds for the model. Translations
% coordinates are in meters, rotations are in radians.
pelvisTYValue = 0.8350;
pelvisTYSpeed = 0;
pelvisTXValue = 0;
pelvisTXSpeed  = 0;
rHipValue = deg2rad(30.0);
rHipSpeed = deg2rad(0);
lHipValue = deg2rad(-10);
lHipSpeed = deg2rad(0);
rKneeValue = deg2rad(-30.0);
rKneeSpeed = deg2rad(0);
rKneeValue = deg2rad(-30.0);
rKneeSpeed = deg2rad(0);

%% Import OpenSim Libraries
import org.opensim.modeling.*;

%% Define the Model File Path.
% The default is a relative path from the working directory for the example
model_path = '../Model/WalkerModelTerrain.osim';

%% Instantiate the Model
osimModel = Model(model_path);

% Set the visualizer use.
osimModel.setUseVisualizer(visualize)

% Initialize the underlying computational system and get a reference to
% the system state.
state = osimModel.initSystem();

%% Set the Vizualizer parameters
if visualize
    sviz = osimModel.updVisualizer().updSimbodyVisualizer();
    sviz.setShowSimTime(true);
    % Show "ground and sky" background instead of just a black background.
    sviz.setBackgroundTypeByInt(1);
    % Set the default ground height down so that the walker platform is
    % viewable.
    sviz.setGroundHeight(-10)
    % Set the initial position of the camera
    sviz.setCameraTransform(Transform(Rotation(),Vec3(1,0,5)));
    % Show help text in the visualization window.
    help = DecorativeText('ESC or CMD-Q to quit.');
    help.setIsScreenText(true);
    sviz.addDecoration(0, Transform(Vec3(0, 0, 0)), help);
end

%% Change the initial Coordinate values and speeds of the model
osimModel.updCoordinateSet().get('Pelvis_ty').setValue(state, pelvisTYValue);
osimModel.updCoordinateSet().get('Pelvis_ty').setLocked(state, pelvisTYSpeed);
osimModel.updCoordinateSet().get('Pelvis_tx').setValue(state, pelvisTXValue);
osimModel.updCoordinateSet().get('Pelvis_tx').setLocked(state, pelvisTXSpeed);
osimModel.updCoordinateSet().get('RHip_rz').setValue(state, rHipValue);
osimModel.updCoordinateSet().get('RHip_rz').setLocked(state, rHipSpeed);
osimModel.updCoordinateSet().get('LHip_rz').setValue(state, lHipValue);
osimModel.updCoordinateSet().get('LHip_rz').setLocked(state, lHipSpeed);
osimModel.updCoordinateSet().get('RKnee_rz').setValue(state, rKneeValue);
osimModel.updCoordinateSet().get('RKnee_rz').setLocked(state, rKneeSpeed)
osimModel.updCoordinateSet().get('LKnee_rz').setValue(state, rKneeValue);
osimModel.updCoordinateSet().get('LKnee_rz').setLocked(state, rKneeSpeed);

%% Run a fwd simulation using the manager
manager = Manager(osimModel);
state.setTime(0);
manager.initialize(state);
state = manager.integrate(endTime);

%% Get the states table from the manager and print the results.
sTable = manager.getStatesTable();
stofiles = STOFileAdapter();
if ~isdir('ResultsFWD')
    mkdir ResultsFWD
end
stofiles.write(sTable, 'ResultsFWD/simulation_states.sto');

%% Use the provided plotting function to plot some results.
if plotResults
    PlotOpenSimData;
end

%% Display Messages
display('Forward Tool Finished.');
display('Output files were written to the /Results/FWD directory:')
