% Simulation_using_Matlab_Integration. 
% This script uses Uses Matlab based integrators to perform a simulation of an
% OpenSim Model. This would be useful if you prefer to use Matlab integration.
%
% This script opens a Model, edits its initial state, runs a forward
% simulation with the built-in Matlab integrators, and creates a plot from
% the motion data.

% -----------------------------------------------------------------------
% The OpenSim API is a toolkit for musculoskeletal modeling and
% simulation. See http://opensim.stanford.edu and the NOTICE file
% for more information. OpenSim is developed at Stanford University
% and supported by the US National Institutes of Health (U54 GM072970,
% R24 HD065690) and by DARPA through the Warrior Web program.
%
% Copyright (c) 2005-2019 Stanford University and the Authors
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

% Import
import org.opensim.modeling.*;

% Open a Model by name
osimModel = Model('../Model/WalkerModelTerrain.osim');

% Use the visualizer (must be done before the call to init system)
osimModel.setUseVisualizer(true);

% Initialize the system and get the initial state
osimState = osimModel.initSystem();

% Set the initial states of the model
editableCoordSet = osimModel.updCoordinateSet();
editableCoordSet.get('Pelvis_ty').setValue(osimState, 1.1);
editableCoordSet.get('Pelvis_tx').setSpeedValue(osimState, 1.0);

% Recalculate the derivatives after the coordinate changes
osimModel.computeStateVariableDerivatives(osimState);

% The base model has no controls
controlsFuncHandle = [];

% Integrate plant using Matlab Integrator
timeSpan = [0 2];
integratorName = 'ode15s';
integratorOptions = odeset('AbsTol', 1E-5');

% Run Simulation
motionData = IntegrateOpenSimPlant(osimModel, controlsFuncHandle, timeSpan, ...
    integratorName, integratorOptions);

% Plot Results
[figHandle, axisHandle] = PlotOpenSimData([],motionData, 'time', ...
    {'Pelvis_tx', 'RHip_rz', 'RKnee_rz'});
