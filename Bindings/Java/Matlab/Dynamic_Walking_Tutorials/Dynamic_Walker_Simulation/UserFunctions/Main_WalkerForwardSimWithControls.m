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

%   This script opens a Model, edits its initial state, runs a forward
% simulation with the built-in Matlab integrators, and creates a plot from
% the motion data.  The model has an coordinate actuator on the left knee
% and a position controller to set the angle to -135 degrees.

% Make sure to run the AddCoordinateActuator.m script first!

% Import
import org.opensim.modeling.*;

% Open a Model by name
% display('Make sure you run the script file "AddCoordinateActuator.m" first.')
model = Model('../Model/WalkerModelTerrain_CoordAct.osim');

nActuators = model.getForceSet().getSize();

% Use the visualizer (must be done before the call to init system)
model.setUseVisualizer(true);

% Get a reference to the underlying computational system
osimState = model.initSystem();

% Set the initial states of the model
CoordSet = model.getCoordinateSet();
CoordSet.get('Pelvis_ty').setValue(osimState, 1.5);
CoordSet.get('Pelvis_ty').setLocked(osimState, true);
CoordSet.get('Pelvis_tx').setValue(osimState, 0);
CoordSet.get('Pelvis_tx').setLocked(osimState, true);
CoordSet.get('RHip_rz').setValue(osimState, 0);
CoordSet.get('RHip_rz').setLocked(osimState, true);
CoordSet.get('RKnee_rz').setValue(osimState, 0);
CoordSet.get('RKnee_rz').setLocked(osimState, true);
CoordSet.get('LHip_rz').setValue(osimState, 60*pi/180);
CoordSet.get('LHip_rz').setLocked(osimState, true);

% Recalculate the derivatives after the coordinate changes
stateDerivVector = model.computeStateVariableDerivatives(osimState);

% Controls function
controlsFuncHandle = @OpenSimPlantControlsFunction;

% Integrate plant using Matlab Integrator
timeSpan = [0 5];
integratorName = 'ode15s';
integratorOptions = odeset('AbsTol', 1E-5);

% Run Simulation

motionData = IntegrateOpenSimPlant(model, controlsFuncHandle, timeSpan, ...
    integratorName, integratorOptions);

% Plot Results
figHandle = figure;
PlotOpenSimData(fighandle, motionData, 'time', ...
    {'Pelvis_tx_u', 'LKnee_rz'});

% Clean up
clearvars stateDerivVector CoordSet
