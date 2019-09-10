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

% Reset
clear all; close all; clc

% Import
import org.opensim.modeling.*;

% Open a Model by name
osimModel = Model('../Model/WalkerModelTerrain.osim');

% Use the visualizer (must be done before the call to init system)
osimModel.setUseVisualizer(true);

% get a reference to the underlying computational system
osimState = osimModel.initSystem();

% Set the initial states of the model
CoordSet = osimModel.getCoordinateSet();
CoordSet.get('Pelvis_tx').setValue(osimState, 0.0);
CoordSet.get('Pelvis_ty').setValue(osimState, 1.0);
CoordSet.get('LHip_rz').setValue(osimState, 0.52359878);
CoordSet.get('RHip_rz').setValue(osimState,-0.17453293);
CoordSet.get('LKnee_rz').setValue(osimState, -0.52359878);
CoordSet.get('RKnee_rz').setValue(osimState, -0.52359878);

stateDerivVector = osimModel.computeStateVariableDerivatives(osimState);

% Controls function
controlsFuncHandle = [];

% Integrate plant using Matlab Integrator
timeSpan = [0 2];
integratorName = 'ode15s';
integratorOptions = odeset('AbsTol', 1E-07, 'RelTol', 1E-05);

% Run Simulation
motionData = IntegrateOpenSimPlant(osimModel, controlsFuncHandle, timeSpan, ...
    integratorName, integratorOptions);

% Plot Results
figHandle = figure;
PlotOpenSimData(figHandle, motionData, 'time', ...
    {'Pelvis_tx_u', 'LKnee_rz'});

% Clean up
clearvars stateDerivVector CoordSet
