% ----------------------------------------------------------------------- 
% The OpenSim API is a toolkit for musculoskeletal modeling and           
% simulation. See http://opensim.stanford.edu and the NOTICE file         
% for more information. OpenSim is developed at Stanford University       
% and supported by the US National Institutes of Health (U54 GM072970,    
% R24 HD065690) and by DARPA through the Warrior Web program.             
%                                                                         
% Copyright (c) 2005-2013 Stanford University and the Authors             
% Author(s): Chris Dembia                                            
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
% This file is a sample client script to the following scripts in this
% directory:
%     AddCustomFeet.m                          
%     AddExpressionPointToPointForceMagnets.m                
%     IntegrateOpenSimPlant.m                            
%     PlotOpensimData.m                                         
% 
% The file compares the performance of the
% DW2013_WalkerModelTerrainAddCustomFeet.osim and
% DW2013_WalkerModelTerrainAddMagnet.osim models for initial conditions of
% our choosing.
%
% It should be fairly easy to modify this file to compare any two other
% walker models that are based off of DW2013_WalkerModelTerrain.osim.
%
% Author: Chris Dembia
% ----------------------------------------------------------------------- 
% Import Java Library 
import org.opensim.modeling.*


%% Settings
duration = 2.0; % seconds.

%% Initial conditions

% Coordinate values.
Pelvis_tx = 0.0; % meters.
Pelvis_ty = 1.5; % meters.

LHip_rz = 40.0; % degrees.
RHip_rz = -10.0; % degrees.
LKnee_rz = 0; % degrees.
RKnee_rz = 0.0; % degrees.

% Initial speeds.
Pelvis_tx_u = 0.0; % meters/second.
Pelvis_ty_u = 0.0; % meters/second.

LHip_rz_u = 0.0; % radians/second.
RHip_rz_u = 0.0; % radians/second.
LKnee_rz_u = 0.0; % radians/second.
RKnee_rz_u = 0.0; % radians/second.


%% Create and load the model files that we'll compare.
% If you've already completed these files on your own, feel free to
% change this boolean to false.
useReferenceCompleteModelGeneratingScripts = true;

if useReferenceCompleteModelGeneratingScripts
    referenceDir = '../Reference/';
    % Creates DW2013_WalkerModelTerrainAddFoot.osim in ../Model:
    % ('fullfile' conveniently concatenates strings to make file paths)
    run(fullfile(referenceDir, 'AddCustomFeetComplete.m'));
    % Creates DW2013_WalkerModelTerrainAddMagnet.osim in ../Model:
    run(fullfile(referenceDir, 'AddExpressionPointToPointForceMagnetsComplete.m'));
else
    AddCustomFeet; 
    AddExpressionPointToPointForceMagnets;
end

% Load the models now.
modelDir = '../Model/';
modelA = Model(fullfile(modelDir, 'DW2013_WalkerModelTerrainAddCustomFeet.osim'));
modelB = Model(fullfile(modelDir, 'DW2013_WalkerModelTerrainAddMagnet.osim'));


%% Set the initial conditions in the models.
models = {modelA, modelB};

% For both models:

for iM = 1:length(models)
    
    % Values.
    models{iM}.updCoordinateSet().get('Pelvis_tx').setDefaultValue(Pelvis_tx);
    models{iM}.updCoordinateSet().get('Pelvis_ty').setDefaultValue(Pelvis_ty);
    
    deg2rad = pi / 180.0;
    models{iM}.updCoordinateSet().get('LHip_rz').setDefaultValue(deg2rad * LHip_rz);
    models{iM}.updCoordinateSet().get('RHip_rz').setDefaultValue(deg2rad * RHip_rz);
    models{iM}.updCoordinateSet().get('LKnee_rz').setDefaultValue(deg2rad * LKnee_rz);
    models{iM}.updCoordinateSet().get('RKnee_rz').setDefaultValue(deg2rad * RKnee_rz);
    
    % Speeds.
    models{iM}.updCoordinateSet().get('Pelvis_tx').setDefaultSpeedValue(Pelvis_tx_u);
    models{iM}.updCoordinateSet().get('Pelvis_ty').setDefaultSpeedValue(Pelvis_ty_u);
    
    models{iM}.updCoordinateSet().get('LHip_rz').setDefaultSpeedValue(LHip_rz_u);
    models{iM}.updCoordinateSet().get('RHip_rz').setDefaultSpeedValue(RHip_rz_u);
    models{iM}.updCoordinateSet().get('LKnee_rz').setDefaultSpeedValue(LKnee_rz_u);
    models{iM}.updCoordinateSet().get('RKnee_rz').setDefaultSpeedValue(RKnee_rz_u);
    
end

% Integrator Options
integratorName = 'ode15s';
integratorOptions = odeset('AbsTol', 1E-5');
controlsFuncHandle = [];

% Simulate!
disp('Simulating model A.');
statesA = IntegrateOpenSimPlant(modelA, controlsFuncHandle, [0 duration], integratorName, integratorOptions);
disp('Simulating model B.');
statesB = IntegrateOpenSimPlant(modelB, controlsFuncHandle, [0 duration], integratorName, integratorOptions);

PlotOpenSimData(statesA, 'Pelvis_tx', {'Pelvis_ty'});
axis equal; % Since we're plotting in space.
PlotOpenSimData(statesB, 'Pelvis_tx', {'Pelvis_ty'});
axis equal;











