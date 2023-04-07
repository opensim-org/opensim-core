% CustomStaticOptimization
% ------------------------
%   This script provides a framework for you to build your own custom code to 
%   solve the static optimization problem. Fill in the sections labeled "TODO". 
%   Visit the companion Confluence page for suggestions on how to complete this 
%   code using a sample data set: 
%   simtk-confluence.stanford.edu/display/OpenSim/Custom+Static+Optimization+in+MATLAB

%-----------------------------------------------------------------------%
% The OpenSim API is a toolkit for musculoskeletal modeling and         %
% simulation. See http://opensim.stanford.edu and the NOTICE file       %
% for more information. OpenSim is developed at Stanford University     %
% and supported by the US National Institutes of Health (U54 GM072970,  %
% R24 HD065690) and by DARPA through the Warrior Web program.           %
%                                                                       %
% Copyright (c) 2020 Stanford University and the Authors                %
% Author(s): Nick Bianco                                                %
%                                                                       %
% Licensed under the Apache License, Version 2.0 (the "License");       %
% you may not use this file except in compliance with the License.      %
% You may obtain a copy of the License at                               %
% http://www.apache.org/licenses/LICENSE-2.0.                           %
%                                                                       %
% Unless required by applicable law or agreed to in writing, software   %
% distributed under the License is distributed on an "AS IS" BASIS,     %
% WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or       %
% implied. See the License for the specific language governing          %
% permissions and limitations under the License.                        %
%-----------------------------------------------------------------------%

close all; clear all; clc; beep off;

%% Import the OpenSim libraries.
import org.opensim.modeling.*;

%% Set filenames
% TODO {
modelFile = 
coordinatesFile = 
% }

%% Load model and get initial state.
model = Model(modelFile);
state = model.initSystem();

%% Use OpenSim tools to generate data for optimization.
% Use the AnalyzeTool to compute coordinate speeds. The tool automatically
% generates the files 'analyze_Kinematics_q.sto' and 'analyze_Kinematics_u.sto'
% which we'll use below to load the coordinate values and speeds.
if ~exist('analyze_Kinematics_q.sto', 'file')
    fprintf('Running kinematics analysis...\n\n');
    analyze = AnalyzeTool(model);
    analyze.setName('analyze');
    analyze.setCoordinatesFileName(coordinatesFile);
    analyze.loadStatesFromFile(state);
    analyze.setStartTime(0);
    analyze.setFinalTime(2.37);
    analysisSet = analyze.getAnalysisSet();
    kinematicsAnalysis = Kinematics();
    kinematicsAnalysis.setInDegrees(false);
    analysisSet.cloneAndAppend(kinematicsAnalysis);
    analyze.addAnalysisSetToModel();
    analyze.run();
end

% Run inverse dynamics tool to compute joint moments.
genForcesFile = 'generalized_forces.sto';
if ~exist(genForcesFile, 'file')
    fprintf('Running inverse dynamics...\n\n');
    idtool = InverseDynamicsTool();        
    % Part 1: Fill in the missing InverseDynamicsTool API commands.
    % TODO: Add API commands here {
    
    % }
    excludedForces = ArrayStr();
    excludedForces.append('muscles');
    idtool.setExcludedForces(excludedForces);
    idtool.run();
end

%% Load data into MATLAB arrays.
% Use the loadFilterCropArray() function included with the assigment to load the 
% coordinate kinematic and generalized force data into MATLAB arrays. This 
% function also filters and crops the loaded array based on its two input 
% arguments (more details in loadFilterCropArray.m).
lowpassFreq = 6.0; % Hz
timeRange = [0.81 1.96]; % Full gait cycle based on left-leg heelstrikes.
[coordinates, coordNames, time] = ...
        loadFilterCropArray('analyze_Kinematics_q.sto', lowpassFreq, timeRange);
[speeds, speedNames, ~] = ...
        loadFilterCropArray('analyze_Kinematics_u.sto', lowpassFreq, timeRange);
[genForces, forceNames, ~] = ...
        loadFilterCropArray(genForcesFile, lowpassFreq, timeRange);

% Part 2: Plot and inspect the generalized forces from inverse dynamics, to
% ensure they resemble typical walking joint moments.
% TODO: Plot forces here {

% }

% Re-order the columns of the generalized force data array to match the order of 
% the coordinate data array columns.
force2coord = zeros(length(coordNames),1);
for i = 1:length(forceNames)
   forcename = forceNames{i};
   for j = 1:length(coordNames)
       coordname = coordNames{j};
       if contains(forcename, '_moment')
           forcename = forcename(1:end-7);
       elseif contains(forcename, '_force')
           forcename = forcename(1:end-6);
       end
       if strcmp(forcename, coordname)
           force2coord(j) = i;
       end
   end
end
genForces = genForces(:, force2coord);

%% Part 3: Remove any unneeded generalized force data columns.
% Add substring entries to this array to remove all generalized forces whose 
% names contain each substring. For example, the provided entry, 'beta', removes
% 'knee_angle_r_beta_force' and 'knee_angle_l_beta_force', the forces associated
% with the CoordinateCoupler constraint that enforces patellar motion, which can
% be excluded from the static optimization problem.
% TODO: Add substrings here {  
forcesToRemove = {'beta', ...
    
    };
% }
% We'll build an array of indicies to remove all columns at once (see below).
colsToRemove = [];
for i = 1:length(coordNames)
    for j = 1:length(forcesToRemove)
        if contains(coordNames{i}, forcesToRemove{j})
            colsToRemove = [colsToRemove i];
        end
    end
end
% Before updating coordNames, store an array contain all the original coordinate
% names (this may be useful later).
coordNamesAll = coordNames;
% Remove columns and associated coordinate labels.
genForces(:, colsToRemove) = [];  
coordNames(colsToRemove) = [];

%% Part 4: Store max isometric force values and disable muscle dynamics
muscles = model.updMuscles();

% TODO: Initialize max isometric force array {
   
% }
for i = 1:muscles.getSize()
   % Downcast base muscle to Millard2012EquilibriumMuscle.
   muscle = Millard2012EquilibriumMuscle.safeDownCast(muscles.get(i-1));
   % Disable muscle dynamics.
   muscle.set_ignore_tendon_compliance(true);
   muscle.set_ignore_activation_dynamics(true);
   
   % TODO: Fill in max force array here {
   
   % }
end
% Update the system to include any muscle modeling changes.
state = model.initSystem();
 
%% Part 5: Perform static optimization.
% Use FMINCON to solve the static optimization problem at selected time points. 
% Type "help fmincon" into the command window for information to help you get
% started.

% Set the 'timeInterval' variable to select the time points to be included in the 
% optimization. For example, if set to 5, every 5th time point is selected. A 
% time interval of 1 will select all available time points.
timeInterval = 5;
% Update data arrays based on the time interval.
N = size(coordinates, 1);
coordinates = coordinates(1:timeInterval:N, :);
speeds = speeds(1:timeInterval:N, :);
genForces = genForces(1:timeInterval:N, :);
numTimePoints = size(coordinates, 1);

% Create the FMINCON options structure.
options = optimoptions('fmincon','Display','notify-detailed', ...
     'TolCon',1e-4,'TolFun',1e-12,'TolX',1e-8,'MaxFunEvals',100000, ...
     'MaxIter',5000,'Algorithm','interior-point');

% TODO: Write an activation squared cost as a MATLAB anonymous function here {

% }
 
% TODO: Construct initial guess and bounds arrays here {

% }

coords = model.getCoordinateSet();
for i = 1:numTimePoints
    fprintf('Optimizing...time step %i/%i \n',  i, numTimePoints);
    
    % TODO: Construct inputs to FMINCON here {

    % Loop through model coordinates to set coordinate values and speeds. We set
    % all coordinates to make sure we have the correct kinematic state when
    % compute muscle multipliers and moment arms.
    for j = 1:length(coordNamesAll)
        coord = coords.get(coordNamesAll{j});
        coord.setValue(state, coordinates(i,j));

        % TODO: Set the coordinate speed here {

        % }
    end

    % }
    
    % TODO: Call FMINCON to solve the problem here {
    
    % }
    
    % TODO: Store solution and set guess for next time point {
    
    % }
end

%% Part 6: Plot results.
% TODO: Create plots here {

% }
