%% walkerOptimization.m
% Example code to perform an optimization of initial states for the OpenSim
% Dynamic Walker Example. This script requires the model
% 'Walker_Model.osim' be in the same folder. Once the script is complete, a
% model with optimized initial coordinate values and speeds will be
% printed to the same directory ('Walker_Model_Optimized.osim').
% 
% This example is unoptimized for efficiency, instead prioritizing simple, 
% easily understood structure and code. Be aware that the run time of this 
% function is anywhere between 40 to 60 minutes on a single core machine. 
% A good exercise would be to improve the efficiency of this function by
% editing the optimizer parameters and editing the objective function. 
% You can stop the optimization at any time by using the hotkeys ctl+c while 
% in the command window. 

% Written by James Dunne, Carmchael Ong, Tom Uchida, Ajay Seth. 

%% Clear all variables from the workspace
clear all; close all; clc; format long;

%% Define some Global variables. 
global model translation_store vectorOfDefaults_store;

%% Set the value of the Global Variables
translation_store = 0; % Store for pelvis X translations
vectorOfDefaults_store = []; % Store for Vector of Defaults

%% Import OpenSim libraries
import org.opensim.modeling.*

%% Define the path to your local OpenSim directory.
% Add path2GeometryFolder to the full path of your local Geometry folder.
path2GeometryFolder = '';
% Add Local Geometry folder to path 
ModelVisualizer.addDirToGeometrySearchPaths(path2GeometryFolder);

%% Open Model
% model = Model('Walker_Model.osim');
model = Model('Walker_Model.osim');
initial_state = model.initSystem();

% Define a vector of initial states that that will be changed by the
% optimizer to maximize the walking distance of the model. The
% vector is 12x1 in size and includes all the default, unlocked, coordinate
% values and coordinate speeds of the model. 
vectorOfDefaults = zeros(12,1);
vectorOfDefaults(1)  = model.getCoordinateSet().get('Pelvis_tx').getDefaultValue();
vectorOfDefaults(2)  = model.getCoordinateSet().get('Pelvis_tx').getDefaultSpeedValue();
vectorOfDefaults(3)  = model.getCoordinateSet().get('Pelvis_ty').getDefaultValue();
vectorOfDefaults(4)  = model.getCoordinateSet().get('Pelvis_ty').getDefaultSpeedValue();
vectorOfDefaults(5)  = model.getCoordinateSet().get('LHip_rz').getDefaultValue();
vectorOfDefaults(6)  = model.getCoordinateSet().get('LHip_rz').getDefaultSpeedValue();
vectorOfDefaults(7)  = model.getCoordinateSet().get('RHip_rz').getDefaultValue();
vectorOfDefaults(8)  = model.getCoordinateSet().get('RHip_rz').getDefaultSpeedValue();
vectorOfDefaults(9)  = model.getCoordinateSet().get('LKnee_rz').getDefaultValue();
vectorOfDefaults(10) = model.getCoordinateSet().get('LKnee_rz').getDefaultSpeedValue();
vectorOfDefaults(11) = model.getCoordinateSet().get('RKnee_rz').getDefaultValue();
vectorOfDefaults(12) = model.getCoordinateSet().get('RKnee_rz').getDefaultSpeedValue();

%% Set the Optimizer Parameters. 
% The optimizer has a large number of parameters that you can change to
% improve the efficiency of search of your optimization. We only change a
% few parameters for ease of use.  
opts = optimset('LargeScale','off', 'Display','on','UseParallel',false);

%% Run the Optimzation 
% Since the optimizer can easily get caught in a local minimum, we iterate
% a few times. For each iteration, we use the best result from the previous 
% optimization to begin. 
tic
iteration = 1;
while iteration <=3 
    % We pass the optimizer (fminunc) a handle to the function that does
    % the forward integration (walker_simulation_objective_function) the
    % vector of values that are optimized (vectorOfDefaults) and the
    % optimizer parameters (opts).
    [coeffs_new, J] = fminunc(@walker_simulation_objective_function, vectorOfDefaults, opts);
    % When the optimization ends, get the set of default model coordinate 
    % values and speeds that resulted in the best translation
    [n,i] = max(translation_store);
    vectorOfDefaults = vectorOfDefaults_store(i,:)';
    % Increment the iteration
    iteration = iteration + 1;
end
toc
%% After the iterations are complete, print a new model to file 
model.getCoordinateSet().get('Pelvis_tx').setDefaultValue(vectorOfDefaults(1));
model.getCoordinateSet().get('Pelvis_tx').setDefaultSpeedValue(vectorOfDefaults(2));
model.getCoordinateSet().get('Pelvis_ty').setDefaultValue(vectorOfDefaults(3));
model.getCoordinateSet().get('Pelvis_ty').setDefaultSpeedValue(vectorOfDefaults(4));
model.getCoordinateSet().get('LHip_rz').setDefaultValue(vectorOfDefaults(5));
model.getCoordinateSet().get('LHip_rz').setDefaultSpeedValue(vectorOfDefaults(6));
model.getCoordinateSet().get('RHip_rz').setDefaultValue(vectorOfDefaults(7));
model.getCoordinateSet().get('RHip_rz').setDefaultSpeedValue(vectorOfDefaults(8));
model.getCoordinateSet().get('LKnee_rz').setDefaultValue(vectorOfDefaults(9));
model.getCoordinateSet().get('LKnee_rz').setDefaultSpeedValue(vectorOfDefaults(10));
model.getCoordinateSet().get('RKnee_rz').setDefaultValue(vectorOfDefaults(11));
model.getCoordinateSet().get('RKnee_rz').setDefaultSpeedValue(vectorOfDefaults(12));

% Print the model to file
model.print('Walker_Model_Optimized.osim')




    
    
