%% OpenSense_OrientationTracking.m
% Example code to perform orienation tracking with OpenSense. 

% The OpenSim API is a toolkit for musculoskeletal modeling and           %
% simulation. See http://opensim.stanford.edu and the NOTICE file         %
% for more information. OpenSim is developed at Stanford University       %
% and supported by the US National Institutes of Health (U54 GM072970,    %
% R24 HD065690) and by DARPA through the Warrior Web program.             %
%                                                                         %
% Copyright (c) 2005-2019 Stanford University and the Authors             %
% Author(s): James Dunne                                                  %
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

%% Clear the Workspace variables. 
clear all; close all; clc;
import org.opensim.modeling.*

%% Set variables to use
modelFileName = 'Rajagopal_2015_calibrated.osim';                % The path to an input model
orientationsFileName = 'MT_012005D6_009-001_orientations.sto';   % The path to orientation data for calibration 
sensor_to_opensim_rotation = Vec3(-pi/2, 0, 0); % The rotation of IMU data to the OpenSim world frame 
visualizeTracking = true;  % Boolean to Visualize the tracking simulation
startTime = 7.25;          % Start time (in seconds) of the tracking simulation. 
endTime = 15;              % End time (in seconds) of the tracking simulation.
resultsDirectory = 'IKResults';

%% Instantiate an InverseKinematicsTool
imuIK = IMUInverseKinematicsTool();
 
%% Set the model path to be used for tracking
imuIK.set_model_file(modelFileName);
imuIK.set_orientations_file(orientationsFileName);
imuIK.set_sensor_to_opensim_rotations(sensor_to_opensim_rotation)
% Set time range in seconds
imuIK.set_time_range(0, startTime); 
imuIK.set_time_range(1, endTime);   
% Set a directory for the results to be written to
imuIK.set_results_directory(resultsDirectory)
% Run IK
imuIK.run(visualizeTracking);