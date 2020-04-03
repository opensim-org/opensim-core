%% CalibrateOpenSenseModel.m	
% Example code to calibrate orienation data with OpenSense IMU procedure. 

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
modelFileName = 'Rajagopal_2015.osim';          % The path to an input model
orientationsFileName = 'MT_012005D6_009-001_orientations.sto';   % The path to orientation data for calibration 
sensor_to_opensim_rotations = Vec3(-pi/2, 0, 0);% The rotation of IMU data to the OpenSim world frame 
baseIMUName = 'pelvis_imu';                     % The base IMU is the IMU on the base body of the model that dictates the heading (forward) direction of the model.
baseIMUHeading = 'z';                           % The Coordinate Axis of the base IMU that points in the heading direction. 
visulizeCalibration = true;                     % Boolean to Visualize the Output model

%% Instantiate an IMUPlacer object
imuPlacer = IMUPlacer();

% Set properties for the IMUPlacer
imuPlacer.set_model_file(modelFileName);
imuPlacer.set_orientation_file_for_calibration(orientationsFileName);
imuPlacer.set_sensor_to_opensim_rotations(sensor_to_opensim_rotations);
imuPlacer.set_base_imu_label(baseIMUName);
imuPlacer.set_base_heading_axis(baseIMUHeading);

% Run the IMUPlacer
imuPlacer.run(visulizeCalibration);

% Get the model with the calibrated IMU's
model = imuPlacer.getCalibratedModel();

%% Print the calibrated model to file.
model.print( strrep(modelFileName, '.osim', '_calibrated.osim') );