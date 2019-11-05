%% CalibrateOpenSenseModel.m
% Example code to calibrate orienation data with OpenSense. This
% script uses the OpenSense library functions and is part of the OpenSense
% Example files. 

% ----------------------------------------------------------------------- %
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
modelFileName = 'Rajagopal_2015.osim';        % The path to an input model
orientationsFileName = 'MT_012005D6_009-001_orientations.sto';   % The path to orientation data for calibration 
baseIMUName = 'pelvis_imu';                     % The base IMU is the IMU on the base body of the model that dictates the heading (forward) direction of the model.
baseIMUHeading = CoordinateAxis(2);             % The Coordinate Axis of the base IMU that points in the heading direction. 
visulizeCalibration = true;                     % Boolean to Visualize the Output model

%% Instantiate an OpenSenseUtilities object
ou = OpenSenseUtilities();
 
%% Generate a model that calibrates the IMU sensors to a model pose.
model = ou.calibrateModelFromOrientations(modelFileName,...
                                          orientationsFileName,...
                                          baseIMUName,...
                                          baseIMUHeading,...
                                          visulizeCalibration);
%% Print the calibrated model to file.
model.print(['calibrated_' modelFileName])