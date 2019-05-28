%% OrientationTacking_UsingConvenianceClasses.m
% Example code to calibrate and track orienation data with OpenSense. This
% script uses the Matlab Class orientationTrackingHelper(). 

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

%% Orientation Tracking
clear all; close all; clc;
% Instantiate an orienation tracking helper object. This is a Matlab Class
% that wraps both Calibration and Tracking in the same object for greater
% conveniance. 
ot = orientationTrackingHelper();

%% Calibrate the Model 
% Setup the Model Calibration.
ot.setModelCalibrationPoseFile('imuTrackingModel.osim');
% ot.setCalibrationTrialName('MT_012005D6_009-quaternions_calibration_trial_Facing_X.sto');
ot.setCalibrationTrialName('MT_012005D6_009-001_orientations.sto');
ot.setBaseHeadingAxis(0)
ot.setVisualizeCalibratedModel(0)
ot.setCalibratedModelOutputName('calibrated_imuTrackingModel.osim')
% Run the orientation calibration.
ot.generateCalibratedModel()
% Write the calibrated model to file. 
ot.writeCalibratedModel2File()

%% Run IK Orientation Tracking
% Set the name of the orientation File to be tracked
ot.setTrackingOrientationsFileName('MT_012005D6_009-001_orientations.sto');
% Set the Output directory for the results file
ot.setIKResultsDir('IKResults')
% Set the time interval (in minutes) to be tracked. 
ot.setIKTimeIntervalInMinutes(0.05,0.10);
% Set the Visualization Boolean.
ot.setVisualizeTracking(1);
% Run orientation tracking
ot.runOrientationTracking();





