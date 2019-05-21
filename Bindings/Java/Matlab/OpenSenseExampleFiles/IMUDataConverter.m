%% IMUDataConverter.m
% Example code for reading, and converting, XSENS IMU sensor data to
% OpenSense friendly format.
% Run this script from the OpenSenseExampleFiles directory. 

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

%% Clear any variables in the workspace
clear all; close all; clc; 

%% Import OpenSim libraries
import org.opensim.modeling.*

%% Build an Xsens Settings Object. 
% Instantiate the Reader Settings Class
xsensSettings = XsensDataReaderSettings('IMUData/MT_012005D6_009-001_Mappings.xml');
% Instantiate an XsensDataReader
xsens = XsensDataReader(xsensSettings);
% Get a table reference for the data
table = xsens.read('IMUData/');
% get the trial name from the settings
trial = char(xsensSettings.get_trial_prefix());
%% Get Orientation Data
quatTableTyped = xsens.getOrientationsTable(table);
% Write to file
STOFileAdapterQuaternion.write(quatTableTyped,  [trial '-quaternions.sto']);

%% Get Acceleration Data
accelTableTyped = xsens.getLinearAccelerationsTable(table);
% Write to file
STOFileAdapterVec3.write(accelTableTyped, [trial '-accelerations.sto']);

%% Get Magenometer Data
magTableTyped = xsens.getMagneticHeadingTable(table);
% Write to file
STOFileAdapterVec3.write(magTableTyped,  [trial  '-magnetometers.sto']);

%% Get Gyro Data
gyroTableTyped = xsens.getAngularVelocityTable(table);
% Write to file
STOFileAdapterVec3.write(gyroTableTyped,  [trial  '-gyros.sto']);
