%% GenerateSubtrials_UsingConvenianceClasses.m
% Example code generating sub-trials from a single, large, IMU data file.
% This is usefull when many minutes of IMU data have been collected but
% orienation tracking with OpenSense is only necessary during specific
% times of the trial. This script uses the OpenSense Matlab Class 
% imuDataSlicer(). 

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

%% Instantiate a imuDataSlicer()
trialpath = 'MT_012005D6_009-001_orientations.sto';
accPath = 'MT_012005D6_009-001_accelerations.sto';
% Instantiate the data slicer to carve out smaller trial files
ds = imuDataSlicer(trialpath);

%% Slice the calibration trial and write it to file
stime = 0.00;
etime = 0.05;
ds.setDataTimeIntervalInMinutes(stime, etime)
ds.generateSubTrial();
ds.writeSubTrial('calibration_trial')

%% Slice out a section of walking data and write it to file. 
stime = 0.05;
etime = 0.15;
ds.setDataTimeIntervalInMinutes(stime, etime);
ds.generateSubTrial();
ds.writeSubTrial('Walking_05_10');

