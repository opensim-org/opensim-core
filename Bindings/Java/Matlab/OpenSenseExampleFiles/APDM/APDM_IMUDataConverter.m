%% APDMReading_example.m
% Example code for reading, and converting, APDM IMU sensor data to OpenSim
% friendly format.

% ----------------------------------------------------------------------- %
% The OpenSim API is a toolkit for musculoskeletal modeling and           %
% simulation. See http://opensim.stanford.edu and the NOTICE file         %
% for more information. OpenSim is developed at Stanford University       %
% and supported by the US National Institutes of Health (U54 GM072970,    %
% R24 HD065690) and by DARPA through the Warrior Web program.             %
%                                                                         %
% Copyright (c) 2005-2019 Stanford University and the Authors             %
% Author(s): James Dunne, Ajay Seth, Ayman Habib, Jen Hicks, Chris Dembia %
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

%% Clear any vairables in the workspace
clear all; close all; clc;

%% Import OpenSim libraries
import org.opensim.modeling.*

%% Build an apdm Settings Object.
% Instantiate the Reader Settings Class
apdmSettings = APDMDataReaderSettings();

% List of files to read
vecOfNames = StdVectorString();
vecOfNames.add('Static');
vecOfNames.add('Upper');
vecOfNames.add('Middle');

% List of IMU Names to read
vecOfIMUNames = StdVectorString();
vecOfIMUNames.add('torso');
vecOfIMUNames.add('pelvis');
vecOfIMUNames.add('shank');

for i = 0 : vecOfIMUNames.size() - 1
    % Instantiate an ExpermentalSensor using the source data file and a IMU
    % name
    nextSensor = ExperimentalSensor(vecOfNames.get(i), vecOfIMUNames.get(i) );
    % Add the ExperimentalSensor to the Settings
    apdmSettings.append_ExperimentalSensors(nextSensor);
end

% Write the settings to xml file
apdmSettings.print('apdmTrial_Settings.xml');

%% Instantiate a apdmDataReader
apdm = APDMDataReader(apdmSettings);
trialName = 'imuData01.csv';
tables = apdm.read(trialName);

%% Get Orientation Data as quaternions
quatTableTyped = apdm.getOrientationsTable(tables);
% Write to file
STOFileAdapterQuaternion.write(quatTableTyped,  strrep(trialName,'.csv', '_quaternions.sto'));

%% Get Acceleration Data
accelTable = apdm.getLinearAccelerationsTable(tables);
% Write to file
STOFileAdapterVec3.write(accelTable, strrep(trialName,'.csv', '_accelerations.sto'));

%% Get Magnetic (North) Heading Data
magTable = apdm.getMagneticHeadingTable(tables);
% Write to file
STOFileAdapterVec3.write(magTable,  strrep(trialName,'.csv', '_magnetometers.sto'));

%% Get Angular Velocity Data
angVelTable = apdm.getAngularVelocityTable(tables);
% Write to file
STOFileAdapterVec3.write(angVelTable,  strrep(trialName,'.csv', '_gyros.sto'));
