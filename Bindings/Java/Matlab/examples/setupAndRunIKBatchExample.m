% ----------------------------------------------------------------------- %
% The OpenSim API is a toolkit for musculoskeletal modeling and           %
% simulation. See http://opensim.stanford.edu and the NOTICE file         %
% for more information. OpenSim is developed at Stanford University       %
% and supported by the US National Institutes of Health (U54 GM072970,    %
% R24 HD065690) and by DARPA through the Warrior Web program.             %
%                                                                         %   
% Copyright (c) 2005-2017 Stanford University and the Authors             %
% Author(s): Edith Arnold                                                 %  
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

% setupAndRunIKBatchExample.m                                                 
% Author: Edith Arnold

% This example script runs multiple inverse kinematics trials for the model Subject01. 
% All input files are in the folder ../Matlab/testData/Subject01
% To see the results load the model and ik output in the GUI.

% Pull in the modeling classes straight from the OpenSim distribution
import org.opensim.modeling.*

% move to directory where this subject's files are kept
subjectDir = uigetdir('testData', 'Select the folder that contains the current subject data');

% Go to the folder in the subject's folder where .trc files are
trc_data_folder = uigetdir(subjectDir, 'Select the folder that contains the marker data files in .trc format.');

% specify where results will be printed.
results_folder = uigetdir(subjectDir, 'Select the folder where the IK Results will be printed.');

% Get and operate on the files
% Choose a generic setup file to work from
[genericSetupForIK,genericSetupPath,FilterIndex] = ...
    uigetfile('*.xml','Pick the a generic setup file to for this subject/model as a basis for changes.');
ikTool = InverseKinematicsTool([genericSetupPath genericSetupForIK]);

% Get the model
[modelFile,modelFilePath,FilterIndex] = ...
    uigetfile('*.osim','Pick the the model file to be used.');

% Load the model and initialize
model = Model(fullfile(modelFilePath, modelFile));
model.initSystem();

% Tell Tool to use the loaded model
ikTool.setModel(model);

trialsForIK = dir(fullfile(trc_data_folder, '*.trc'));

nTrials = size(trialsForIK);

% Loop through the trials
for trial= 1:nTrials
    
    % Get the name of the file for this trial
    markerFile = trialsForIK(trial).name;
    
    % Create name of trial from .trc file name
    name = regexprep(markerFile,'.trc','');
    fullpath = fullfile(trc_data_folder, markerFile);
    
    % Get trc data to determine time range
    markerData = MarkerData(fullpath);
    
    % Get initial and intial time 
    initial_time = markerData.getStartFrameTime();
    final_time = markerData.getLastFrameTime();
    
    % Setup the ikTool for this trial
    ikTool.setName(name);
    ikTool.setMarkerDataFileName(fullpath);
    ikTool.setStartTime(initial_time);
    ikTool.setEndTime(final_time);
    ikTool.setOutputMotionFileName(fullfile(results_folder, [name '_ik.mot']));
    
    % Save the settings in a setup file
    outfile = ['Setup_IK_' name '.xml'];
    ikTool.print(fullfile(genericSetupPath, outfile));
    
    fprintf(['Performing IK on cycle # ' num2str(trial) '\n']);
    % Run IK
    ikTool.run();

end
