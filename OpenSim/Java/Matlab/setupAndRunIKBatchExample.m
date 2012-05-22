% This example script runs multiple inverse kinematics trials for the model Subject01. 
% All input files are in the folder ../Matlab/testData/Subject01
% To see the results load the model and ik output in the GUI.

% Pull in the modeling classes straight from the OpenSim distribution
import org.opensim.modeling.*

% Turn up debug level so that exceptions due to typos etc. are handled gracefully
OpenSimObject.setDebugLevel(3);

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
model = Model([modelFilePath modelFile]);
model.initSystem()

% Tell Tool to use the loaded model
ikTool.setModel(model)

% Choose the marker data in .trc format
[trialsForIK,trialsForIKPath,FilterIndex] = ...
    uigetfile('*.trc','Pick the .trc files for the trials to include.','MultiSelect','on');
nTrials =length(trialsForIK);

% Loop through the trials
for trial= 1:nTrials;
    
    % Get the name of the file for this trial
    markerFile = trialsForIK(trial);
    
    % Create name of trial from .trc file name
    name = regexprep(trialsForIK{trial},'.trc','');
    
    % Get trc data to determine time range
    markerData = MarkerData([trialsForIKPath trialsForIK{trial}]);
    
    % Get initial and intial time 
    initial_time = markerData.getStartFrameTime();
    final_time = markerData.getLastFrameTime();
    
    % Setup the ikTool for this trial
    ikTool.setName(name);
    ikTool.setMarkerDataFileName([trialsForIKPath markerFile{1}]);
    ikTool.setStartTime(initial_time);
    ikTool.setEndTime(final_time);
    ikTool.setOutputMotionFileName([results_folder '\' name '_ik.mot']);
    
    % Save the settings in a setup file
    outfile = ['Setup_IK_' name '.xml'];
    ikTool.print([genericSetupPath '\' outfile]);
    
    % Run IK
    ikTool.run();
    fprintf(['Performing IK on cycle # ' num2str(trial) '\n']);
    
    % Rename the out.log so that it doesn't get overwritten
    copyfile('out.log',[results_folder '\' name '_out.log'])

end