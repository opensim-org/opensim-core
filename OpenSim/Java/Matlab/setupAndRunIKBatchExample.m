%  
%   This is a Matlab function that generates OpenSim Setup Files for the 
%   Inverse Kinematics tool and runs the tool in OpenSim 2.3.2.
%
%   This work was developed as part of the batch processing demonstration 
%   during the OpenSim Advanced Users' Workshop, August 15-17, 2011 at
%   Stanford University and made freely available on The Musculoskeletal 
%   Modeler's Kitchen on simtk.org.
%  
% 
%   Author:  Edith Arnold
%            Neuromuscular Biomechanics Lab
%            Stanford University
%   Contact: https://simtk.org/home/modelerskitchen
%
%   Acknowledgements:  This work was made possible with guidance from Jeff
%   Reinbolt, Ajay Seth, and Ayman Habib of the Neuromuscular Biomechanics
%   Lab and the MathWorks Support website:
%   http://www.mathworks.com/support/solutions/en/data/1-3PRRDV/

%
% Portions copyright (c) 2011 the Author and/or Stanford University

% Permission is hereby granted, free of charge, to any person obtaining a
% copy of this software and associated documentation files (the
% "Software"), to deal in the Software without restriction, including
% without limitation the rights to use, copy, and modify the Software and
% to permit  persons to whom the Software is furnished to do so, subject to
% the  following conditions: 
% 
% This example was developed based on programs I wrote to save myself time
% during my PhD research.  It is being shared freely so that others may
% benefit from the parts of this work that won't show up in a peer reviewed
% publication. You must acknowledge the author and source of this
% work(https://simtk.org/home/modelerskitchen/) in any presentations or
% publications made possible by the tools presented here. 
% 
% I ask you to provide improvements you make to the Software to the
% biomechanics community via simtk.org, preferrably at the Musculoskeletal
% Modeler's Kitchen, so that we, and others, may benefit from your work.
% 
% You may not distribute this work. If others are interested in using the
% simulations, please direct them to this website.
% https://simtk.org/home/modelerskitchen/
% 
% The above copyright notice and this permission notice shall be included
% in all copies or substantial portions of the Software. 
%  
%  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
%  OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
%  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
%  IN NO EVENT SHALL THE AUTHORS, CONTRIBUTORS OR COPYRIGHT HOLDERS BE
%  LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
%  OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
%  WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
% 
%
% ==============================================================================
% =========================================================================
% =====
% Pull in the modeling classes straight from the OpenSim distribution
import org.opensim.modeling.*

% Turn up debug level so that exceptions due to typos etc. are handled gracefully
OpenSimObject.setDebugLevel(3);

function setupAndRunIKBatchExample(subjectNumber)

% move to directory where this subject's files are kept
subjectDir = subjectNumber;
cd(subjectDir)

% Go to the folder in the subject's folder where .trc files are
trc_data_folder = uigetdir(subjectDir, 'Select the folder that contains the marker data files in .trc format.')

% specify where setup files will be printed.
setupfiles_folder = uigetdir(subjectDir, 'Select the folder where the IK Setup Files will be printed.');
cd(setupfiles_folder);

% specify where results will be printed.
results_folder = uigetdir(subjectDir, 'Select the folder where the IK Results will be printed.')


%% To send an email at the end of this function define these variables appropriately:
% more details available here:
% http://www.mathworks.com/support/solutions/en/data/1-3PRRDV/

mail = 'youremailaddress@gmail.com'; %Your GMail email address
password = 'yourPassword'; %Your GMail password

% Then this code will set up the preferences properly:
setpref('Internet','E_mail',mail);
setpref('Internet','SMTP_Server','smtp.gmail.com');
setpref('Internet','SMTP_Username',mail);
setpref('Internet','SMTP_Password',password);
props = java.lang.System.getProperties;
props.setProperty('mail.smtp.auth','true');
props.setProperty('mail.smtp.socketFactory.class', 'javax.net.ssl.SSLSocketFactory');
props.setProperty('mail.smtp.socketFactory.port','465');

%% Get and operate on the files
% choose a generic setup file to work from
[genericSetupForIK,genericSetupPath,FilterIndex] = ...
    uigetfile('*.xml','Pick the a generic setup file to for this subject/model as a basis for changes.');
%%%%UseAPI xmlDoc = xmlread([genericSetupPath genericSetupForIK]);
ikTool = InverseKinematicsTool(genericSetupForIK)

% choose the marker data in .trc format
cd(trc_data_folder);
[trialsForIK,trialsForIKPath,FilterIndex] = ...
    uigetfile('*.trc','Pick the .trc files for the trials to include.','MultiSelect','on');
nTrials =length(trialsForIK);

for trial= 1:nTrials;
    %get the name of the file for this trial
    markerFile = trialsForIK(trial);
    
    % create name of trial from .trc file name
    name = regexprep(trialsForIK{trial},'.trc','');
    
    % get trc data to determine time range
    % this example uses trc data that where column 0 is
    % frame number and column 1 is time.
    % verify that the row and column values (6 and 1 in this case) in
    % dlmread are appropriate for your input files.
    %%%%UseAPI trcData=dlmread([trialsForIKPath trialsForIK{trial}],'\t',6,1);
    markerData = MarkerData(trcFileName)
    % get initial and inal time from first and last entry in time column
    %%%%UseAPI initial_time = trcData(1,1);
    %%%%UseAPI final_time=trcData(end,1);
    initial_time = markerData.getStartFrameTime();
    final_time = markerData.getLastFrameTime();
    %%%%UseAPI xmlDoc.getElementsByTagName('IKTool').item(0).getAttributes.item(0).setValue(name);
    %%%%UseAPI xmlDoc.getElementsByTagName('marker_file').item(0).getFirstChild.setNodeValue([trialsForIKPath markerFile{1}]);
    %%%%UseAPI xmlDoc.getElementsByTagName('output_motion_file').item(0).getFirstChild.setNodeValue([results_folder '\' name '_ik.mot']);
    %%%%UseAPI xmlDoc.getElementsByTagName('time_range').item(0).getFirstChild.setNodeValue([num2str(initial_time) ' ' num2str(final_time)]);
    ikTool.setName(name);
    ikTool.setMarkerDataFileName([trialsForIKPath markerFile{1}]);
    ikTool.setStartTime(initial_time);
    ikTool.setEndTime(final_time);
    ikTool.getPropertySet().get('output_motion_file').setValue([results_folder '\' name '_ik.mot']);
    cd(setupfiles_folder)
    
    outfile = ['Setup_IK_' name '.xml'];
    %%%%UseAPI xmlwrite([setupfiles_folder '\' outfile], xmlDoc);
    ikTool.print([setupfiles_folder '\' outfile]);
    
    %%%%UseAPI Command = ['c:/OpenSim2.3.2/bin/ik.exe -S ' outfile];
    ikTool.run();
    
    fprintf(['Performing IK on cycle # ' num2str(trial) '\n']);
    system(Command);
    
    % rename the out.log so that it doesn't get overwritten
    copyfile('out.log',[results_folder '\' name '_out.log'])
    
    
    
end
%sendmail(mail,subjectNumber, ['Hello! IK for ' subjectNumber '  is complete']);
end

