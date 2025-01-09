% ----------------------------------------------------------------------- %
% The OpenSim API is a toolkit for musculoskeletal modeling and           %
% simulation. See http://opensim.stanford.edu and the NOTICE file         %
% for more information. OpenSim is developed at Stanford University       %
% and supported by the US National Institutes of Health (U54 GM072970,    %
% R24 HD065690) and by DARPA through the Warrior Web program.             %
%                                                                         %   
% Copyright (c) 2005-2017 Stanford University and the Authors             %
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

% Pull in the modeling classes straight from the OpenSim distribution
import org.opensim.modeling.*

% move to directory where this subject's files are kept
subjectDir = uigetdir('testData', 'Select the folder that contains the current subject data');

% Go to the folder in the subject's folder where IK Results are
ik_results_folder = fullfile(subjectDir, 'IKResults');

% specify where setup files will be printed.
setupfiles_folder = fullfile(subjectDir, 'AnalyzeSetup');

% specify where results will be printed.
results_folder = fullfile(subjectDir, 'AnalyzeResults');

% %% To send an email at the end of this function define these variables appropriately:
% % more details available here:
% % http://www.mathworks.com/support/solutions/en/data/1-3PRRDV/
% %
% mail = 'youremailaddress@gmail.com'; %Your GMail email address
% password = 'yourPassword'; %Your GMail password
% 
% % Then this code will set up the preferences properly:
% setpref('Internet','E_mail',mail);
% setpref('Internet','SMTP_Server','smtp.gmail.com');
% setpref('Internet','SMTP_Username',mail);
% setpref('Internet','SMTP_Password',password);
% props = java.lang.System.getProperties;
% props.setProperty('mail.smtp.auth','true');
% props.setProperty('mail.smtp.socketFactory.class', 'javax.net.ssl.SSLSocketFactory');
% props.setProperty('mail.smtp.socketFactory.port','465');

%% Get and operate on the files

subjectNumber = 'subject01';
genericSetupForAn = fullfile(setupfiles_folder, [subjectNumber '_Setup_Analyze_generic.xml']);
analyzeTool = AnalyzeTool(genericSetupForAn);

% get the file names that match the ik_reults convention
% this is where consistent naming conventions pay off
trialsForAn = dir(fullfile(ik_results_folder, '*_ik.mot'));
nTrials =length(trialsForAn);

for trial= 1:nTrials
    % get the name of the file for this trial
    motIKCoordsFile = trialsForAn(trial).name;
    
    % create name of trial from .trc file name
    name = regexprep(motIKCoordsFile,'_ik.mot','');
    
    % get .mot data to determine time range
    motCoordsData = Storage(fullfile(ik_results_folder, motIKCoordsFile));
    
    % for this example, column is time
    initial_time = motCoordsData.getFirstTime();
    final_time = motCoordsData.getLastTime();
    
    analyzeTool.setName(name);
    analyzeTool.setResultsDir(results_folder);
    analyzeTool.setCoordinatesFileName(fullfile(ik_results_folder, motIKCoordsFile));
    analyzeTool.setInitialTime(initial_time);
    analyzeTool.setFinalTime(final_time);   
    
    outfile = ['Setup_Analyze_' name '.xml'];
    analyzeTool.print(fullfile(setupfiles_folder, outfile));
    
    analyzeTool.run();
    fprintf(['Performing IK on cycle # ' num2str(trial) '\n']);
    
    % rename the out.log so that it doesn't get overwritten
    copyfile('out.log', fullfile(results_folder, [name '_out.log']));
    
end
%sendmail(mail,subjectNumber, ['Hello! Analysis for ' subjectNumber '  is complete']);

