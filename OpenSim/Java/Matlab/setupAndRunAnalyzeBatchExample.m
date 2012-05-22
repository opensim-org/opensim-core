 %  
%   This is a Matlab function that generates OpenSim Setup Files for the 
%   Analyze>MuscleAnalysis tool and runs the Analyze tool.
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
%   Laband the MathWorks Support website:
%   http://www.mathworks.com/support/solutions/en/data/1-3PRRDV/
%  
% =========================================================================
% =====
% Pull in the modeling classes straight from the OpenSim distribution
import org.opensim.modeling.*

% Turn up debug level so that exceptions due to typos etc. are handled gracefully
OpenSimObject.setDebugLevel(3);

% move to directory where this subject's files are kept
subjectDir = uigetdir('testData', 'Select the folder that contains the current subject data');

% Go to the folder in the subject's folder where IK Results are
ik_results_folder = [subjectDir './IKResults/'];

% specify where setup files will be printed.
setupfiles_folder = [subjectDir './AnalyzeSetup/'];

% specify where results will be printed.
results_folder = [subjectDir './AnalyzeResults/'];

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
genericSetupForAn = [setupfiles_folder subjectNumber '_Setup_Analyze_generic.xml'];
analyzeTool = AnalyzeTool(genericSetupForAn);

% get the file names that match the ik_reults convention
% this is where consistent naming conventions pay off
trialsForAn = dir([ik_results_folder '*_ik.mot']);
nTrials =length(trialsForAn);

for trial= 1:nTrials;
    %get the name of the file for this trial
    motIKCoordsFile = trialsForAn(trial).name;
    
    % create name of trial from .trc file name
    name = regexprep(motIKCoordsFile,'_ik.mot','');
    
    % get .mot data to determine time range
    motCoordsData = Storage([ik_results_folder motIKCoordsFile]);
    
    % for this example, column is time
    initial_time = motCoordsData.getFirstTime();
    final_time = motCoordsData.getLastTime();
    
    analyzeTool.setName(name);
    analyzeTool.setResultsDir(results_folder);
    analyzeTool.setCoordinatesFileName([ik_results_folder motIKCoordsFile]);
    analyzeTool.setInitialTime(initial_time);
    analyzeTool.setFinalTime(final_time);   
    
    outfile = ['Setup_Analyze_' name '.xml'];
    analyzeTool.print([setupfiles_folder outfile]);
    
    analyzeTool.run();
    fprintf(['Performing IK on cycle # ' num2str(trial) '\n']);
    
    % rename the out.log so that it doesn't get overwritten
    copyfile('out.log',[results_folder name '_out.log'])
    
end
%sendmail(mail,subjectNumber, ['Hello! Analysis for ' subjectNumber '  is complete']);

