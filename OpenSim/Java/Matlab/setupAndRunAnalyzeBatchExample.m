 %  
%   This is a Matlab function that generates OpenSim Setup Files for the 
%   Analyze>MuscleAnalysis tool and runs the Analyze tool in OpenSim 2.3.2.
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
% THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
% OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
% MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN
% NO EVENT SHALL THE AUTHORS, CONTRIBUTORS OR COPYRIGHT HOLDERS BE LIABLE
% FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
% CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH
% THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
%
% ==============================================================================
% =========================================================================
% =====
% Pull in the modeling classes straight from the OpenSim distribution
import org.opensim.modeling.*

% Turn up debug level so that exceptions due to typos etc. are handled gracefully
OpenSimObject.setDebugLevel(3);

function setupAndRunMABatchExample(subjectNumber)

% move to directory where this subject's files are kept
subjectDir = subjectNumber;
cd(subjectDir)

% Go to the folder in the subject's folder where IK Results are
ik_results_folder = './IKResults/';

% specify where setup files will be printed.
setupfiles_folder = './AnalyzeSetup/';

% specify where results will be printed.
results_folder = './AnalyzeResults/';

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
% choose files in results folder with .csv ending
cd(setupfiles_folder);
genericSetupForAn = [subjectNumber '_Setup_Analyze_generic.xml'];
%%%%UseAPI xmlDoc = xmlread(genericSetupForAn);
analyzeTool = AnalyzeTool(genericSetupForAn);
% get the file names that match the ik_reults convention
% this is where consistent naming conventions pay off
cd('..');
cd(ik_results_folder);
trialsForAn = dir('*_ik.mot');
nTrials =length(trialsForAn);

cd('..')
for trial= 1:nTrials;
    %get the name of the file for this trial
    motIKCoordsFile = trialsForAn(trial).name;
    
    % create name of trial from .trc file name
    name = regexprep(motIKCoordsFile,'_ik.mot','');
    
    % get .mot data to determine time range
    %%%%UseAPI motCoordsData=dlmread([ik_results_folder motIKCoordsFile],'\t',11,0);
    Storage motCoordsData=Storage([ik_results_folder motIKCoordsFile])
    
    % for this example, column is time
    %%%%UseAPI initial_time = motCoordsData(1,1);
    %%%%UseAPI final_time=motCoordsData(end,1);
    initial_time = motCoordsData.getFirstTime();
    final_time = motCoordsData.getLastTime();
    
    %%%%UseAPI xmlDoc.getElementsByTagName('AnalyzeTool').item(0).getAttributes.item(0).setValue(name);
    analyzeTool.setName(name);
    %%%%UseAPI xmlDoc.getElementsByTagName('coordinates_file').item(0).getFirstChild.setNodeValue([ik_results_folder motIKCoordsFile]);
    analyzeTool.setCoordinatesFileName([ik_results_folder motIKCoordsFile]);
    %%%%UseAPI xmlDoc.getElementsByTagName('initial_time').item(0).getFirstChild.setNodeValue([num2str(initial_time)]);
    analyzeTool.setInitialTime(initial_time);
    %%%%UseAPI xmlDoc.getElementsByTagName('final_time').item(0).getFirstChild.setNodeValue([num2str(final_time)]);
    analyzeTool.setFinalTime(final_time);
    %%%%UseAPI xmlDoc.getElementsByTagName('start_time').item(0).getFirstChild.setNodeValue([num2str(initial_time)]);
    %%%%UseAPI xmlDoc.getElementsByTagName('end_time').item(0).getFirstChild.setNodeValue([num2str(final_time)]);
   
    
    outfile = ['Setup_Analyze_' name '.xml'];
    %%%%UseAPI xmlwrite([setupfiles_folder outfile], xmlDoc);
    analyzeTool.print([setupfiles_folder outfile]);
    
    %%%%UseAPI Command = ['c:/OpenSim2.3.2/bin/analyze.exe -S ' setupfiles_folder outfile];
    analyzeTool.run();
    fprintf(['Performing IK on cycle # ' num2str(trial) '\n']);
    system(Command);
    
    % rename the out.log so that it doesn't get overwritten
    copyfile('out.log',['./AnalyzeResults/' name '_out.log'])
    
    
    
end
%sendmail(mail,subjectNumber, ['Hello! Analysis for ' subjectNumber '  is complete']);
end

