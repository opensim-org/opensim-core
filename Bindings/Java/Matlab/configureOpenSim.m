 function configureOpenSim
%% configureOpenSim() adds OpenSim library paths to MATLAB path files
%    Allows user to select the location of the OpenSim install directory
%    and add these paths to the matlab library paths. Will allow MATLAB 
%    users to run OpenSim calls in Matlab.
%    Build Data: 10.17.2013
%    Tested with Matlab 2012a and OpenSim 3.1 on Windows 7
%    Authors: The OpenSim Team (James Dunne, Chris Dembia & Ayman Habib)  
% #----------------------------------------------------------------------- %
% #The OpenSim API is a toolkit for musculoskeletal modeling and           %
% #simulation. See http://opensim.stanford.edu and the NOTICE file         %
% #for more information. OpenSim is developed at Stanford University       %
% #and supported by the US National Institutes of Health (U54 GM072970,    %
% #R24 HD065690) and by DARPA through the Warrior Web program.             %
% #                                                                        %   
% #Copyright (c) 2005-2013 Stanford University and the Authors             %
% #                                                                        %   
% #Licensed under the Apache License, Version 2.0 (the "License");         %
% #you may not use this file except in compliance with the License.        %
% #You may obtain a copy of the License at                                 %
% #http://www.apache.org/licenses/LICENSE-2.0.                             %
% #                                                                        % 
% #Unless required by applicable law or agreed to in writing, software     %
% #distributed under the License is distributed on an "AS IS" BASIS,       %
% #WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or         %
% #implied. See the License for the specific language governing            %
% #permissions and limitations under the License.                          %
% #----------------------------------------------------------------------- %

%% Determine the OpenSim install location and compatability
% MATLAB root dir
    mr = matlabroot;
% point to opensim folder
   correctFolder=0;
    while ~correctFolder
        prospectiveOpenSimFolder = getenv('OPENSIM_HOME');
        openSimFolder = uigetdir(prospectiveOpenSimFolder,'Select the folder where OpenSim is installed.');
        if ~openSimFolder  % user cancelled process
            error('User cancelled function')
        end
        % Check if the correct OpenSim directory is selected
        if ~exist(fullfile(openSimFolder, 'sdk', 'buildinfo.txt'))
            h = msgbox('Folder is not an OpenSim Install folder. Please select OpenSim Install folder','Error');
            uiwait(h)
        else
            correctFolder = 1;
            % Check if Matlab and OpenSim are compatible (64 vs 32 bit)
            checkSystemInfo(openSimFolder) 
        end
    end
%% Edit the classpath.txt file (need full path for print)
% Create the string names used
    classFile = fullfile(mr, 'toolbox', 'local', 'classpath.txt');
    OpenSimJarPath =  fullfile(openSimFolder, 'opensim','modules','org-opensim-modeling.jar');
% Edit the class path txt file
    edit_path_txt_file(classFile,OpenSimJarPath)

%% Edit the librarypath.txt (need full path for print)
% Create the string names used
    libraryFile = fullfile(mr, 'toolbox', 'local','librarypath.txt');
    OpenSimLibPath  = fullfile(openSimFolder, 'bin');
% Edit the library path txt file
    edit_path_txt_file(libraryFile,OpenSimLibPath)
%% Edit Matlab path
    cleanMatlabSearchPath
    addpath(OpenSimLibPath)
    savepath
%% Display message
    h = msgbox('Paths have been successfully updated. To complete setup, restart Matlab. To test your set up, type: model = org.opensim.modeling.Model();',  [char(openSimFolder) ' Successfully added!']);
    uiwait(h)
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function edit_path_txt_file(txtname,spath)
% Edit's path .txt files. Deletes old entries and adds new path.
    % Open the txt file
    fileID = fopen(txtname);
    C = textscan(fileID,'%s', 'delimiter','\n');
    fclose(fileID);
    % Search the lines for previous entries and delete
    Cnew = searchForOpenSimString(C);
    % Add matlab jar path to cell
    [Cnrows,Cncols] = cellfun(@size, Cnew);
    Cnew{1}{Cnrows+1} = {spath};
    % Print the new cell array back to the txt file
    fileID = fopen(txtname,'w');
    for i=1:Cnrows+1
        fprintf(fileID,'%s\n',char(Cnew{1}{i}));
    end
    fclose(fileID);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [newC] = searchForOpenSimString(C)
% Creates a new cell (newCnrows)and populates with only lines that dont
% contain an 'OpenSim' string. Effectively deleting previous entries.
    [Cnrows,Cncols] = cellfun(@size, C);
    for i=1:Cnrows
        if isempty(strfind(lower(C{1}{i}),'opensim'))
           if i==1 
              newC{1}{1,1}=C{1}{1,1};
           else
              [newCnrows,k] = cellfun(@size, newC); 
               newC{1}{newCnrows+1,1} = C{1}{i,1};
           end
        end
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function checkSystemInfo(openSimFolder)
 % check to see if the buildinfo.txt file is present. This file will give
 % us install type info that determines compatability between MATLAB and
 % opensim (both 64 bit?)
 if ~exist(fullfile(openSimFolder, 'sdk', 'buildinfo.txt'))
	 % is systemInfo.txt file available?
     h = msgbox('Unable to check OpenSim version (64 or 32). Buildinfo.txt not found in OpenSim\SDK folder. Please download OpenSim 3.2', 'Error');
     uiwait(h)
     return
 else
    fileID = fopen(fullfile(openSimFolder, 'sdk', 'buildinfo.txt'));
    OpenSimInstallInfo = textscan(fileID,'%s');
    fclose(fileID);
    platformID=char(OpenSimInstallInfo{1,1}{end,1});
    % get the last two digits of the text string ie '86' or '64'
    OsimInstallBuild= platformID(length(platformID)-1:end);
    % matlab is 64 and OpenSim is 32
    if strncmp(computer('arch'),'win64',5) &&  ~strncmp(OsimInstallBuild,'64',2) 
        h = msgbox('Matlab version is 64 bit while OpenSim is 32 bit. Please install OpenSim 64bit','Error');
        uiwait(h)
        error('Program exit')
    end
    % matlab is 32 and OpenSIm is 64
     if ~strncmp(computer('arch'),'win64',5) &&  strncmp(OsimInstallBuild,'64',2) 
        h = msgbox('Matlab version is 32 bit while OpenSim is 64 bit. Please install OpenSim 32bit','Error');
        uiwait(h)
        error('Program exit') 
     end
 end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function cleanMatlabSearchPath
 % goes through the matlab search path and removes any folders that have
 % the strings 'OpenSim' and 'bin' in them. This is to clean out older bin
 % folders.

 % get the matlab path    
 matlabPath         = path;
 % matlab path is just 1 long string soindex location of ';'
 matLabFoldersIndex = strfind(matlabPath,';');
 matLabFoldersIndex = [0 matLabFoldersIndex];
 % How many folders?
 nFoldersInPath     = length(matLabFoldersIndex); 
 
 % for each folder
 for i = 1:nFoldersInPath-1
     % get the start end end index for each folder name   
     startString = matLabFoldersIndex(i);
     finishString = matLabFoldersIndex(i+1);
     % ouput the folder string
     folderName = matlabPath(startString+1 : finishString-1);
     
     % check to see if the folder string contains 'OpenSim' or 'bin'
     if ~isempty(strfind(lower(folderName), 'opensim')) && ~isempty(strfind(folderName, 'bin'))
         rmpath(folderName)
     end     
 end
end



