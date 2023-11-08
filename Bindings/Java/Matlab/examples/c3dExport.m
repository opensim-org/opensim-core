% ----------------------------------------------------------------------- %
% The OpenSim API is a toolkit for musculoskeletal modeling and           %
% simulation. See http://opensim.stanford.edu and the NOTICE file         %
% for more information. OpenSim is developed at Stanford University       %
% and supported by the US National Institutes of Health (U54 GM072970,    %
% R24 HD065690) and by DARPA through the Warrior Web program.             %
%                                                                         %
% Copyright (c) 2005-2023 Stanford University and the Authors             %
% Author(s): James Dunne                                                  %
%            BJ Fregly                                                    %
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

% Specify point used for calculating output joint reaction moments
% 0 = electrical center, 1 = COP
%useCenterOfPressureAsMomentsPoint = 0;
% Set flag indicating whether or not to convert length units from mm to m
% 0 = no, 1 = yes
%convertLengthUnits = 0 if unspecified;
function c3dExport(useCenterOfPressureAsMomentsPoint, convertLengthUnits)

% The function names output .trc and .mot files with the same
% basename as the selcted input .c3d file.

    
    if nargin < 2
        convertLengthUnits =  0
    end
    %% Example of using the Matlab-OpenSim class 
    
    %% Load OpenSim libs
    import org.opensim.modeling.*
    
    %% Get the path to a C3D file
    [filename, path] = uigetfile('*.c3d');
    c3dpath = fullfile(path,filename);
    
    %% Construct an opensimC3D object with input c3d path
    % Constructor takes full path to c3d file and an integer for forceplate
    % representation in output forces file (0 = electrical center, 1 = COP). 
    c3d = osimC3D(c3dpath,useCenterOfPressureAsMomentsPoint);
    
    %% Get some stats...
    % Get the number of marker trajectories
    nTrajectories = c3d.getNumTrajectories();
    % Get the marker data rate
    rMakers = c3d.getRate_marker();
    % Get the number of forces 
    nForces = c3d.getNumForces();
    % Get the force data rate
    rForces = c3d.getRate_force();
    
    % Get Start and end time
    t0 = c3d.getStartTime();
    tn = c3d.getEndTime();
    
    %% Rotate the data 
    c3d.rotateData('x',-90)
    
    %% Get the c3d in different forms
    % Get OpenSim tables
    markerTable = c3d.getTable_markers();
    forceTable = c3d.getTable_forces();
    % Get as Matlab Structures
    [markerStruct forceStruct] = c3d.getAsStructs();
    
    %% Convert COP (mm to m) and Moments (Nmm to Nm)
    if convertLengthUnits
        c3d.convertMillimeters2Meters();
    end
    
    %% Write the marker and force data to file
    % Define output file names
    basename = strtok(filename,'.');
    markersFilename = strcat(basename,'_markers.trc');
    
    switch useCenterOfPressureAsMomentsPoint
        case 0
            forcesFilename = strcat(basename,'_forces_EC.mot');
        case 1
            forcesFilename = strcat(basename,'_forces_COP.mot');
    end
    
    % Write marker data to trc file.
    % c3d.writeTRC()                       Write to dir of input c3d.
    % c3d.writeTRC('Walking.trc')          Write to dir of input c3d with defined file name.
    % c3d.writeTRC('C:/data/Walking.trc')  Write to defined path input path.
    c3d.writeTRC(markersFilename);
    
    % Write force data to mot file.
    % c3d.writeMOT()                       Write to dir of input c3d.
    % c3d.writeMOT('Walking.mot')          Write to dir of input c3d with defined file name.
    % c3d.writeMOT('C:/data/Walking.mot')  Write to defined path input path.
    c3d.writeMOT(forcesFilename);

end
