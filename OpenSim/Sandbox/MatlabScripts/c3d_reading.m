% ----------------------------------------------------------------------- %
% The OpenSim API is a toolkit for musculoskeletal modeling and           %
% simulation. See http://opensim.stanford.edu and the NOTICE file         %
% for more information. OpenSim is developed at Stanford University       %
% and supported by the US National Institutes of Health (U54 GM072970,    %
% R24 HD065690) and by DARPA through the Warrior Web program.             %
%                                                                         %
% Copyright (c) 2005-2017 Stanford University and the Authors             %
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

% Author: James Dunne, Tom Uchida, Shrinidhi K. Lakshmikanth, Chris Dembia, 
% Ajay Seth, Ayman Habib, Jen Hicks.

%% Utility function for converting c3d data to .trc and .mot format. 
% c3d_reading(varargin)
% Inputs to function are pairs of string-string or string-number pairs.
%
% 'filepath', 'path2file'   reads c3d data at path2file.
% 'firstRotation'           num applies first rotation of num (dble)
% 'axis', 'X'               applies first rotation to axis X (string)
% 'secondRotation'          num applies second rotation of num (dble) 
% 'axis2', 'Y'              applies second rotation to axis Y (string)
%
% Example ? read WalkingData.c3d and perform a 90 degree rotation about X
%   c3d_reading('filepath', 'C:/data/WalkingData.c3d',...
%                 'firstRotation', '90'...
%                 'axis', 'X')

function c3d_reading(varargin)

p = inputParser;
defaultFirstRotation = 0;
default2ndRotation = 0;
defaultFilePath = '';
defaultAxis = '';
expectedAxis = {'X','Y','Z'};

addOptional(p,'filepath',defaultFilePath)
addOptional(p,'firstRotation',defaultFirstRotation,@isnumeric);
addOptional(p,'secondRotation',default2ndRotation,@isnumeric);
addOptional(p,'axis',defaultAxis,...
                 @(x) any(validatestring(x,expectedAxis)));
addOptional(p,'axis2',defaultAxis,...
                 @(x) any(validatestring(x,expectedAxis)));
             
parse(p,varargin{:});

axis = p.Results.axis;
axis2 = p.Results.axis2;
filepath = p.Results.filepath;
rot1 = p.Results.firstRotation;
rot2 = p.Results.secondRotation;


%% check for file path
if isempty(filepath)
        [filein, pathname] = uigetfile({'*.c3d','C3D file'}, 'C3D data file...');
        filepath = fullfile(pathname,filein);
elseif nargin >= 1
        if exist(filepath,'file') == 0
            error('file does not exist')
        end        
end
[path, filename, ext] = fileparts(filepath);

%% import java libraries
import org.opensim.modeling.*

%% Use a c3dAdapter to turn read a c3d file
adapter = C3DFileAdapter();
tables = adapter.read(filepath);

%% get the marker data
markers = tables.get('markers');

%% get the force data
forces = tables.get('forces');

%% Define a rotation matix
rotations = rotateCoordinateSys(axis,rot1,axis2,rot2);

if isempty(rotations);
    rotate = false;
else
    rotate = true;
    rotNames = fieldnames(rotations);
    nRots    = length(rotNames);
end

%% Rotate marker and force data
if rotate
    for i = 1 : nRots
        rotationMatrix = rotations.(rotNames{i});
        % rotate marker data
        markers = rotateTableData(markers, rotationMatrix);
        % rotate force data
        forces_rotated = rotateTableData(forces,rotationMatrix);
    end
end
%% Print the rotated markers to trc file
% make trc adapter and write marker tables to file.
% TRCFileAdapter requires the table to have DataRate and Units meta data.
% In this case, we made a copy of markers table, so the meta data got
% copied. If you make a new table, you will need to set these meta data
% keys before using TRCFileAdapter.
% ie markers.addTableMetaDataString('DataRate', '250')
TRCFileAdapter().write(markers,[filename '.trc']);

%% Print the force data as a Vec3 sto file and a flattened doubles sto file
% make postfix string vector for naming colomns
postfix = StdVectorString();
postfix.add('_x');
postfix.add('_y');
postfix.add('_z');

%% flatten the Vec3 table to a doubles table. 
% This converts a table of Vec3's into a flat table of doubles. Default
% column names have '_1', '_2', '_3' added. Here we specify the postfic as
% '_x', '_y', and 'z'.
forces_flattened = forces.flatten(postfix);
% make a sto adapter and write the forces table to file.
STOFileAdapter().write(forces_flattened,[filename '.mot']);

end


function table_rotated = rotateTableData(table, rotationMatrix)

import org.opensim.modeling.*

nLabels = table.getNumColumns();
nRows = table.getNumRows();

table_rotated = table();

for it = 0 : nLabels - 1

    % get the column data for the marker
    table_column = table_rotated.updDependentColumnAtIndex( it   );

    % go through each element of the table column, rotate the Vec3, and write
    % back to the column.
    for iRow = 0 : nRows - 1
        % get Matlab vector marker position
        vectorData = [table_column.getElt(0,iRow).get(0)...
                      table_column.getElt(0,iRow).get(1)...
                      table_column.getElt(0,iRow).get(2)];

        % rotate the marker data
        rotatedData = [rotationMatrix*vectorData']';

        % Write the rotated data back to the Vec3TimesSeriesTable
        elem = Vec3(rotatedData(1),rotatedData(2),rotatedData(3));
        % set the value of the element
        table_column.set(iRow, elem);
     end
end
end

function rotations = rotateCoordinateSys(axis,rot1,axis2,rot2)

if isempty(axis)
    rotations = [];
    return
elseif isempty(axis2)
    nRot = 1;
else
    nRot = 2;
end

rotations = struct();

for i = 1 : nRot
 
    if i == 1 
        rotAxis = axis;
        Rot     = rot1;
        rotOrder = 'firstRotation';
    else
        rotAxis = axis2;
        Rot     = rot2;
        rotOrder = 'secondRotation';
    end
    
    % Create roation matrices according to Rot (degrees)   
    RotAboutX1 = [1,0,0;0,cosd(Rot),-(sin(Rot*pi/180));0,sin(Rot*pi/180),cosd(Rot)];
    RotAboutY1 = [cosd(Rot),0,sin(Rot*pi/180);0,1,0;-(sin(Rot*pi/180)),0,cosd(Rot)];
    RotAboutZ1 = [cosd(Rot),-(sin(Rot*pi/180)),0;sin(Rot*pi/180),cosd(Rot),0;0,0,1];

    % choose which rotation matrix to use based on user input 
    if strcmp(upper(rotAxis),'X') 
        rotationMatrix = RotAboutX1;
    elseif strcmp(upper(rotAxis),'Y') 
        rotationMatrix = RotAboutY1;
    elseif strcmpi(upper(rotAxis),'Z')
        rotationMatrix = RotAboutZ1;
    end
    
    rotations.(rotOrder) = rotationMatrix;
      
end
end
