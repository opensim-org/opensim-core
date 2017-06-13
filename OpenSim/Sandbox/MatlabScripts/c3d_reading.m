function c3d_reading(varargin)
% Utility function for converting c3d data to OpenSim format.
% Inputs to function are pairs of string-string or string-number pairs.
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

% Author: James Dunne, Tom Uchida, Shrinidhi K. Lakshmikanth, Chris Dembia, 
% Ajay Seth, Ayman Habib, Jen Hicks.


p = inputParser;
value = 0;
defaultFilePath = '';
defaultAxis = '';
expectedAxis = {'x','y','z'};

addOptional(p,'filepath',defaultFilePath)
addOptional(p,'value',value,@isnumeric);
addOptional(p,'axis',defaultAxis,...
                 @(x) any(validatestring(x,expectedAxis)));             
parse(p,varargin{:});

filepath = p.Results.filepath;
axis = p.Results.axis;
value = p.Results.value;

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

%% get the marker and force data into OpenSim tables
markers = tables.get('markers');
forces = tables.get('forces');

%% Rotate marker and force data
axis = 'x'; value = 90;
markers_rot = rotateTableData(markers, axis, value);
forces_rot = rotateTableData(forces, axis, value);

%% Print the rotated markers to trc file
TRCFileAdapter().write(markers_rot,[filename '.trc']);

%% Print the force data as a Vec3 sto file and a flattened doubles sto file
% make postfix string vector for naming colomns
postfix = StdVectorString(); postfix.add('_x');postfix.add('_y');postfix.add('_z');

%% flatten the Vec3 table to a table of doubles. 
forces_flattened = forces_rot.flatten(postfix);
% make a sto adapter and write the forces table to file.
STOFileAdapter().write(forces_flattened,[filename '.mot']);

MOTfileadapter()

end

