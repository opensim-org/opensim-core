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

%% flatten the Vec3 table to a table of doubles. 
postfix = StdVectorString(); postfix.add('_x');postfix.add('_y');postfix.add('_z');
forces_flattened = forces_rot.flatten(postfix);

% update the column names to have v,p,m n them.

labels = forces_flattened.getColumnLabels();
newlabels = forces_flattened.getColumnLabels();

for i = 0 : labels.size() - 1
    
    label = char(labels.get(i));
    
    if ~isempty(strfind(label,'f'))
        s = 'v';
    elseif ~isempty(strfind(label,'p'))
        s = 'p';
    elseif ~isempty(strfind(label,'m'))
        s = 'm';
    else
        error(['Column name ' label ' isnt recognized as a force, point, or moment'])
    end
        % Get the index for the underscore
        in = strfind(label,'_');
        % add the specifier (f,p,or m) to the label name. 
        label_new = [label(1:in) s label(in+1:end)];
        
        % update the label name 
        newlabels.set(i,label_new);
end
% set the column labels
forces_flattened().setColumnLabels(newlabels)

%% Change the points from mm to m
for i = 0 : labels.size() - 1
    label = char(forces_flattened.getColumnLabels().get(i));
    if ~isempty(strfind(label,'p'))
        for u = 0 : forces_flattened.getNumRows()-1
            % Get the point data
            point = forces_flattened.getDependentColumnAtIndex(i).get(u);
            % Divide the point by 1000 and set it back into the colomn
            forces_flattened.getDependentColumnAtIndex(i).set(u,point/1000);
        end
    end
end

%% Change the header in the file to meet Storage conditions
for i = 0 : forces_flattened.getTableMetaDataKeys().size() - 1
    % get the metakey string at index zero. Since the array gets smaller on
    % each loop, we just need to keep taking the first one off. 
    metakey = char(forces_flattened.getTableMetaDataKeys().get(0));
    % remove the key from the meta data
    forces_flattened.removeTableMetaDataKey(metakey)
end
    
% Add the column and row data to the meta day    
forces_flattened.addTableMetaDataString('nColumns',num2str(forces_flattened.getNumColumns()+1))
forces_flattened.addTableMetaDataString('nRows',num2str(forces_flattened.getNumRows()));

%% make a sto adapter and write the forces table to file.
STOFileAdapter().write(forces_flattened,[filename '.mot']);


end











































