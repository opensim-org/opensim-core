function osimC3Dconverter(varargin)
% Utility function for converting c3d data to OpenSim format.
% Inputs to function are pairs of string-string or string-number pairs. 
% 'filepath', 'path2file'   reads c3d data at path2file.
% 'axis', 'X'               applies first rotation to axis X (string)
% 'value', 90               value, in degrees, to rotate data by
%
%
% Example: read WalkingData.c3d and perform a 90 degree rotation about X
%   osimC3Dconverter('filepath', 'C:/data/WalkingData.c3d',...
%                 'value', '90'...
%                 'axis', 'X')

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
% Author: James Dunne

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

%% Check for file path
if isempty(filepath)
        [filein, path] = uigetfile({'*.c3d','C3D file'}, 'Select C3D data file(s)...', 'MultiSelect', 'on');
else
    % If the input path to file is wrong, return exception
    if exist(filepath,'file') == 0
            error('Path to input file is incorrect or file does not exist')
    else    
        % If the input path is local (called from current folder), you will
        % need to set the full path. 
        [path, filename, ext] = fileparts(filepath);
        if isempty(path)
            path = cd;
        end
        filein = [filename ext];
    end
end

%% 
if iscell(filein)
    nFiles = length(filein);
    multipleFiles = 1;
else
    nFiles = 1;
    multipleFiles = 0;
end
    

%% 
for iFile = 1 : nFiles
    
    if multipleFiles == 1
        % get the file name (with extension)
        filename = filein{iFile};
    else 
        filename = filein;
    end    
        
     % get the fullpath to the file 
    filepath = fullfile(path,filename);
    % get the file name (without the extension)
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
    if isempty(axis)
        display('Either no axis or value for rotation has been been set; no rotations will be applied')
        forces_rot = forces.clone();
        markers_rot = markers.clone();
    else
        display(['Rotating Marker and Force data ' num2str(value) ' about the ' axis ' axis']); 
        markers_rot = osimRotateTableData(markers, axis, value);
        forces_rot = osimRotateTableData(forces, axis, value);
    end
    
    %% Print the rotated markers to trc file
    TRCFileAdapter().write(markers_rot,fullfile(path,[filename '.trc']));
    display([[filename '.trc'] ' written to dir: ' path]);
    %% Print the force data as a Vec3 sto file and a flattened doubles sto file
    % make postfix string vector for naming colomns

    %% flatten the Vec3 table to a table of doubles. 
    postfix = StdVectorString(); postfix.add('_x');postfix.add('_y');postfix.add('_z');
    forces_flattened = forces_rot.flatten(postfix);

    % Tge GUI (~3.3) looks for force labels containing '_v'. We add
    % the char 'v' here. 
    labels = forces_flattened.getColumnLabels();
    newlabels = forces_flattened.getColumnLabels();

    for i = 0 : labels.size() - 1

        label = char(labels.get(i));

        if i < 9
            n = '1';
        elseif  8 < i < 18;
            n = '2';
        elseif 17 < i < 27;
           n = '3';
        end
        
       if ~isempty(strfind(label,'f'))
            s = ['ground_force_' n '_v'];
        elseif ~isempty(strfind(label,'p'))
            s = ['ground_force_' n '_p'];
        elseif ~isempty(strfind(label,'m'))
            s = ['ground_torque_' n '_m'];
        else
            error(['Column name ' label ' isnt recognized as a force, point, or moment'])
        end
            % Get the index for the underscore
            % in = strfind(label,'_');
            % add the specifier (f,p,or m) to the label name. 
            %label_new = [label(1:in) s label(in+1:end)];
            label_new = [s label(end)];

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
    
    %% Check for NaNs
    % If there are any NaNs in the forces data, OpenSim will interpret them as
    % the 'end' of the file. We must set NaNs to zero and report where they are
    % found
    for i = 0 : forces_flattened.getNumColumns() - 1
        dependentData = forces_flattened.getDependentColumnAtIndex(i);
        for u = 0 : forces_flattened.getNumRows() - 1
            if isnan(dependentData.get(u))
                % Display NaN value to console
                warning(['Nan values found for ' char(forces_flattened.getColumnLabels().get(i)) ', at row(s) ' num2str(u) '. Replacing with zeros'])
                % replace with 0;
                dependentData.set(u,0)
            end
        end
    end
    
    %% Change the header in the file to meet Storage conditions
    if forces_flattened.getTableMetaDataKeys().size() > 0
        for i = 0 : forces_flattened.getTableMetaDataKeys().size() - 1
            % get the metakey string at index zero. Since the array gets smaller on
            % each loop, we just need to keep taking the first one in the array. 
            metakey = char(forces_flattened.getTableMetaDataKeys().get(0));
            % remove the key from the meta data
            forces_flattened.removeTableMetaDataKey(metakey)
        end
    end

    % Add the column and row data to the meta key    
    forces_flattened.addTableMetaDataString('nColumns',num2str(forces_flattened.getNumColumns()+1))
    forces_flattened.addTableMetaDataString('nRows',num2str(forces_flattened.getNumRows()));

    %% make a sto adapter and write the forces table to file.
    STOFileAdapter().write(forces_flattened,fullfile(path,[filename '.mot']));
    display([[filename '.mot'] ' written to dir: ' path]);
end

end











































