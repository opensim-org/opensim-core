%% Convert Matlab Struct to OpenSim time Series Table
%  Input is an OpenSim TimesSeriesTable or TimesSeriesTableVec3
%
%  Output is a Maltab stucture where data.label = nX1 or nx3 array
%  eg structdata.LASI = [2 3 4; 4 5 6, ...

% ----------------------------------------------------------------------- %
% The OpenSim API is a toolkit for musculoskeletal modeling and           %
% simulation. See http://opensim.stanford.edu and the NOTICE file         %
% for more information. OpenSim is developed at Stanford University       %
% and supported by the US National Institutes of Health (U54 GM072970,    %
% R24 HD065690) and by DARPA through the Warrior Web program.             %
%                                                                         %
% Copyright (c) 2005-2019 Stanford University and the Authors             %
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
%         Ajay Seth, Ayman Habib, Jen Hicks, Apoorva Rajagopal.

%%
function structdata = osimTableToStruct(osimtable)

% Import Java libraries 
import org.opensim.modeling.*

%% Type check the input variables 
if strcmp( char(osimtable.getClass()), 'class org.opensim.modeling.TimeSeriesTableVec3')
        vec3Table = true;

elseif strcmp( char(osimtable.getClass()), 'class org.opensim.modeling.TimeSeriesTable')
        vec3Table = false;
        
else
    disp(['Input is of type ' char(osimtable.getClass()) ])
    error(['This function only converts TimeSeriesTable and TimeSeriesTableVec3']);
end

%% get the number of data columns as label headers and data rows
nLabels = osimtable.getNumColumns(); 
nRows = osimtable.getNumRows();

%% pre-allocate data arrarys. 
structdata = struct();
if vec3Table == true
    dataArray = NaN([nRows 3]);
else
    dataArray = NaN([nRows 1]);
end

%% cycle through columns and rows to make new arrays, then addd then to struct. 
for iLabel = 0 : nLabels - 1
    % This works for both TimeSeriesTable and TimeSeriesTableVec3.
    dataArray = osimtable.getDependentColumnAtIndex(iLabel).getAsMat();

    % Get the osim table column label
    col_label  = char(osimtable.getColumnLabels.get(iLabel));
    disp_label = col_label;
    % MATLAB variable names must start with a letter, can only contain letters,
    % digits, and underscores, and be under a maximum length
    if ~isvarname(col_label)
        % Find any non-alphanumeric characters and replace with '_'
        col_label(~(isstrprop(col_label,'alphanum'))) = '_';
        % Check if first character is a letter, and prepend 'a_' if not.
        if ~(isletter(col_label(1)))
            col_label = ['a_' col_label(2:end)];
        end
        % Last check for too long a name 
        % Have user input a new name, and make them keep doing it until the
        % variable name is valid
        while ~isvarname(col_label)
            disp(['Error: ', disp_label ' is an invalid label name; enter a new name that:'])
            disp('  - starts with a letter')
            disp('  - contains only letters, numbers, and/or underscores (_)')
            disp(['  - is no longer than ', num2str(namelengthmax), ' characters'])
            col_label = input('New name (no quotes): ', 's');
        end
    end
    % If the label has changed, inform the User.
    if ~strcmp(disp_label,col_label)
        disp(['Illegal Column label. ' disp_label ' changed to ' col_label ]); 
    end
    % Add the label and data to the data struct
    structdata.(col_label) = dataArray;
end

% Get the time
time = NaN([nRows 1]);
% read time data from table
for iRow = 0 : nRows - 1
    time(iRow+1,1) = osimtable.getIndependentColumn.get(iRow);
end
% add time field to structure
[structdata.time] = time;

end
