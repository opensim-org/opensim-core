%% Convert Matlab Struct to OpenSim time Series Table
%  Input is a Maltab stucture where data.label = nX1 or nX3 array
%  eg s.LASI = [2 3 4; 4 5 6, ...
%  One of the structures values MUST be a time vector and called 'time'
%  Output is an OpenSim TimesSeriesTable

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

% Written by: James Dunne, Tom Uchida, Chris Dembia, Ajay Seth,
%                   Ayman Habib, Jen Hicks,Shrinidhi K. Lakshmikanth.

%%
function timeseriesosimtable = osimTableFromStruct(s)
%% import Java Libraries
import org.opensim.modeling.*

%% Get all the data labels in the Structure
labels = fieldnames(s);

%% Check for, save, then remove the time array from the structure
% Get the index for time array
tIndex = find(cellfun(@(s) contains('time', s),lower(labels)));
% Check to see if the time exists
if isempty(tIndex)
    error('"time" field required, none found')
end
% Get the time vector
time = s.time();
% Remove time from the struct
s = rmfield(s,'time');
% remove time from the labels
labels(tIndex) = [];
nfields = length(labels);

%% Check the structure for row and column length consistency across fields. 
for i = 1 : nfields
    % For all fields in s, check that the array size is either nX1 or nX3,
    % other array sizes are unsupported. 
    [nRows,nCols] = size(s.(labels{i}));
    
    if i == 1
        rowRef = nRows; colRef = nCols;
        % Check that the first field has 1 or 3 Columns
        if ~(colRef == 1 || colRef == 3)
           error(['Data Columns must be 1 or 3, s.' labels{i} ' has ' num2str(colRef)])
        end
    else
        % Check that the current field has the same number of rows and
        % columns as the reference. 
        if rowRef ~= nRows || colRef ~= nCols
            error('Array rows or columns are non-uniform across fields. Check input stucture for error')
        end
    end
end

%% Instantiate an empty OpenSim TimesSeriesTable()
if colRef == 1
    timeseriesosimtable = TimeSeriesTable();
else
    timeseriesosimtable = TimeSeriesTableVec3();
end

% Set the TimesSeriesTable() column names
osimlabels =  StdVectorString();
for i = 1 : nfields
    osimlabels.add( labels{i} );
end
timeseriesosimtable.setColumnLabels(osimlabels);

%% Build the TimesSeriesTable()
if colRef == 1
    % Get an OpenSim Row Vector
    row = RowVector(nfields, NaN);
    % Fill row vector with data 
    for iRow = 1 : nRows 
        for iCol = 1 : nfields 
           % Get the row value from each field of the struct
           row.set(iCol-1, s.(labels{iCol})(iRow) );
        end
        % Append the row vector to the opensim table
        timeseriesosimtable.appendRow(iRow-1, row);
    end
else
    % Get an OpenSim Row Vector
    row = RowVectorVec3(nfields);
    for iRow = 1 : nRows
        % Create and fill a row of data
        for iCol = 1 : nfields 
            % Make a vec3 element from the rowdata
            row.set(iCol-1, osimVec3FromArray(s.(labels{iCol})(iRow,:)));
        end
        % Append the RowVectorofVec3's to the opensim table
        timeseriesosimtable.appendRow(iRow-1, row);
    end
end

%% Set the Time Column values
timeColumn = timeseriesosimtable.getIndependentColumn();
for i = 1 : nRows 
      timeColumn.set(i-1, time(i));
end

end
