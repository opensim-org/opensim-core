% ----------------------------------------------------------------------- %
% The OpenSim API is a toolkit for musculoskeletal modeling and           %
% simulation. See http://opensim.stanford.edu and the NOTICE file         %
% for more information. OpenSim is developed at Stanford University       %
% and supported by the US National Institutes of Health (U54 GM072970,    %
% R24 HD065690) and by DARPA through the Warrior Web program.             %
%                                                                         %
% Copyright (c) 2005-2016 Stanford University and the Authors             %
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

%% Convert Matlab Struct to OpenSim time Series Table
%  Input is a Maltab stucture where data.label = nX1 or nX3 array
%  eg structdata.LASI = [2 3 4; 4 5 6, ...
%  One of the structures values MUST be a time vector and called 'time'
%  Output is an OpenSim TimesSeriesTable

% Author: James Dunne, Tom Uchida, Shrinidhi K. Lakshmikanth, Chris Dembia, 
% Ajay Seth, Ayman Habib, Jen Hicks.

function timeseriesosimtable = osimTableFromStruct(structdata)
% import Java Libraries
import org.opensim.modeling.*

% Each column of data has a label. We wiget all the data labels
labelnames = fieldnames(structdata);
% get the index for the time array
timeIndex = find(cellfun(@(s) ~isempty(strfind('time', s)),lower(labelnames)));

%% Check the dimensions of the structure confirm to requirements
if isempty(timeIndex)
    error('Time field required ? none found')
end

%% Check the structure for 'shape' consistency across fields. 
nRowsList = []; nColsList = [];

for i = 1 : length(labelnames)
    % The special case is for the time field. It should always be an 
    % nX1 array. the number of rows (n) should will need to bec checked
    % against all the rest of the fields for consistency. 
    if i == timeIndex
        % Check that the time field has the correct number of columns
        [nRows,nCol] = size(structdata.(labelnames{timeIndex}));
        if nCol ~= 1
            error(['time field is a nX' num2str(nCol) ' array - nX1 required'])
        end
        % add the number of rows to a list to be checked
        nRowsList = [nRowsList nRows];
    else
        % for all other fields, we want to check that the array size is
        % either nX1 or nX3, other array sizes are unsupported. The number
        % of rows and columns of each field array must be checked for
        % consistency. 
        [nRows,nCol] = size(structdata.(labelnames{i}));

        if nCol ~= 1 && nCol ~= 3
            error(['Field .' labelnames{i} ' is nX' num2str(nCol) ' - must be nX1 or nX3'])
        end
        % add the number of rows to a list to be checked for consistency
        nRowsList = [nRowsList nRows];
        % add the number of columns to a list to be checked for consistency
        nColsList = [nColsList nCol];
    end
    
    % check to see if nRows is uniform
    if ~isempty(find( nRowsList ~= nRowsList(1)) )
        error(['Number of rows is non-uniform across fields. Check .' labelnames{i} ' for error'])
    end
    
    % check to see if nRows is uniform
    if ~isempty(find( nColsList ~= nColsList(1)) )
        error(['Number of columns is non-uniform across fields. Check .' labelnames{i} ' for error'])
    end        
end
  
%% make an array of index numbers that will be used in the main For loop
indexArray = [0 : length(labelnames) - 1];
indexArray(timeIndex) = [];
nLabels = length(indexArray);

%% Pre-allocate an empty data table from data type. Also set some flags. 
if max(nColsList) == 3
    vec3table = true;
    osimtable = DataTableVec3();
else
    vec3table = false;
    osimtable = DataTable();
end

%% set the osimtable column names
labels =  StdVectorString();
for i = indexArray
    labels.add( labelnames{i+1} );
end
osimtable.setColumnLabels(labels);

%% get a pointer to the time column
timeColumn = osimtable.getIndependentColumn();

if vec3table == true
    % If the data is nX3, then build an osimtable of Vec3's.
    % We build the osimtable row by row, filling a row vector from left to
    % right and then appending the row to the OpenSim table.
   for iRow = 1 : nRows
        % elems will be a vec3 vector that we will append elements to
        elems = StdVectorVec3();
        % create and fill a row of data
        for iCol = indexArray
            % get the data from the input structure
            rowdata = structdata.(labelnames{iCol+1})(iRow,:);
            % make a vec3 element from the rowdata
            elem = Vec3(rowdata(1), rowdata(2), rowdata(3));
            % append the Vec3 element to the vec3 vector
            elems.add(elem);
        end
        % type change the elems vector to a RowVectorofVec3's
        row = RowVectorOfVec3(elems);
        % append the RowVectorofVec3's to the opensim table
        osimtable.appendRow(iRow-1, row);
        % set the time value for the appended row
        timeColumn.set(iRow-1, structdata.(labelnames{timeIndex})(iRow));
    end
    
else
    % Else the data is in doubles, then build a osimtable of doubles.
    % We build the osimtable row by row, filling a row vector from left to
    % right and then appending the row to the OpenSim osimtable. 
    row = RowVector(nLabels, NaN);

    for iRow = 1 : nRows
        % create and fill a row of data 
        for iCol = indexArray
            % get double from col and row index.
            rowdata = structdata.(labelnames{iCol+1})(iRow,:);
            % update the value in the row. 
            row.set(iCol, rowdata);
        end
        % Append the row vector to the opensim table
        osimtable.appendRow(iRow-1, row);
        % set the time value
        timeColumn.set(iRow-1, structdata.(labelnames{timeIndex})(iRow) );
    end
end

% type change the DataTable to a TimeSeriesTable 
if vec3table == true
   timeseriesosimtable = TimeSeriesTableVec3(osimtable);
else
    timeseriesosimtable = TimeSeriesTable(osimtable);
end

end
