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

%% // Convert Matlab Struct to OpenSim time Series Table
%  Input is a Maltab stucture where data.lable = n X 1 or n x 3 array
%  ie structdata.LASI = [2 3 4; 4 5 6, ...
%  One of the structures values MUST be a time vector and called 'time'
%  Output is an OpenSim TimesSeriesTable

% Author: James Dunne, Shrinidhi K. Lakshmikanth, Chris Dembia, Tom Uchida,
% Ajay Seth, Ayman Habib, Jen Hicks.


function timeseriestable = osimTableFromStruct(structdata)

import org.opensim.modeling.*

% get all the data labels
labelnames = fieldnames(structdata);
% get the index for the time array
timeIndex = find(cellfun(@(s) ~isempty(strfind('time', s)),lower(labelnames)));

% get n col and rows
if timeIndex == 1
    [nRows,nCol] = size(structdata.(labelnames{2}));
else
    [nRows, nCol] = size(structdata.(labelnames{1}));
end

% throw an error if the number of columns is neither 1 nor 3
if nCol ~= 1 && nCol ~= 3
    error('data must have one or three columns')
end

indexArray = [1 : length(labelnames)];
indexArray(timeIndex) = [];
nLabels = length(indexArray);

%% make an empty table
if nCol == 1
    table = DataTable();
else
    table = DataTableVec3();
end

%% set the table column names
labels =  StdVectorString();
for i = indexArray
    labels.add( labelnames{i} );
end
table.setColumnLabels(labels);

%% get a pointer to the time column
timeColumn = table.getIndependentColumn();

if nCol == 1
    % If the data is in doubles, then build a table of doubles.
    row = RowVector(nLabels, 1);

    for iRow = 1 : nRows

        % create and fill a row of data at a time
        for iCol = indexArray

            rowdata = structdata.(labelnames{iCol})(iRow,:);

            row.set(iCol-1, rowdata);
        end

        table.appendRow(iRow-1, row);
        % set the time value
        timeColumn.set(iRow-1, structdata.(labelnames{timeIndex})(iRow) );
    end

else nCol == 3
    % if the data is in triples, then build a table of Vec3's


    for iRow = 1 : nRows

        elems = StdVectorVec3();

        % create and fill a row of data at a time
        for iCol = indexArray

            rowdata = structdata.(labelnames{iCol})(iRow,:);

            elem = Vec3(rowdata(1), rowdata(2), rowdata(3));

            elems.add(elem);
        end

        row = RowVectorOfVec3(elems);

        table.appendRow(iRow-1, row);
        % set the time value
        timeColumn.set(iRow-1, structdata.(labelnames{timeIndex})(iRow));
    end
end

% convert the data table to a timesseriestable
if nCol == 1
    timeseriestable = TimeSeriesTable(table);
elseif nCol == 3
    timeseriestable = TimeSeriesTableVec3(table);
end

end
