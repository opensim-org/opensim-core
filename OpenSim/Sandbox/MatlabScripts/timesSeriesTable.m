function output_table = timesSeriesTable(generictable)
%% // Convert between OpenSim and Matlab Timeseries 
%  
%   Inputs can be either an OpenSim TimeSeriesTable, TimesSeriesTableVec3,
%   or a Matlab times series collection. 
%
%   Outputs from OpenSim TimesSeriesTable's will be a Matlab times series
%   collection. 
%
%   Function does not maintain meta data between types. 
%

% Author: James Dunne, Shrinidhi K. Lakshmikanth, Chris Dembia, Tom Uchida,
% Ajay Seth, Ayman Habib, Jen Hicks. 

import org.opensim.modeling.*

if strcmp( class(generictable), 'org.opensim.modeling.TimeSeriesTableVec3') ...
   || strcmp( class(generictable), 'org.opensim.modeling.TimeSeriesTable') 
   
    output_table = table2tsc(generictable);

elseif  strcmp( class(generictable), 'tscollection') 
   
    output_table = tsc2table(generictable);
    
else
    error('input must either be an OpenSim TimeSeriesTable or a Matlab tscollection')
end

end


function tsc = table2tsc(generictable) 
    % convert a OpenSim timeSeriesTable to a Matlab timeseries object
    import org.opensim.modeling.*

    %% get the number or rows and columns
    nCol = generictable.getNumColumns;
    nRow = generictable.getNumRows;

    %% instantiate an empty table
    tsc = tscollection();

    %% determine temp storage array size. 
    if strcmp( char(generictable.getClass), 'class org.opensim.modeling.TimeSeriesTableVec3')
        rowdata = zeros(nRow, 3);
    elseif strcmp( char(generictable.getClass), 'class org.opensim.modeling.TimeSeriesTable')
         % if data is double
        rowdata = zeros(nRow, 1);
    else
        error('unkown class type: must either be a TimeSeriesTable or TimeSeriesTableVec3')
    end

    % get the time column. 
    for iRow = 0 : nRow - 1
        time(iRow+1,1) = generictable.getIndependentColumn.get(iRow);
    end 


    for iCol = 0 : nCol -1

        % get the data column name
        col_label  = char(generictable.getColumnLabels.get(iCol));

        if strcmp( char(generictable.getClass), 'class org.opensim.modeling.TimeSeriesTableVec3');

            for iRow = 0 : nRow - 1
                rowdata(iRow+1,1) = generictable.getDependentColumnAtIndex(iCol).getElt(iRow,1).get(0);
                rowdata(iRow+1,2) = generictable.getDependentColumnAtIndex(iCol).getElt(iRow,1).get(1);
                rowdata(iRow+1,3) = generictable.getDependentColumnAtIndex(iCol).getElt(iRow,1).get(2);       
            end

        else strcmp( char(generictable.getClass), 'class org.opensim.modeling.TimeSeriesTable');

            for iRow = 0 : nRow - 1
                rowdata(iRow+1,1) = generictable.getDependentColumnAtIndex(iCol).getElt(iRow,0);
            end
        end

        % append column (as a table) to the existing table. 
        tso = timeseries(rowdata,time,'Name',col_label);
        tsc = addts(tsc, tso);

    end

end

function timeseriestable = tsc2table(tsc)
    % convert a Matlab timeseries object to a OpenSim timeSeriesTable 

    import org.opensim.modeling.*

    %% get the column names
    labelnames = tsc.gettimeseriesnames;

    %% data size
    dataSize = tsc.size();
    nRow = dataSize(1); nlabels = dataSize(2);
    [m, nCol] = size(tsc.(labelnames{1}).Data);
    
    %% empty opensim table
    if nCol == 1
        table = DataTable();
    elseif nCol == 3
        table = DataTableVec3();
    else 
        error('data must have either one or three columns')
    end

    %% set the table column names
    labels =  StdVectorString();
    for i = 1 : nlabels
        labels.add( labelnames{i} );    
    end
    table.setColumnLabels(labels);

    %% get a pointer to the time column
    timeColumn = table.getIndependentColumn();

    if nCol == 1
        % If the data is in doubles, then build a table of doubles. 
        row = RowVector(nlabels, 1);
        
        for iRow = 1 : nRow

            % create and fill a row of data at a time
            for iCol = 1 : nlabels

                rowdata = tsc.(labelnames{iCol}).Data(iRow,:);

                row.set(iCol-1, rowdata);
            end
        
            table.appendRow(iRow-1, row);
            % set the time value    
            timeColumn.set(iRow-1, double(tsc.Time(iRow)) );
        end
        
    elseif nCol == 3
        % if the data is in triples, then build a table of Vec3's
        
        
        for iRow = 1 : nRow
    
            elems = StdVectorVec3();
            
            % create and fill a row of data at a time
            for iCol = 1 : nlabels

                rowdata = tsc.(labelnames{iCol}).Data(iRow,:);

                elem = Vec3(rowdata(1), rowdata(2), rowdata(3));

                elems.add(elem); 
            end
        
            row = RowVectorOfVec3(elems);

            table.appendRow(iRow-1, row);
            % set the time value    
            timeColumn.set(iRow-1, double(tsc.Time(iRow)) )   
        end
    end
    
   % convert the data table to a timesseriestable
    if nCol == 1
        timeseriestable = TimeSeriesTable(table);
    elseif nCol == 3
        timeseriestable = TimeSeriesTableVec3(table);
    end

end

% ----------------------------------------------------------------------- %
% The OpenSim API is a toolkit for musculoskeletal modeling and           %
% simulation. See http://opensim.stanford.edu and the NOTICE file         %
% for more information. OpenSim is developed at Stanford University       %
% and supported by the US National Institutes of Health (U54 GM072970,    %
% R24 HD065690) and by DARPA through the Warrior Web program.             %
%                                                                         %   
% Copyright (c) 2005-2012 Stanford University and the Authors             %
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
