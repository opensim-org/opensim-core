function data = osimTableToStruct(table)
%% // Convert Matlab Struct to OpenSim time Series Table 
%  Input is an OpenSim TimesSeriesTable
%
%  Output is a Maltab stucture where data.lable = n X 1 or n x 3 array
%  ie structdata.LASI = [2 3 4; 4 5 6, ...

% Author: James Dunne, Shrinidhi K. Lakshmikanth, Chris Dembia, Tom Uchida,
% Ajay Seth, Ayman Habib, Jen Hicks. 

%%
import org.opensim.modeling.*
%%
nCol = table.getNumColumns;
nRow = table.getNumRows;

%% make an empty data structure
data = struct();

%%
if strcmp( char(table.getClass), 'class org.opensim.modeling.TimeSeriesTableVec3')
    rowdata = zeros(nRow, 3);
elseif strcmp( char(table.getClass), 'class org.opensim.modeling.TimeSeriesTable')
    rowdata = zeros(nRow, 1);
end

%%
for iCol = 0 : nCol -1

    if strcmp( char(table.getClass), 'class org.opensim.modeling.TimeSeriesTableVec3')

        for iRow = 0 : nRow - 1
            rowdata(iRow+1,1) = table.getDependentColumnAtIndex(iCol).getElt(iRow,1).get(0);
            rowdata(iRow+1,2) = table.getDependentColumnAtIndex(iCol).getElt(iRow,1).get(1);
            rowdata(iRow+1,3) = table.getDependentColumnAtIndex(iCol).getElt(iRow,1).get(2);       
        end

    elseif strcmp( char(table.getClass), 'class org.opensim.modeling.TimeSeriesTable')

        for iRow = 0 : nRow - 1
            rowdata(iRow+1,1) = table.getDependentColumnAtIndex(iCol).getElt(iRow,0);
        end
    end

    col_label  = char(table.getColumnLabels.get(iCol));

    if isempty(strfind(col_label, '/'))
        eval(['[data.' col_label '] = rowdata;']);
    else
       temp = strfind(col_label, '/');
       new_col_label = col_label(temp(end-1)+1:temp(end)-1);
       eval(['[data.' new_col_label '] = rowdata;']);
    end
end

%%
time = zeros(nRow,1);

for iRow = 0 : nRow - 1
    time(iRow+1,1) = table.getIndependentColumn.get(iRow);
end 

[data.time] = time;


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
