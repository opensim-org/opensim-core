function tsc = timeSeriesTable(opensimtable)

import org.opensim.modeling.*
%% get the number or rows and columns
nCol = opensimtable.getNumColumns;
nRow = opensimtable.getNumRows;

%% instantiate an empty table
tsc = tscollection();

%% determine temp storage array size. 
if strcmp( char(opensimtable.getClass), 'class org.opensim.modeling.TimeSeriesTableVec3')
    rowdata = zeros(nRow, 3);
elseif strcmp( char(opensimtable.getClass), 'class org.opensim.modeling.TimeSeriesTable')
     % if data is double
    rowdata = zeros(nRow, 1);
else
    error('unkown class type: must either be a TimeSeriesTable or TimeSeriesTableVec3')
end

%%
 

for iRow = 0 : nRow - 1
    time(iRow+1,1) = opensimtable.getIndependentColumn.get(iRow);
end 


for iCol = 0 : nCol -1
    
    % get the data column name
    col_label  = char(opensimtable.getColumnLabels.get(iCol));
    
    if strcmp( char(opensimtable.getClass), 'class org.opensim.modeling.TimeSeriesTableVec3')
          
        for iRow = 0 : nRow - 1
            rowdata(iRow+1,1) = opensimtable.getDependentColumnAtIndex(iCol).getElt(iRow,1).get(0);
            rowdata(iRow+1,2) = opensimtable.getDependentColumnAtIndex(iCol).getElt(iRow,1).get(1);
            rowdata(iRow+1,3) = opensimtable.getDependentColumnAtIndex(iCol).getElt(iRow,1).get(2);       
        end
        
    else strcmp( char(opensimtable.getClass), 'class org.opensim.modeling.TimeSeriesTable')

        for iRow = 0 : nRow - 1
            rowdata(iRow+1,1) = opensimtable.getDependentColumnAtIndex(iCol).getElt(iRow,0);
        end
    end

    % append column (as a table) to the existing table. 
    tso = timeseries(rowdata,time,'Name',col_label);
    tsc = addts(tsc, tso);
    
end
