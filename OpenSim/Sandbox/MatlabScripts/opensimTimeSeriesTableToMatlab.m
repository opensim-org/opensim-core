function data = osimTableToStruct(table)


%%
import org.opensim.modeling.*
%%
nCol = table.getNumColumns;
nRow = table.getNumRows;

%% 
data = struct;

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


