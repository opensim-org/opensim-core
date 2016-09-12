function data = opensimVec3TableToStruct(vec3table)

import org.opensim.modeling.*


nCol = vec3table.getNumColumns;
nRow = vec3table.getNumRows;

data = struct;
rowdata = zeros(nRow, 3);

for iCol = 0 : nCol -1
    for iRow = 0 : nRow - 1
        rowdata(iRow+1,1) = vec3table.getDependentColumnAtIndex(iCol).getElt(iRow,1).get(0);
        rowdata(iRow+1,2) = vec3table.getDependentColumnAtIndex(iCol).getElt(iRow,1).get(1);
        rowdata(iRow+1,3) = vec3table.getDependentColumnAtIndex(iCol).getElt(iRow,1).get(2);       
    end
    
    col_label  = char(vec3table.getColumnLabels.get(iCol));

    if isempty(strfind(col_label, '/'))
        eval(['[data.' col_label '] = rowdata;']);
    else
       temp = strfind(col_label, '/');
       new_col_label = col_label(temp(end-1)+1:temp(end)-1);
       eval(['[data.' new_col_label '] = rowdata;']);
    end
    
    
end

time = zeros(nRow,1);

for iRow = 0 : nRow - 1
    time(iRow+1,1) = vec3table.getIndependentColumn.get(iRow);
end 

[data.time] = time;