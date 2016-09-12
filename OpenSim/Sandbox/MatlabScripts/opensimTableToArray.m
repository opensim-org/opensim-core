function [data_array,labels] = opensimTableToArray(table)





import org.opensim.modeling.*

% allocate an array
nCol = table.getNumColumns;
nRow = table.getNumRows;
data = zeros(nRow, nCol);
labels = {};

for iCol = 0 : nCol - 1

    for iRow = 0 : nRow - 1

        data(iRow+1,iCol+1) = table.getDependentColumnAtIndex(iCol).getElt(iRow,1);

    end    
        
    labels = [labels {char(table.getColumnLabel(iCol))}];
    
end

time = zeros(nRow,1);

for iRow = 0 : nRow - 1

        time(iRow+1,1) = table.getIndependentColumn.get(iRow);

end   

data_array = [time data];
