function data = opensimTimeSeriesTableToMatlab(table)


nCol = table.getNumColumns;
nRow = table.getNumRows;

data = struct;
rowdata = zeros(nRow, 3);

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

    if ~isempty(strfind(col_label, '/')) || ~isempty(strfind(col_label, '|'))
        % Remove an initial slash.
        col_label = regexprep(col_label, '^/', '');
        % Replace all other slashes or vertical bars with an underscore.
        col_label = strrep(col_label, '/', '_');
        col_label = strrep(col_label, '|', '_');
    end
    eval(['[data.' col_label '] = rowdata;']);


end

time = zeros(nRow,1);

for iRow = 0 : nRow - 1
    time(iRow+1,1) = table.getIndependentColumn.get(iRow);
end 

[data.time] = time;
    
