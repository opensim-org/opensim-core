function table_rotated = rotateTableData(table, axis, value)

%% import java libraries
import org.opensim.modeling.*

%% set up the transform
if strcmp(axis, 'x')
    coordinate = CoordinateAxis(0)
elseif strcmp(axis, 'y')
    coordinate = CoordinateAxis(1)
elseif strcmp(axis, 'z')
    coordinate = CoordinateAxis(2)
else
    error(['axis value must be either x,y or z'])
end

%% instantiate a transform object
r = Rotation( deg2rad(value) , coordinate ) ;

%% rotate the values in each row

% make a table copy
table_rotated = table;

for iRow = 0 : table_rotated.getNumRows() - 1
    % get a row from the table
    row = table_rotated.getRowAtIndex(iRow);
    % pass the row to Rotation. 
    row_rotated = r.multiply(row);
    % override the row value
    table_rotated.setRowAtIndex(iRow,row_rotated)
end

end







