import org.opensim.modeling.*


table = DataTableVec3()
% Set columns labels.

colnames = StdVectorString;

colnames.add('0')
colnames.add('1')
colnames.add('2')


table.setColumnLabels(colnames);


%% nothing below this line works....
row = RowVectorOfVec3([Vec3(1, 2, 3),Vec3(4, 5, 6),Vec3(7, 8, 9)])



table.appendRow(0.1, row)
assert table.getNumRows() == 1
assert table.getNumColumns() == 3
row0 = table.getRowAtIndex(0)
assert (str(row0[0]) == str(row[0]) and
        str(row0[1]) == str(row[1]) and
        str(row0[2]) == str(row[2]))
% Append another row to the table.
row = RowVectorOfVec3([Vec3( 2,  4,  6), 
                            Vec3( 8, 10, 12),
                            Vec3(14, 16, 18)])
table.appendRow(0.2, row)

row1 = table.getRow(0.2)

% Append another row to the table.
row = RowVectorOfVec3([Vec3( 4,  8, 12), 
                            Vec3(16, 20, 24),
                            Vec3(28, 32, 36)])
table.appendRow(0.3, row)

row2 = table.getRow(0.3)

% Retrieve independent column.