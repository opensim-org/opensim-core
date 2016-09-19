

import org.opensim.modeling.*

new_table = TimeSeriesTable()

table = DataTable

colnames = StdVectorString;

colnames.isEmpty

colnames.add('0')
colnames.add('1')
colnames.add('2')
colnames.add('3')
colnames.add('4')

colnames.isEmpty
colnames.size

table.setColumnLabels(colnames);


row0 = RowVector(5, 0);
row1 = RowVector(5, 0.25 );
row2 = RowVector(5, 0.50 );
row3 = RowVector(5, 0.75 );
row4 = RowVector(5, 1 );


table.appendRow(0, row0);
table.appendRow(0, row1);
table.appendRow(0, row2);
table.appendRow(0, row3);
table.appendRow(0, row4);

% Retrieve a column by its label.
col3 = table.getDependentColumn('3');

col3.getElt(0,0)
col3.getElt(1,0)
col3.getElt(2,0)
col3.getElt(3,0)
col3.getElt(4,0)


col2 = table.getDependentColumn('2');

col2.getElt(0,0)
col2.getElt(1,0)
col2.getElt(2,0)
col2.getElt(3,0)
col2.getElt(4,0)
