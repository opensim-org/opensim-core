
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
row1 = RowVector(5, 3.24 );
row2 = RowVector(5, 4.6 );
row3 = RowVector(5, 9.2 );
row4 = RowVector(5, 1.112 );


table.appendRow(0, row0);
table.appendRow(0.25, row1);
table.appendRow(0.5, row2);
table.appendRow(0.75, row3);
table.appendRow(1, row4);



%% Edit individual elements of either a RowVector ir table.  


 row = RowVector([1 2 3 4])




