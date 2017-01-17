



clear all; close all; clc

%% import opensim libraries 
import org.opensim.modeling.*

%% make a blank data table
table = DataTable;

%% make a vector of strings that will be used to make column labels
colnames = StdVectorString;
colnames.add('data_1')
colnames.add('data_2')
colnames.add('data_3')
colnames.add('data_4')
colnames.add('data_5')

%% Set the column label names
table.setColumnLabels(colnames);

%% create some row arbitary row vectors 
row0 = RowVector(5, 0); 
row1 = RowVector(5, 0.25 );
row2 = RowVector(5, 0.50 );
row3 = RowVector(5, 0.75 );
row4 = RowVector(5, 1 );

%% append the rows to the table
table.appendRow(0, row0);
table.appendRow(0, row1);
table.appendRow(0, row2);
table.appendRow(0, row3);
table.appendRow(0, row4);

%% Retrieve a column by its label and by the label
col3 = table.updDependentColumnAtIndex(0);
col2 = table.updDependentColumn('data_2');

%% dump the column data into a matlab array
data = [];
for iRow = 0 : table.getNumRows - 1
    data = [data; col3.get(iRow)];    
end    

%% do transformation and write the data back to the OpenSim table
data_new = data * 1000;
for iRow = 0 : table.getNumRows - 1
    col3.set( iRow , data_new(iRow + 1) );    
end 





% Following construction of TimeSeriesTable succeeds because the DataTable
% has its independent column strictly increasing.
timeseriestable = TimeSeriesTable(table);

% Editing the DataTable to not have strictly increasing independent column
% will fail the construction of TimeSeriesTable.

table.appendRow(0.9,row3)

try
   timeseriestable = TimeSeriesTable(table);
catch exception
   'Times series table exception: Time has to be increasing'
end

% Edit the entry in the independent column to make the column strictly
% increasing.
table.setIndependentValueAtIndex(5, 1.25);

try
   timeseriestable = TimeSeriesTable(table);
catch exception
   'Times series table exception: Time has to be increasing'
end