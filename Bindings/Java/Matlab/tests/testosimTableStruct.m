%% clear working space
clear all;close all;clc;
%% import opensim libraries
import org.opensim.modeling.*
%% Build an OpenSim Times Series Table Programmatically 
table = DataTableVec3();
labels = StdVectorString();
labels.add('marker1');labels.add('marker2');
labels.add('marker3');labels.add('marker4');
table.setColumnLabels(labels);

R = Rotation(pi/3, Vec3(0.1, 0.2, 0.3));

for i = 1 : 10
    elem = Vec3(randi(10,1),randi(10,1),randi(10,1));
    elems = StdVectorVec3();
    elems.add(elem); elems.add(elem); elems.add(elem); elems.add(elem);
    row = RowVectorVec3(elems);
    
    rotatedRow = R.multiply(row);    
    table.appendRow(0.1,rotatedRow);
end

% Set the indpendentColumn (Time) values
indCol = table.getIndependentColumn();
for i = 0 : 9
    indCol.set(i, i/100)
end

% Turn DataTable to a timesSeriesTable, then flatten
tsTable = TimeSeriesTableVec3(table);
tsTable_d = tsTable().flatten();

%% Convert to OpenSim Tables to Matlab data type
mData = osimTableToStruct(tsTable);
mData_d = osimTableToStruct(tsTable_d);
% Convert Matlab Structs back to OpenSim tables
tsTable_2 = osimTableFromStruct(mData);
tsTable_d_2 = osimTableFromStruct(mData_d);

%% Check the number of columns and rows are maintained
nCol = tsTable.getNumColumns();nRow = tsTable.getNumRows();
nCold = tsTable_d.getNumColumns();nRowd = tsTable_d.getNumRows();
nCol_2 = tsTable_2.getNumColumns();nRow_2 = tsTable_2.getNumRows();
nCold_2 = tsTable_d_2.getNumColumns();nRowd_2 = tsTable_d_2.getNumRows();

assert(nCol == nCol_2 & nRow == nRow_2,'Vec3 conversion OpenSim-Matlab_OpenSim incorrect: Rows or Columns not preserved');
assert(nCold == nCold_2 & nRowd == nRowd_2,'dbl conversion OpenSim-Matlab_OpenSim incorrect: Rows or Columns not preserved');

%% Check data across is conserved across conversions. 
for u = 0 : 9
    for i = 0 : 3
        % get the data from the element
        d = tsTable.getRowAtIndex(u).getElt(0,i);
        d2 = tsTable_2.getRowAtIndex(u).getElt(0,i);
        
        assert( d.get(0) == d2.get(0) & d.get(1) == d2.get(1) & d.get(2) == d2.get(2),['Data at row= ' num2str(u) ' & column= '  num2str(i) ' is not retained during conversion'])
    end
end

for u = 0 : 9
    for i = 0 : 11
        % get the data from the element
        d = tsTable_d.getRowAtIndex(u).getElt(0,i);
        d2 = tsTable_d_2.getRowAtIndex(u).getElt(0,i);
        
        assert(d == d2,['Data at row= ' num2str(u) ' & column= '  num2str(i) ' is not retained during conversion'])
    end
end

%% Add new Data to a struct, then convert into an OpenSim table
% Make copies of the data tables
s = mData_d; sV3 = mData; 
% Add new fields to the data
f = fieldnames(s);
for i = 1:length(f)-1
    s.([f{i} '_new']) = s.(f{i});
end
f = fieldnames(sV3);
for i = 1:length(f)-1
    sV3.([f{i} '_new']) = sV3.(f{i});
end

% Convert the new struct into a another table.
try 
   oTable   = osimTableFromStruct(s);
catch
    error('Failed to convert structure (of doubles) with added fields to table')
end

try 
    oTableV3 = osimTableFromStruct(sV3);
catch
    error('Failed to convert structure (of Vec3s) with added fields to table')
end

% Convert tables back into stucts
s_new           = osimTableToStruct(oTable);
sV3_new         = osimTableToStruct(oTableV3);

% Check that the number of fields have been preserved. 
assert(length(fieldnames(s)) == length(fieldnames(s_new)), 'Number of fields in s and s_new are different')
assert(length(fieldnames(sV3)) == length(fieldnames(sV3_new)), 'Number of fields in sV3 and sV3_new are different')

%% Check column names can be converted to Matlab structure fields
labels = oTable.getColumnLabels();
labels.set(0,'8*_hi there!');
labels.set(1,'*6');
oTable.setColumnLabels(labels)

try
    mTable = osimTableToStruct(oTable);
catch
    error('Failed to convert rare column labels to Structure fields in osimTableToStruct()')
end



