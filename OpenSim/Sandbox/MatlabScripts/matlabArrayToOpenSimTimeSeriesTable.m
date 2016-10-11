function table = matlabArrayToOpenSimTimeSeriesTableVec3(table_array)
%% Matlab Utility to take a Matlab Struct and convert into an OpenSim Table


%%
import org.opensim.modeling.*
% table_array = force_array

%% make a empty table
table = DataTable;

%% get the label names from the structure
labels = fieldnames(table_array);    

%% Set the number of columns and the column labels
colnames = StdVectorString;
for i = 1 : length(labels)
   
    if ~strcmp(labels{i},'time')
        colnames.add(labels{i})
    end
end

table.setColumnLabels(colnames);

%% Create an empty Array
row = RowVector(colnames.size, 0);

for ii  = 1 : length(table_array.(labels{1}))

    table.appendRow(0, row)
    
end
    
%% write values to the OpenSim table
for i  = 0 : length(labels) -1

    if strcmp(labels(i+1),'time')

        inCol = table.getIndependentColumn;
      
        for iRow = 0 : table.getNumRows - 1
        
            inCol.set(iRow, table_array.(labels{i+1})(iRow+1))
            
        end
        
        continue
    end
    
    column = table.getDependentColumnAtIndex(i);
    
   for iRow = 0 : table.getNumRows - 1

        column.set( iRow , table_array.(labels{i+1})(iRow+1)  );    
   end 
  
end 

%% TODO: Decide to output a timerseries table or not

try
   timeseriestable = TimeSeriesTable(table);
catch exception
   'Times series table exception: Time has to be increasing'
end
