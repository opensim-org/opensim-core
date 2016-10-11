function table = matlabArrayToOpenSimTimeSeriesTableVec3(table_array)
%% Matlab Utility to take a Matlab Struct and convert into an OpenSim Table


%%
import org.opensim.modeling.*
% table_array = marker_array

%% TODO: Type check the input data for Vec3 compatabilty 


%% make a empty table
table = DataTableVec3;
    
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
elem = Vec3(0, 0, 0);
elems = StdVectorVec3();

for iCol = 1 : table.getColumnLabels.size
      elems.add(elem); 
end

for iRow  = 1 : length(table_array.(labels{1}))
    % Append row to the table.
    row = RowVectorOfVec3(elems);
    table.appendRow(0, row);
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

        t = num2cell([ table_array.(labels{i+1})(iRow+1,1)...
                   table_array.(labels{i+1})(iRow+1,2)...
                   table_array.(labels{i+1})(iRow+1,3) ]);
        [x,y,z] = deal(t{:});
    
        
        column.set( iRow , Vec3(x,y,z) );    
   end 
  
end

%% TODO: Decide to output a timerseries table or not

try
   timeseriestable = TimeSeriesTableVec3(table);
catch exception
   'Times series table exception: Time has to be increasing'
end

