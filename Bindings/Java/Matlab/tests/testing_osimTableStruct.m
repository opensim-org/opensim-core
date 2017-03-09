clear all;close all;clc;
%%
import org.opensim.modeling.*

%% Use a c3dAdapter to turn read a c3d file
c3dAdapter =C3DFileAdapter();
c3d_data = c3dAdapter .read('test_walking.c3d');

%% get a table of markers 
% This is a vec3 table
markerTable_vec3 = c3d_data.get('markers');
% flatten to get a table of doubles
markerTable_doubles = markerTable_vec3.flatten();

%% convert to OpenSim objects
% Struct of Vec3's
markerStruct_vec3 = osimTableToStruct(markerTable_vec3);
% Struct of doubles. 
markerStruct_doubles = osimTableToStruct(markerTable_doubles);

%% Convert structs back to tables
markerTable_Frank_vec3 = osimTableFromStruct(markerStruct_vec3);
% Struct of doubles. 
markerTable_Frank_doubles = osimTableFromStruct(markerStruct_doubles);

%% Transformation from table to struct

% number of labels and rows for the initial opensim table
nLabels1 = markerTable_vec3.getNumColumns();
nRows1 = markerTable_vec3.getNumRows();

% number of labels and rows in the intermediate struct
labels = fieldnames(markerStruct_vec3);
nLabels2 = length(labels) - 1;
nRows2   = length(markerStruct_vec3.(labels{1}));

% number of labels and rows for the final opensim table
nLabels3 = markerTable_Frank_vec3.getNumColumns();
nRows3 =   markerTable_Frank_vec3.getNumRows();

if nLabels1 == nLabels2 && nLabels2 == nLabels3
    disp(['number of labels are consistent across each data type'])
else 
    error(['number of labels are inconsistent between transformation'])
end

if nRows1 == nRows2 && nRows2 == nRows3
    disp(['number of Rows are consistent across each data type'])
else 
    error(['number of labels are inconsistent between transformation'])
end

% check 500 random columns and rows and compare elements across all types    
randomLabelIndex = randi([1 nLabels1-1],1,500);
randomRowIndex = randi([1 nRows1-1],1,500);

for i = 1 : 500 
    
%   disp(char(markerTable_vec3.getColumnLabel(randomLabelIndex(i))));
    colData = markerTable_vec3.getDependentColumnAtIndex(randomLabelIndex(i));
    data1 = colData.get(randomRowIndex(i));
    
    labels{randomLabelIndex(i)+1};
    data2 = markerStruct_vec3.(labels{randomLabelIndex(i)+1})(randomRowIndex(i)+1,:);
   
%     disp([num2str(data1.get(0)) ' ' num2str(data1.get(1)) ' ' num2str(data1.get(2))  ]);
%     disp([num2str(data2(1)) ' ' num2str(data2(2)) ' ' num2str(data2(3))  ]);
%     disp(sprintf ( '\n') );

     if data1.get(0) ~= data2(1) || data1.get(1) ~= data2(2) || data1.get(2) ~= data2(3)
        error('Transform from table to struct resulted in incorrect values')
     end
     
    if i == 500
        disp('Transformation from table to struct was successfull - data in correct index')
    end
end

%% test that the resulting struct is the same as the original

% check 500 random columns and rows and compare elements across all types    
randomLabelIndex = randi([1 nLabels1-1],1,500);
randomRowIndex = randi([1 nRows1-1],1,500);

for i = 1 : 500 
    
%   disp(char(markerTable_vec3.getColumnLabel(randomLabelIndex(i))));
    colData = markerTable_vec3.getDependentColumnAtIndex(randomLabelIndex(i));
    data1 = colData.get(randomRowIndex(i));
    
    colData = markerTable_Frank_vec3.getDependentColumnAtIndex(randomLabelIndex(i));
    data2 = colData.get(randomRowIndex(i));
   
     if data1.get(0) ~= data2.get(0) || data1.get(1) ~= data2.get(1) || data1.get(2) ~= data2.get(2)
        error('New Table is different from original')
     end
     
    if i == 500
        disp('New Table is the same as the original')
    end
end
















