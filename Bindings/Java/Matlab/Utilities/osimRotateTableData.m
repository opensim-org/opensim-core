function T_R = osimRotateTableData(T, A, V)
% t = rotateTableData(table, A, V) returns an opentsim table with
% elements rotated about A by V. table is an opensim
% TimeseriesTableVec3, A is a string ('x','y','z'), and V is a 
% number.
%  
% Example;
% t_r = rotateTableData(t, 'x', 90) - rotate all (Vec3) elements in t by 90
%                                     degrees about the x axis.
 
% By James Dunne

%% import java libraries
import org.opensim.modeling.*

%% set up the transform
if strcmp(A, 'x')
    coordinate = CoordinateAxis(0);
elseif strcmp(A, 'y')
    coordinate = CoordinateAxis(1);
elseif strcmp(A, 'z')
    coordinate = CoordinateAxis(2);
else
    error(['Axis must be either x,y or z'])
end

%% instantiate a transform object
r = Rotation( deg2rad(V) , coordinate ) ;

%% rotate the elements in each row
% clone the table.
T_R = T.clone();

for iRow = 0 : T_R.getNumRows() - 1
    % get a row from the table
    R = T_R.getRowAtIndex(iRow);
    % pass the row to Rotation. 
    R_R = r.multiply(R);
    % overwrite row with rotated row
    T_R.setRowAtIndex(iRow,R_R)
end

end







