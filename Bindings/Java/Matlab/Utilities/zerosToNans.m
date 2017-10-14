function osimTable = zerosToNans(osimTable)
%% replaceZerosWNaNs()
%   btk outputs marker frames that have no vales as zero's. In openSim, 
%   zero's are interpreted literally.
%   This replaces zero values of marker sdata with NaN's
import org.opensim.modeling.*
notification = 0;
for i = 0 : osimTable.getNumRows() - 1
    row = osimTable.getRowAtIndex(i)
    for u = 0 : osimTable.getNumColumns() - 1
        p = row.get(u);
        % If the Marker components all equal 0, then replace with NaN
        if p.get(0) == 0 & p.get(1) == 0 & p.get(2) == 0 
            p.setToNaN()
            notifcation = 1;
        end  
        % Set the Vec3 for the row. 
        row.set(u,p);
    end
end

% Send a notification to the console to warn the user that the data has
% beeb changed. 
if notification == 1
    warning('Some Marker components were [0,0,0]. They were changed to [NaN,Nan,Nan]')
end

