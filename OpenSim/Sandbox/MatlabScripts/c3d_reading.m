
import org.opensim.modeling.*



%% Use a c3dAdapter to turn read a c3d file
adapter = C3DFileAdapter()

tables = adapter.read('test_walking.c3d')


%% get the Markers
markers = tables.get('markers');
markerdata = opensimVec3TableToStruct(markers);
markerlabels = fieldnames(markerdata);

% Print the (unrotated) markers to trc file
trcfileadapter = TRCFileAdapter();
trcfileadapter.write(markers,'test_walking.trc');

%% Get the force
forces = tables.get('forces');
forcedata = opensimVec3TableToStruct(forces);
markerlabels = fieldnames(forcedata);

% Print the (unrotated) markers to trc file
trcfileadapter = MOTFileAdapter();
trcfileadapter.write(markers,'test_walking.trc');


%% Define a rotation matix
Rot = 90;          
rotationMatrix = [1,0,0;0,cos(Rot*pi/180),-(sin(Rot*pi/180));0,sin(Rot*pi/180),cos(Rot*pi/180)];
        
%% 
for iMarker = 0 : length(markerdata) - 1

    % get the column data for the marker
    marker = markers.updDependentColumn(markerlabels(iMarker+1) );

    % go through each element of the table column, rotate the Vec3, and write
    % back to the column.
    for iRow = 0 : markers.getNumRows - 1
        % get Matlab vector marker position
        vectorData = [marker.getElt(0,iRow).get(0)...
                      marker.getElt(0,iRow).get(1)...
                      marker.getElt(0,iRow).get(2)];

        % rotate the marker data
        rotatedData = [rotationMatrix'*vectorData']';

        % Write the rotated data back to the Vec3TimesSeriesTable
        % THE BELOW LINES DO NOT WORK AND THERE IS CURRENTLY NO FIX
        % marker.updElt(0,iRow).set(0, rotatedData)
             
    end
end

%% 
for iForces = 0 : length(forcesdata) - 1

    % get the column data for the marker
    force = markers.updDependentColumn(forcelabels(iForce+1) );

    % go through each element of the table column, rotate the Vec3, and write
    % back to the column.
    for iRow = 0 : forces.getNumRows - 1
        % get Matlab vector marker position
        vectorData = [force.getElt(0,iRow).get(0)...
                      force.getElt(0,iRow).get(1)...
                      force.getElt(0,iRow).get(2)];

        % rotate the marker data
        rotatedData = [rotationMatrix'*vectorData']';

        % Write the rotated data back to the Vec3TimesSeriesTable
        % THE BELOW LINES DO NOT WORK AND THERE IS CURRENTLY NO FIX
        % force.updElt(0,iRow).set(0, rotatedData)
             
    end
end




%% Print the rotated markers to trc file
trcfileadapter = TRCFileAdapter();
trcfileadapter.write(markers,'test_walking_rotated.trc');

