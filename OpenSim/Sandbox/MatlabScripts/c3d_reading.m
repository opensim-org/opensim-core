
import org.opensim.modeling.*



%% Use a c3dAdapter to turn read a c3d file
adapter = C3DFileAdapter()

tables = adapter.read('test_walking.c3d')


%% get the Markers
markers = tables.get('markers');

markerdata = opensimTimeSeriesTableToMatlab(markers);
markerlabels = fieldnames(markerdata);

% Print the (unrotated) markers to trc file
trcfileadapter = TRCFileAdapter();
trcfileadapter.write(markers,'test_walking.trc');

%% Get the force
% forces = tables.get('forces');
% forcedata = opensimVec3TableToStruct(forces);
% markerlabels = fieldnames(forcedata);
% 
% % Print the (unrotated) markers to trc file
% trcfileadapter = TRCFileAdapter();
% trcfileadapter.write(markers,'test_walking.trc');


%% Define a rotation matix
Rot = 90;          
rotationMatrix = [1,0,0;0,cos(Rot*pi/180),-(sin(Rot*pi/180));0,sin(Rot*pi/180),cos(Rot*pi/180)];
        
%% Rotate marker data

new_table = TimeSeriesTableVec3()


TimeSeriesTableVec3::DependentsMetaData marker_dep_metadata{};
marker_dep_metadata.setValueArrayForKey("labels", marker_labels);
marker_table.setDependentsMetaData(marker_dep_metadata);

double time_step{1.0 / acquisition->GetPointFrequency()};
for f = 1:0 
    
    
    rowVector = RowVector()
    
    
    
    for(auto it = marker_pts->Begin();
        it != marker_pts->End();
        ++it) {
        auto pt = *it;
        row[m++] = SimTK::Vec3{pt->GetValues().coeff(f, 0),
                               pt->GetValues().coeff(f, 1),
                               pt->GetValues().coeff(f, 2)};
    }

    
marker_table.appendRow(0 + f * time_step, row);


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
        Vec3(rotatedData(1),rotatedData(2),rotatedData(3))
        
        methodsview(new_table)
                 
          create an empty timesseriestable and use append every row Vec3
    end
end

%% Print the rotated markers to trc file
trcfileadapter = TRCFileAdapter();
trcfileadapter.write(markers,'test_walking_rotated.trc');

%% Rotate Force data

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

%% Print the force data




