classdef opensimC3D < matlab.mixin.SetGet
% opensimC3D C3D data into OpenSim Tables
%   Utility  Class that uses OpenSim's C3DFileAdapter() class to produce
%   OpenSim tables. Methods include Simbody rotation.
    properties (Access = private)
    path
    markers
    forces
    end
    methods
        function obj = opensimC3D(path2c3d)
            % Class Constructor: input is an absolute path to a C3D file. 
            
            % verify the file path is correct
            if exist(path2c3d, 'file') == 0
                error('File does not exist. Check path is correct')
            else
                obj.path = path2c3d;
            end
            % load java libs
            import org.opensim.modeling.*
            % Use a c3dAdapter to turn read a c3d file
            tables = C3DFileAdapter().read(path2c3d);
            % get the marker and force data into OpenSim tables
            obj.markers = tables.get('markers');
            obj.forces = tables.get('forces');
        end            
        function rate = getRate_marker(obj)
            % Get the capture rate used for the Marker Data
            rate = str2double(obj.markers.getTableMetaDataAsString('DataRate'));
        end
        function rate = getRate_force(obj)
            % Get the capture rate used for the Force Data
            rate = str2double(obj.forces.getTableMetaDataAsString('DataRate'));
        end
        function n = getNumTrajectories(obj)
            % Get the number of markers in the c3d file
            n = obj.markers.getNumColumns;
        end
        function n = getNumForces(obj)
            % Get the number of forceplates in the c3d file
            n = (obj.forces.getNumColumns)/3;
        end
        function t = getStartTime(obj)
            % Get the start time of the c3d file
            t = obj.markers.getIndependentColumn().get(0);
        end
        function t = getEndTime(obj)
            % Get the end time of the c3d file
            t = obj.markers.getIndependentColumn().get(obj.markers.getNumRows - 1);
        end
        function name = getFileName(obj)
            % Get the name of the c3d file
            [filedirectory, name, extension] = fileparts(obj.path);
        end
        function filedirectory = getDirectory(obj)
            % Get the directory path for the c3d file
            [filedirectory, name, extension] = fileparts(obj.path);
        end
        function table = getTable_markers(obj)
            table = obj.markers();
        end
        function table = getTable_forces(obj)
            table = obj.forces();
        end
        function [markerStruct, forcesStruct] = getAsStruct(obj)
            % Convert the OpenSim tables into Matlab Structures
            markerStruct = osimTableToStruct(obj.markers);
            forcesStruct = osimTableToStruct(obj.forces);
            disp('Maker and force data returned as Matlab Structs')
        end
        function rotateData(obj,axis,value)
            % Method for rotating marker and force data stored in OpenSim 
            % tables
            if ~ischar(axis) 
               error('Axis must be either x,y or z')
            end
            if ~isnumeric(value)
                error('value must be numeric (90, -90, 270, ...')
            end
            
            % rotate the tables
            markers_rot = obj.rotateTable(obj.markers, axis, value);
            forces_rot = obj.rotateTable(obj.forces, axis, value);
        
            % set the internal tables to the new rotated ones
            obj.setMarkers(markers_rot)
            obj.setForces(forces_rot)
        
            disp('Marker and Force tables have been rotated')
        end
   end
   
   methods (Access = private, Hidden = true)
        function setMarkers(obj, a)
            % Private Method for setting the internal Marker table
            obj.markers = a;
        end
        function setForces(obj, a)
            % Private Method for setting the internal Force table
            obj.forces = a;
        end 
        function table = rotateTable(obj, table, axisString, value) 
            % Private Method for doing the table rotation operations
            
            import org.opensim.modeling.*
            % set up the transform
            if strcmp(axisString, 'x')
                axis = CoordinateAxis(0);
            elseif strcmp(axisString, 'y')
                axis = CoordinateAxis(1);
            elseif strcmp(axisString, 'z')
                axis = CoordinateAxis(2);
            else
                error('input axis must ne either x,y, or z')
            end

            % instantiate a transform object. Rotation() is a Simbody class
            R = Rotation( deg2rad(value) , axis ) ;

            % Rotation() works on each row. 
            for iRow = 0 : table.getNumRows() - 1
                % get a row from the table
                rowVec = table.getRowAtIndex(iRow);
                % rotate each Vec3 element of row vector, rowVec, at once
                rowVec_rotated = R.multiply(rowVec);
                % overwrite row with rotated row
                table.setRowAtIndex(iRow,rowVec_rotated)
            end
        end
   end
end

   
    
    
    