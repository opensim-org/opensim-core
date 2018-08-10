% ----------------------------------------------------------------------- %
% The OpenSim API is a toolkit for musculoskeletal modeling and           %
% simulation. See http://opensim.stanford.edu and the NOTICE file         %
% for more information. OpenSim is developed at Stanford University       %
% and supported by the US National Institutes of Health (U54 GM072970,    %
% R24 HD065690) and by DARPA through the Warrior Web program.             %
%                                                                         %
% Copyright (c) 2005-2018 Stanford University and the Authors             %
% Author(s): James Dunne                                                  %
%                                                                         %
% Licensed under the Apache License, Version 2.0 (the "License");         %
% you may not use this file except in compliance with the License.        %
% You may obtain a copy of the License at                                 %
% http://www.apache.org/licenses/LICENSE-2.0.                             %
%                                                                         %
% Unless required by applicable law or agreed to in writing, software     %
% distributed under the License is distributed on an "AS IS" BASIS,       %
% WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or         %
% implied. See the License for the specific language governing            %
% permissions and limitations under the License.                          %
% ----------------------------------------------------------------------- %

classdef osimC3D < matlab.mixin.SetGet
% osimC3D(filepath, ForceLocation)
%   C3D data to OpenSim Tables.
%   OpenSim Utility Class
%   Inputs:
%   filepath           Full path to the location of the C3D file
%   ForceLocation      Integer value for representation of force from plate
%                      0 = forceplate orgin, 1 = COP, 2 = Point Of Wrench
%                      Application
    properties (Access = private)
        path
        name
        markers
        forces
        ForceLocation
    end
    methods
        function obj = osimC3D(path2c3d, ForceLocation)
            % Class Constructor: input is an absolute path to a C3D file.

            % Verify the correct number of inputs
            if nargin ~= 2
                error('Number of inputs is incorrect. Class requires filepath (string) and ForceLocation (integer)')
            end
            % verify the file path is correct
            if exist(path2c3d, 'file') == 0
                error('File does not exist. Check path is correct')
            else
                [path, name, ext] = fileparts(path2c3d);
                if isempty(path)
                    error('Input path must be full path to file (C:/data/Walking.mot)')
                end
                obj.path = path;
                obj.name = name;
            end
            % Verify the ForceLocation input is correct
            if  ~isnumeric(ForceLocation) || ~ismember(ForceLocation,[0:2])
                error('ForceLocation must be an integer of value 0, 1, or 2')
            end
            % load java libs
            import org.opensim.modeling.*
            % Use a c3dAdapter to read the c3d file
            tables = C3DFileAdapter().read(path2c3d, ForceLocation);
            % Set the marker and force data into OpenSim tables
            obj.markers = tables.get('markers');
            obj.forces = tables.get('forces');
            % Set the force location specifier in case someone wants to
            % check.
            switch(ForceLocation)
                case 0
                    location = 'OriginOfForcePlate';
                case 1
                    location = 'CenterOfPressure';
                case 2
                    location = 'PointOfWrenchApplication';
            end
            obj.ForceLocation = location;
        end
        function p = getPath(obj)
            p = obj.path();
        end
        function n = getName(obj)
            n = obj.name();
        end
        function location = getForceLocation(obj)
            % Get the Force Location
            location = obj.ForceLocation();
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
            n = obj.markers.getNumColumns();
        end
        function n = getNumForces(obj)
            % Get the number of forceplates in the c3d file
            n = (obj.forces.getNumColumns())/3;
        end
        function t = getStartTime(obj)
            % Get the start time of the c3d file
            t = obj.markers.getIndependentColumn().get(0);
        end
        function t = getEndTime(obj)
            % Get the end time of the c3d file
            t = obj.markers.getIndependentColumn().get(obj.markers.getNumRows() - 1);
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
            table = obj.markers().clone();
        end
        function table = getTable_forces(obj)
            table = obj.forces().clone();
        end
        function [markerStruct, forcesStruct] = getAsStructs(obj)
            % Convert the OpenSim tables into Matlab Structures
            markerStruct = osimTableToStruct(obj.markers);
            forcesStruct = osimTableToStruct(obj.forces);
            disp('Maker and force data returned as Matlab Structs')
        end
        function rotateData(obj,axis,value)
            % Method for rotating marker and force data stored in osim
            % tables.
            % c3d.rotateData('x', 90)
            
            if ~ischar(axis)
               error('Axis must be either x,y or z')
            end
            if ~isnumeric(value)
                error('value must be numeric (90, -90, 270, ...')
            end
            % rotate the tables
            obj.rotateTable(obj.markers, axis, value);
            obj.rotateTable(obj.forces, axis, value);
            disp('Marker and Force tables have been rotated')
        end
        function writeTRC(obj,varargin)
            % Write marker data to trc file.
            % osimC3d.writeTRC()                       Write to dir of input c3d.
            % osimC3d.writeTRC('Walking.trc')          Write to dir of input c3d with defined file name.
            % osimC3d.writeTRC('C:/data/Walking.trc')  Write to defined path input path.

            % Compute an output path to use for writing to file
            outputPath = generateOutputPath(obj,varargin,'.trc');


            import org.opensim.modeling.*
            % Write to file
            TRCFileAdapter().write( obj.markers, outputPath)
            disp(['Marker file written to ' outputPath]);
        end
        function writeMOT(obj,varargin)
        % Write force data to mot file.
        % osimC3d.writeMOT()                       Write to dir of input c3d.
        % osimC3d.writeMOT('Walking.mot')          Write to dir of input c3d with defined file name.
        % osimC3d.writeMOT('C:/data/Walking.mot')  Write to defined path input path.
        % 
        % This function assumes point and torque data are in mm and Nmm and
        % converts them to m and Nm. If your C3D is already in M and Nm,
        % comment out the internal function convertMillimeters2Meters()

         % Compute an output path to use for writing to file
         outputPath = generateOutputPath(obj,varargin,'.mot');
         
         import org.opensim.modeling.*
         % Get the forces table
         forces = obj.getTable_forces();
         % Get the column labels
         labels = forces.getColumnLabels();
         % Make a copy
         updlabels = labels; 
          
         % Labels from C3DFileAdapter are f1, p1, m1, f2,...
         % We edit them to be consistent with requirements of viewing 
         % forces in the GUI (ground_force_vx, ground_force_px,...)
         for i = 0 : labels.size() - 1
            % Get the label as a string
            label = char(labels.get(i));
            % Transform the label depending on force, point, or moment
            if ~isempty(strfind(label,'f'))
                label = strrep(label,'f', 'ground_force_');
                label = [label '_v'];
            elseif ~isempty(strfind(label,'p'))
                label = strrep(label,'p', 'ground_force_');
                label = [label '_p'];
            elseif ~isempty(strfind(label,'m'))
                label = strrep(label,'m', 'ground_moment_');
                label = [label '_m'];
            end
            % update the label name 
            updlabels.set(i,label);
         end
         
         % set the column labels
         forces.setColumnLabels(updlabels)
         
         % Flatten the Vec3 force table
         postfix = StdVectorString();
         postfix.add('x');postfix.add('y');postfix.add('z');
         forces_flat = forces.flatten(postfix);
          
         % Change the header in the file to meet Storage conditions
          if forces_flat.getTableMetaDataKeys().size() > 0
              for i = 0 : forces_flat.getTableMetaDataKeys().size() - 1
                  % Get the metakey string at index zero. Since the array gets smaller on
                  % each loop, we just need to keep taking the first one in the array.
                  metakey = char(forces_flat.getTableMetaDataKeys().get(0));
                  % Remove the key from the meta data
                  forces_flat.removeTableMetaDataKey(metakey)
              end
          end
          % Add the column and row data to the meta key
          forces_flat.addTableMetaDataString('nColumns',num2str(forces_flat.getNumColumns()+1))
          forces_flat.addTableMetaDataString('nRows',num2str(forces_flat.getNumRows()));

          % Convert mm to m
          forces_flat_m  = obj.convertMillimeters2Meters(forces_flat);
          
          % Write to file
          STOFileAdapter().write(forces_flat_m, outputPath)
          disp(['Forces file written to ' outputPath]);
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
        function rotateTable(obj, table, axisString, value)
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
        function outputPath = generateOutputPath(obj,path, ext)
            % Function to generate an output path from no, partial, or fully
            % defined user path. 
            
            % Validate the output filename
            if size(path,2) > 1
                % Path object should be of size == 1, any larger and user
                % input multiple variables into function. 
                error([ num2str(size(path,2)) ' inputs, expecting zero or one'])
            end
        
            if isempty(path)
               % No file path has been input, so use the path and name from
               % the c3d file. 
               filepath = obj.getPath();
               name = obj.getName();
            else
            
                if ~ischar(path{1})
                   error('Input must be a sting of characters')
                end

                if isempty(strfind(path{1}, ext))
                   error(['Input must be a path to a ' ext ' file']);
                end
            
                % User has included a path to write to
                [filepath, name, e] = fileparts(path{1}); 
                if isempty(filepath)
                  % Only the file name is given
                  filepath = obj.getPath();
                end
            end
            % Generate the output path.
            outputPath = fullfile(filepath, [name ext]);
        end
        function table_flat = convertMillimeters2Meters(obj,table_flat)
            % Function to convert displacement forceplate measurements made
            % in millimeters to meters. This will convert point data (mm)
            % to m and Torque data (Nmm) to Nm.
            
            nForces = table_flat.getNumColumns();
            nRows  = table_flat.getNumRows();
            labels = table_flat.getColumnLabels();
            
            for i = 0 : nForces - 1
                % Find all point and torque colomns. Force columns will
                % have _v in the label, all columns that don't have this
                % character will be point and torque columns
                if ~contains(char(labels.get(i)),'v')
                    for u = 0 : nRows - 1
                        % Get the table value
                        c = table_flat.getDependentColumnAtIndex(i).get(u);
                        % set the table value
                        table_flat.getDependentColumnAtIndex(i).set(u,c/1000);
                    end
                end    
            end
           disp('Point and Torque values convert from mm and Nmm to m and Nm, respectively')
        end
   end
end
