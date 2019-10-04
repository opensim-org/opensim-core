classdef osimC3D < matlab.mixin.SetGet
% osimC3D(filepath, ForceLocation)
%   C3D data to OpenSim Tables.
%   OpenSim Utility Class
%   Inputs:
%   filepath           Full path to the location of the C3D file
%   ForceLocation      Integer value for representation of force from plate
%                      0 = forceplate orgin, 1 = COP, 2 = Point Of Wrench
%                      Application

% ----------------------------------------------------------------------- %
% The OpenSim API is a toolkit for musculoskeletal modeling and           %
% simulation. See http://opensim.stanford.edu and the NOTICE file         %
% for more information. OpenSim is developed at Stanford University       %
% and supported by the US National Institutes of Health (U54 GM072970,    %
% R24 HD065690) and by DARPA through the Warrior Web program.             %
%                                                                         %
% Copyright (c) 2005-2019 Stanford University and the Authors             %
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

    properties (Access = private)
        path
        name
        markers
        forces
        ForceLocation
    end
    methods
        function self = osimC3D(path2c3d, ForceLocation)
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
                self.path = path;
                self.name = name;
            end
            % Verify the ForceLocation input is correct
            if  ~isnumeric(ForceLocation) || ~ismember(ForceLocation,[0:2])
                error('ForceLocation must be an integer of value 0, 1, or 2')
            end
            % load java libs
            import org.opensim.modeling.*
            % Use a c3dAdapter to read the c3d file
			c3dAdapter =  C3DFileAdapter();
			c3dAdapter.setLocationForForceExpression(ForceLocation);
            tables = c3dAdapter.read(path2c3d);
            % Set the marker and force data into OpenSim tables
            self.markers = c3dAdapter.getMarkersTable(tables);
            self.forces = c3dAdapter.getForcesTable(tables);
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
            self.ForceLocation = location;
        end
        function p = getPath(self)
            p = self.path();
        end
        function n = getName(self)
            n = self.name();
        end
        function location = getForceLocation(self)
            % Get the Force Location
            location = self.ForceLocation();
        end
        function rate = getRate_marker(self)
            % Get the capture rate used for the Marker Data
            rate = str2double(self.markers.getTableMetaDataAsString('DataRate'));
        end
        function rate = getRate_force(self)
            % Get the capture rate used for the Force Data
            rate = str2double(self.forces.getTableMetaDataAsString('DataRate'));
        end
        function n = getNumTrajectories(self)
            % Get the number of markers in the c3d file
            n = self.markers.getNumColumns();
        end
        function n = getNumForces(self)
            % Get the number of forceplates in the c3d file
            n = (self.forces.getNumColumns())/3;
        end
        function t = getStartTime(self)
            % Get the start time of the c3d file
            t = self.markers.getIndependentColumn().get(0);
        end
        function t = getEndTime(self)
            % Get the end time of the c3d file
            t = self.markers.getIndependentColumn().get(self.markers.getNumRows() - 1);
        end
        function name = getFileName(self)
            % Get the name of the c3d file
            [filedirectory, name, extension] = fileparts(self.path);
        end
        function filedirectory = getDirectory(self)
            % Get the directory path for the c3d file
            [filedirectory, name, extension] = fileparts(self.path);
        end
        function table = getTable_markers(self)
            table = self.markers().clone();
        end
        function table = getTable_forces(self)
            table = self.forces().clone();
        end
        function [markerStruct, forcesStruct] = getAsStructs(self)
            % Convert the OpenSim tables into Matlab Structures
            markerStruct = osimTableToStruct(self.markers);
            forcesStruct = osimTableToStruct(self.forces);
            disp('Maker and force data returned as Matlab Structs')
        end
        function rotateData(self,axis,value)
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
            self.rotateTable(self.markers, axis, value);
            self.rotateTable(self.forces, axis, value);
            disp('Marker and Force tables have been rotated')
        end
        function convertMillimeters2Meters(self)
            % Function to convert  point data (mm) to m and Torque data
            % (Nmm) to Nm.

            import org.opensim.modeling.*

            nRows  = self.forces.getNumRows();
            labels = self.forces.getColumnLabels();

            for i = 0 : self.forces.getNumColumns - 1
                % All force columns will have the 'f' prefix while point
                % and moment columns will have 'p' and 'm' prefixes,
                % respectively.
                if ~startsWith(char(labels.get(i)),'f')
                   columnData = self.forces.updDependentColumnAtIndex(i);
                   for u = 0 : nRows - 1
                     % Divide by 1000
                     columnData.set(u,columnData.get(u).scalarDivideEq(1000));
                   end
                end
            end
           disp('Point and Torque values convert from mm and Nmm to m and Nm, respectively')
        end
        function writeTRC(self,varargin)
            % Write marker data to trc file.
            % osimC3d.writeTRC()                       Write to dir of input c3d.
            % osimC3d.writeTRC('Walking.trc')          Write to dir of input c3d with defined file name.
            % osimC3d.writeTRC('C:/data/Walking.trc')  Write to defined path input path.

            % Compute an output path to use for writing to file
            outputPath = generateOutputPath(self,varargin,'.trc');


            import org.opensim.modeling.*
            % Write to file
            TRCFileAdapter().write( self.markers, outputPath)
            disp(['Marker file written to ' outputPath]);
        end
        function writeMOT(self,varargin)
        % Write force data to mot file.
        % osimC3d.writeMOT()                       Write to dir of input c3d.
        % osimC3d.writeMOT('Walking.mot')          Write to dir of input c3d with defined file name.
        % osimC3d.writeMOT('C:/data/Walking.mot')  Write to defined path input path.

         % Compute an output path to use for writing to file
         outputPath = generateOutputPath(self,varargin,'.mot');

         import org.opensim.modeling.*
         % Get the forces table
         forces = self.getTable_forces();
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

          % Write to file
          STOFileAdapter().write(forces_flat, outputPath)
          disp(['Forces file written to ' outputPath]);
      end
   end

   methods (Access = private, Hidden = true)
        function setMarkers(self, a)
            % Private Method for setting the internal Marker table
            self.markers = a;
        end
        function setForces(self, a)
            % Private Method for setting the internal Force table
            self.forces = a;
        end
        function rotateTable(self, table, axisString, value)
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

            % instantiate a transform selfect. Rotation() is a Simbody class
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
        function outputPath = generateOutputPath(self,path, ext)
            % Function to generate an output path from no, partial, or fully
            % defined user path.

            % Validate the output filename
            if size(path,2) > 1
                % Path selfect should be of size == 1, any larger and user
                % input multiple variables into function.
                error([ num2str(size(path,2)) ' inputs, expecting zero or one'])
            end

            if isempty(path)
               % No file path has been input, so use the path and name from
               % the c3d file.
               filepath = self.getPath();
               name = self.getName();
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
                  filepath = self.getPath();
                end
            end
            % Generate the output path.
            outputPath = fullfile(filepath, [name ext]);
        end
   end
end
