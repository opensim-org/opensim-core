% -------------------------------------------------------------------------- %
%                simulate_double_pendulum_and_write_tables.m                 %
% -------------------------------------------------------------------------- %
% The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  %
% See http://opensim.stanford.edu and the NOTICE file for more information.  %
% OpenSim is developed at Stanford University and supported by the US        %
% National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    %
% through the Warrior Web program.                                           %
%                                                                            %
% Copyright (c) 2005-2017 Stanford University and the Authors                %
% Author(s): James Dunne, Tom Uchida, Chris Dembia                           %
%                                                                            %
% Licensed under the Apache License, Version 2.0 (the "License"); you may    %
% not use this file except in compliance with the License. You may obtain a  %
% copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         %
%                                                                            %
% Unless required by applicable law or agreed to in writing, software        %
% distributed under the License is distributed on an "AS IS" BASIS,          %
% WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   %
% See the License for the specific language governing permissions and        %
% limitations under the License.                                             %
% -------------------------------------------------------------------------- %

% This API example runs a forward simulation of a double-pendulum model and
% writes the results to .sto and .trc files.

import org.opensim.modeling.*

% Read double-pendulum model.
model = Model('double_pendulum_markers.osim');
model.setUseVisualizer(false);

% Set the time interval (in seconds) between consecutive data points stored by
% the reporters.
reportTimeInterval = 0.1;

% Add a console reporter to print the joint angles. The output will be written
% to the log file (out.log) in the current directory.
consoleReporter = ConsoleReporter();
consoleReporter.set_report_time_interval(reportTimeInterval);
% When connecting the outputs, set the alias names to 'pin1_angle' and
% 'pin2_angle'. The aliases will appear as the column labels.
consoleReporter.addToReport( ...
    model.getCoordinateSet().get(0).getOutput('value'), 'pin1_angle');
consoleReporter.addToReport( ...
    model.getCoordinateSet().get(1).getOutput('value'), 'pin2_angle');
model.addComponent(consoleReporter);

% Add a table reporter to record the joint angles.
tableReporterForAngles = TableReporter();
tableReporterForAngles.set_report_time_interval(reportTimeInterval);
tableReporterForAngles.addToReport( ...
    model.getCoordinateSet().get(0).getOutput('value'), 'pin1_angle');
tableReporterForAngles.addToReport( ...
    model.getCoordinateSet().get(1).getOutput('value'), 'pin2_angle');
model.addComponent(tableReporterForAngles);

% Add a table reporter to record the marker locations, which are of type Vec3.
tableReporterForMarkers = TableReporterVec3();
tableReporterForMarkers.set_report_time_interval(reportTimeInterval);
tableReporterForMarkers.addToReport( ...
    model.getMarkerSet().get(0).getOutput('location'), 'marker_1');
tableReporterForMarkers.addToReport( ...
    model.getMarkerSet().get(1).getOutput('location'), 'marker_2');
model.addComponent(tableReporterForMarkers);

% Run a forward simulation using the Manager.
state = model.initSystem();
manager = Manager(model);
manager.setInitialTime(0);
manager.setFinalTime(2);
manager.integrate(state);

% Display results recorded by the console reporter.
if (exist('out.log', 'file') ~= 2), error('log file (out.log) not found'); end
type('out.log');

% Write joint angles to .sto file.
filename = 'pendulum_coordinates.sto';
STOFileAdapter.write(tableReporterForAngles.getTable(), filename);
fprintf('Joint angles written to %s\n', filename);

% Write marker locations to .sto file.
filename = 'marker_locations.sto';
markerTable = tableReporterForMarkers.getTable();
STOFileAdapterVec3.write(markerTable, filename);
fprintf('Marker locations written to %s\n', filename);

% Write marker locations to .trc file. The TRCFileAdapter requires DataRate and
% Units be included in the table metadata.
filename = 'marker_locations.trc';
markerTable.addTableMetaDataString('DataRate', num2str(reportTimeInterval));
markerTable.addTableMetaDataString('Units', 'm');
TRCFileAdapter.write(markerTable, filename);
fprintf('Marker locations written to %s\n', filename);

fprintf('Success!\n');
