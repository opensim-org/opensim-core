%% pendulum_marker_positions.m
% OpenSim API example to build, simulate, and generate outputs for a
% double-pendulum model. Writes the results to .sto and .trc files.

% -------------------------------------------------------------------------- %
% The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  %
% See http://opensim.stanford.edu and the NOTICE file for more information.  %
% OpenSim is developed at Stanford University and supported by the US        %
% National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    %
% through the Warrior Web program.                                           %
%                                                                            %
% Copyright (c) 2005-2019 Stanford University and the Authors                %
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

% Author(s): James Dunne, Tom Uchida, Chris Dembia.                           %

%% Import Java libraries
import org.opensim.modeling.*

%% Instantiate model
model = Model();
model.setUseVisualizer(true);

%% Build model
% Bodies
r1  = Body('rod1', 1, Vec3(0), Inertia(0));
r2  = Body('rod2',  1, Vec3(0), Inertia(0));
model.addBody(r1); model.addBody(r2);

% Joints
p1 = PinJoint('pin1', model.getGround(), Vec3(0,2,0), Vec3(0), r1, Vec3(0, 1, 0), Vec3(0));
p2 = PinJoint('pin2', r1, Vec3(0), Vec3(0), r2, Vec3(0, 1, 0), Vec3(0));
model.addJoint(p1);model.addJoint(p2);

% Attach geometry to the bodies
g = Cylinder(0.05,0.5); g.setColor(Vec3(1));
r1b = PhysicalOffsetFrame(); r1b.setName('r1b');
r1b.setParentFrame(r1); r1b.setOffsetTransform(Transform(Vec3(0, 0.5, 0)));
r1.addComponent(r1b); r1b.attachGeometry(g.clone());
r2b = PhysicalOffsetFrame(); r2b.setName('r2b');
r2b.setParentFrame(r2); r2b.setOffsetTransform(Transform(Vec3(0, 0.5, 0)));
r2.addComponent(r2b); r2b.attachGeometry(g.clone());

% Markers
m = Marker(); m.setParentFrame(r1);m.set_location(Vec3(0.1,0.5,0));m.setName('marker_1');
m2 = Marker(); m2.setParentFrame(r1); m2.set_location(Vec3(0.25,0.2,0.2)); m2.setName('marker_2');
m3 = Marker(); m3.set_location(Vec3(-0.25,0.2,-0.2)); m3.setParentFrame(r1); m3.setName('marker_3');
m4 = Marker(); m4.set_location(Vec3(0.1,0.5,0)); m4.setParentFrame(r2); m4.setName('marker_4')
m5 = Marker(); m5.set_location(Vec3(0.25,0.2,0.2)); m5.setParentFrame(r2); m5.setName('marker_5')
m6 = Marker(); m6.set_location(Vec3(-0.25,0.2,-0.2)); m6.setParentFrame(r2); m6.setName('marker_6')

model.addMarker(m); model.addMarker(m2); model.addMarker(m3);
model.addMarker(m4); model.addMarker(m5); model.addMarker(m6);

%% Set the default values of the coordinates
model.getCoordinateSet().get(0).setDefaultValue(-1.04719755)
model.getCoordinateSet().get(1).setDefaultValue(-0.78539816)

% Set time interval for the reporter
reportTimeInterval = 0.1;

% Add console reporter
reporter = ConsoleReporter();
reporter.set_report_time_interval(reportTimeInterval);
reporter.addToReport(model.getCoordinateSet().get(0).getOutput('value'), 'pin1_coord_0');
reporter.addToReport(model.getCoordinateSet().get(1).getOutput('value'), 'pin2_coord_0');
model.addComponent(reporter);

% Add reporter for coordinates
cReporter = TableReporter();
cReporter.set_report_time_interval(reportTimeInterval);
cReporter.addToReport(model.getCoordinateSet().get(0).getOutput('value'), 'pin1_coord_0');
cReporter.addToReport(model.getCoordinateSet().get(1).getOutput('value'), 'pin2_coord_0');
model.addComponent(cReporter);

% Add reporter for markers
mReporter = TableReporterVec3();
mReporter.set_report_time_interval(reportTimeInterval);
mReporter.addToReport(model.getMarkerSet().get(0).getOutput('location'), 'marker_1');
mReporter.addToReport(model.getMarkerSet().get(1).getOutput('location'), 'marker_2');
mReporter.addToReport(model.getMarkerSet().get(2).getOutput('location'), 'marker_3');
mReporter.addToReport(model.getMarkerSet().get(3).getOutput('location'), 'marker_4');
mReporter.addToReport(model.getMarkerSet().get(4).getOutput('location'), 'marker_5');
mReporter.addToReport(model.getMarkerSet().get(5).getOutput('location'), 'marker_6');
model.addComponent(mReporter);

%% Run a forward simulation using the Manager.
state = model.initSystem();
manager = Manager(model);
manager.initialize( state );
manager.integrate(10);

%% Write joint angles to .sto file.
filename = 'pendulum_coordinates.sto';
STOFileAdapter.write(cReporter.getTable(), filename);
fprintf('Joint angles written to %s\n', filename);

%% Write marker locations to .sto file.
filename = 'pendulum_markerLocations.sto';
markerTable = mReporter.getTable();
STOFileAdapterVec3.write(markerTable, filename);
fprintf('Marker locations written to %s\n', filename);

%% Write marker locations to .trc file.
filename = 'pendulum_markerLocations.trc';
markerTable.addTableMetaDataString('DataRate', num2str(reportTimeInterval));
markerTable.addTableMetaDataString('Units', 'm');
TRCFileAdapter.write(markerTable, filename);

fprintf('Marker locations written to %s\n', filename);
fprintf('Success!\n');
