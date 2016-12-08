%  *                    OpenSim:  testREADME.cpp                                *
%  * -------------------------------------------------------------------------- *
%  * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
%  * See http://opensim.stanford.edu and the NOTICE file for more information.  *
%  * OpenSim is developed at Stanford University and supported by the US        *
%  * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
%  * through the Warrior Web program.                                           *
%  *                                                                            *
%  * Copyright (c) 2005-2016 Stanford University and the Authors                *
%  * Author(s): James Dunne, Thomas Uchida                                      *
%  * Contributor(s): Chris Dembia                                               *
%  *                                                                            *
%  * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
%  * not use this file except in compliance with the License. You may obtain a  *
%  * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
%  *                                                                            *
%  * Unless required by applicable law or agreed to in software                 *
%  * distributed under the License is distributed on an "AS IS" BASIS,          *
%  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
%  * See the License for the specific language governing permissions and        *
%  * limitations under the License.                                             *
%  * -------------------------------------------------------------------------- */

%% Read a model into memory, run a fwd simulation,
clear all; close all; clc;

import org.opensim.modeling.*

%   public static void test_TableReporter_2() throws java.io.IOException {
timeInterval = 0.1;

% add a model
modelFileName = 'double_pendulum_markers.osim';
model = Model(modelFileName);

%% add a console reporter
consoleReporter = ConsoleReporter();
consoleReporter.set_report_time_interval(timeInterval);

% When connecting the outputs, set the alias names 'pin1_anlge' and 'q2'.
% These will appear as the colomn labels
consoleReporter.updInput('inputs').connect(model.getCoordinateSet().get(0).getOutput('value'),'pin1_angle');
consoleReporter.updInput('inputs').connect(model.getCoordinateSet().get(1).getOutput('value'),'q2');
model.addComponent(consoleReporter);

%% add a table reporter for the pendulum coordiantes
coordinateReporter= TableReporter();
coordinateReporter.set_report_time_interval(timeInterval);

% When connecting the outputs, set the alias names 'q1' and 'q2'.
% These will be the colomn labels when printed to file
coordinateReporter.updInput('inputs').connect(model.getCoordinateSet().get(0).getOutput('value'),'q1');
coordinateReporter.updInput('inputs').connect(model.getCoordinateSet().get(1).getOutput('value'),'q2');
model.addComponent(coordinateReporter);

%% add a Vec3 table reporter for the pendulum markers
markerReporter= TableReporterVec3();
markerReporter.set_report_time_interval(timeInterval);

% When connecting the outputs, set the alias names 'marker_1' and 'marker_2'.
% These will be the colomn labels when printed to file
markerReporter.updInput('inputs').connect(model.getMarkerSet().get(0).getOutput('location'),'marker_1');
markerReporter.updInput('inputs').connect(model.getMarkerSet().get(1).getOutput('location'),'marker_2');
model.addComponent(markerReporter);

%% define the adapters
stoAdapter = STOFileAdapter();
stoAdapterVec3 = STOFileAdapterVec3();

% run a fwd simulation using the manager
state = model.initSystem();
manager = Manager(model);
manager.setInitialTime(0);
manager.setFinalTime(2);
manager.integrate(state);

% get the coordinate table from the reporter and write to file
table = coordinateReporter.getTable();
stoAdapter.write(table, 'pendulum_coordinates.sto');

% get the marker table from the reporter and write to file
tableVec3 = markerReporter.getTable();
stoAdapterVec3.write(tableVec3 , 'marker_locations.sto')

trcAdapter = TRCFileAdapter();
trcAdapter.write(tableVec3 , 'marker_locations.trc')
