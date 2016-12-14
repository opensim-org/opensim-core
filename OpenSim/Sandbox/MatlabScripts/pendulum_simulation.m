%  * -------------------------------------------------------------------------- *
%  * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
%  * See http://opensim.stanford.edu and the NOTICE file for more information.  *
%  * OpenSim is developed at Stanford University and supported by the US        *
%  * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
%  * through the Warrior Web program.                                           *
%  *                                                                            *
%  * Copyright (c) 2005-2016 Stanford University and the Authors                *
%  * Author(s): James Dunne, Thomas Uchida, Chris Dembia                        *
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

%% Read a model into memory, run a fwd simulation, and print results to file
import org.opensim.modeling.*

% read double pendulum model into memory
modelFileName = 'double_pendulum_markers.osim';
model = Model(modelFileName);

%% set the time interval that reporters will use
timeInterval = 0.1;

%% add a console reporter
consoleReporter = ConsoleReporter();
consoleReporter.set_report_time_interval(timeInterval);

% When connecting the outputs, set the alias names 'pin1_anlge' and 'q2'.
% These will appear as the column labels
consoleReporter.addToReport(model.getCoordinateSet().get(0).getOutput('value'),'q1');
consoleReporter.addToReport(model.getCoordinateSet().get(1).getOutput('value'),'q2');
model.addComponent(consoleReporter);


%% add a table reporter for the pendulum coordinates
coordinateReporter= TableReporter();
coordinateReporter.set_report_time_interval(timeInterval);

% When connecting the outputs, set the alias names 'q1' and 'q2'.
% These will be the column labels when printed to file
coordinateReporter.addToReport(model.getCoordinateSet().get(0).getOutput('value'),'q1');
coordinateReporter.addToReport(model.getCoordinateSet().get(1).getOutput('value'),'q2');
model.addComponent(coordinateReporter);

%% add a Vec3 table reporter for the pendulum markers
markerReporter= TableReporterVec3();
markerReporter.set_report_time_interval(timeInterval);

% When connecting the outputs, set the alias names 'marker_1' and 'marker_2'.
% These will be the colomn labels when printed to file
markerReporter.addToReport(model.getMarkerSet().get(0).getOutput('location'),'marker_1')
markerReporter.addToReport(model.getMarkerSet().get(1).getOutput('location'),'marker_1')
model.addComponent(markerReporter);

% run a fwd simulation using the manager
state = model.initSystem();
manager = Manager(model);
manager.setInitialTime(0);
manager.setFinalTime(2);
manager.integrate(state);

%% define the adapters
stoAdapter = STOFileAdapter();
stoAdapterVec3 = STOFileAdapterVec3();
trcAdapter = TRCFileAdapter();

% write coordinate data to .sto file
table = coordinateReporter.getTable();
stoAdapter.write(table, 'pendulum_coordinates.sto');

% write marker data to .sto file
tableVec3 = markerReporter.getTable();
stoAdapterVec3.write(tableVec3 , 'marker_locations.sto')

% write marker data to .trc file
% trc adapter requires DataRate and Units be included in the metadata. 
tableVec3.addTableMetaDataString('DataRate','10');
tableVec3.addTableMetaDataString('Units','m');
trcAdapter.write(tableVec3 , 'marker_locations.trc')
