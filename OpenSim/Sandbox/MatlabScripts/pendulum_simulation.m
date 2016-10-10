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
%  * Unless required by applicable law or agreed to in writic3d_readinng, software        *
%  * distributed under the License is distributed on an "AS IS" BASIS,          *
%  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
%  * See the License for the specific language governing permissions and        *
%  * limitations under the License.                                             *
%  * -------------------------------------------------------------------------- */

%% Read a model into memory, run a fwd simulation, 
clear all; close all; 

import org.opensim.modeling.*

%   public static void test_TableReporter_2() throws java.io.IOException {
timeInterval = 0.1;

% add a model
modelFileName = 'double_pendulum_markers.osim';
model = Model(modelFileName);

consoleReporter = ConsoleReporter();
consoleReporter.set_report_time_interval(timeInterval);
consoleReporter.updInput('inputs').connect(model.getCoordinateSet().get(0).getOutput('value'),'pin1_angle');
consoleReporter.updInput('inputs').connect(model.getCoordinateSet().get(1).getOutput('value'),'q2');
model.addComponent(consoleReporter);

tableReporter = TableReporter();
tableReporter.set_report_time_interval(timeInterval);
tableReporter.updInput('inputs').connect(model.getCoordinateSet().get(0).getOutput('value'),'q1');
tableReporter.updInput('inputs').connect(model.getCoordinateSet().get(1).getOutput('value'),'q2');
model.addComponent(tableReporter);

stoAdapter = STOFileAdapter();

assert(~tableReporter.getTable().hasColumnLabels());

%%
for n = 1 : 150
    assert(tableReporter.getTable().getNumRows()    == 0);
    assert(tableReporter.getTable().getNumColumns() == 0);

    state = model.initSystem();
    manager = Manager(model);
    manager.setInitialTime(0);
    manager.setFinalTime(2);
    manager.integrate(state);

    table = tableReporter.getTable();
    stoAdapter.write(table, 'pendulum_coordinates.sto');

    assert(table.getColumnLabels().size() == 2) ;
    assert(table.getColumnLabel(0).equals('q1'));
    assert(table.getColumnLabel(1).equals('q2'));
    assert(table.getNumRows()    == 1 + (2 / timeInterval));
    assert(table.getNumColumns() == 2);

    tableReporter.clearTable();

    assert(table.getColumnLabels().size() == 2);
    assert(table.getColumnLabel(0).equals('q1'));
    assert(table.getColumnLabel(1).equals('q2'));
    assert(table.getNumRows()    == 0);
    assert(table.getNumColumns() == 0);
end


