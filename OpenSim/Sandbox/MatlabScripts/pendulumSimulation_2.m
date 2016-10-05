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


import org.opensim.modeling.*


time_interval = 0.1;

model = Model('double_pendulum_markers.osim');
%  Add a console reporter to print the muscle fiber force and elbow angle.
reporter = ConsoleReporter();
reporter.set_report_time_interval(0.1);
reporter.updInput('inputs').connect(model.getCoordinateSet.get(0).getOutput('value'), 'pin1_angle' );
reporter.updInput('inputs').connect(model.getCoordinateSet.get(1).getOutput('value'), 'q2' );
model.addComponent(reporter);

%% add a table reporter for the coordinates
coordinateReporter = TableReporter();
coordinateReporter.set_report_time_interval(time_interval)
coordinateReporter.updInput('inputs').connect(model.getCoordinateSet.get(0).getOutput('value'), 'q1' );
coordinateReporter.updInput('inputs').connect(model.getCoordinateSet.get(1).getOutput('value'), 'q2' );
model.addComponent(coordinateReporter);

stofileadapter = STOFileAdapter();

for n = 1 : 10
   
    disp(['This is loop ' num2str(n) ''])
       
    % Simulate.
    state = model.initSystem();
    manager = Manager(model);
    manager.setInitialTime(0); manager.setFinalTime(n);
    manager.integrate(state);

    % get the coordinate and marker tables
    coordinatetable = coordinateReporter.getTable; 
    % print coordinates to file
    stofileadapter.write(coordinatetable,'pendulum_coordinates.sto');
    
    % clear the table
    coordinateReporter.clearTable ; 
    clear coordinatetable
    
end


