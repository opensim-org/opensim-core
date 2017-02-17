%-----------------------------------------------------------------------%
% The OpenSim API is a toolkit for musculoskeletal modeling and         %
% simulation. See http://opensim.stanford.edu and the NOTICE file       %
% for more information. OpenSim is developed at Stanford University     %
% and supported by the US National Institutes of Health (U54 GM072970,  %
% R24 HD065690) and by DARPA through the Warrior Web program.           %
%                                                                       %
% Copyright (c) 2017 Stanford University and the Authors                %
% Author(s): Thomas Uchida, Chris Dembia, Carmichael Ong, Nick Bianco,  %
%            Shrinidhi K. Lakshmikanth, Ajay Seth, James Dunne          %
%                                                                       %
% Licensed under the Apache License, Version 2.0 (the "License");       %
% you may not use this file except in compliance with the License.      %
% You may obtain a copy of the License at                               %
% http://www.apache.org/licenses/LICENSE-2.0.                           %
%                                                                       %
% Unless required by applicable law or agreed to in writing, software   %
% distributed under the License is distributed on an "AS IS" BASIS,     %
% WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or       %
% implied. See the License for the specific language governing          %
% permissions and limitations under the License.                        %
%-----------------------------------------------------------------------%

% Build an assistive device and test it on a simple testbed.

import org.opensim.modeling.*;

BuildTestbedModel;
% Show a visualization window when simulating.
testbed.setUseVisualizer(true);

% This script defines the 'device' variable.
BuildDevice;

device.dumpSubcomponentInfo();
testbed.dumpSubcomponentInfo();
device.dumpOutputInfo(true);

% Connect the device to the testbed.
testbedAttachment1 = 'ground';
testbedAttachment2 = 'load';
ConnectDeviceToModel(device, testbed, testbedAttachment1, testbedAttachment2);

% Use a SignalGenerator to create a control signal for testing the device.
AddSignalGeneratorToDevice(device);


% Create a new ConsoleReporter. Set its name and reporting interval.
reporter = ConsoleReporter();
reporter.setName([char(testbed.getName()) '_' char(device.getName()) '_results']);
reporter.set_report_time_interval(0.2) % seconds.
% Configure the outputs we wish to display during the simulation.
reporter.addToReport(device.getComponent('cableAtoB/geompath').getOutput('length'));
reporter.addToReport(device.getComponent('cableAtoB').getOutput('actuation'));
reporter.addToReport(device.getComponent('cableAtoB').getOutput('power'));
reporter.addToReport(device.getComponent('controller').getOutput('myo_control'));
testbed.addComponent(reporter);

sDev = testbed.initSystem();
Simulate(testbed, sDev);
