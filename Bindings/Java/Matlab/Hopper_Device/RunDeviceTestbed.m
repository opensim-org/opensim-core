% TODO license.

% Build an assistive device and test it on a simple testbed.

import org.opensim.modeling.*;

BuildTestbedModel;
% Show a visualization window when simulating.
testbed.setUseVisualizer(true);

% This script defines the 'device' variable.
BuildDevice;

device.dumpSubcomponentInfo();
testbed.dumpSubcomponentInfo();
% TODO showAllOutputs(device);

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
