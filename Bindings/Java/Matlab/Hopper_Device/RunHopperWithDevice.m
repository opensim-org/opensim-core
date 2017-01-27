% TODO license

% Connect the device to the hopper to increase hop height.

import org.opensim.modeling.*;

% This script defines the 'hopper' variable.
BuildHopperModel;
hopper.setUseVisualizer(true);

% This script defines the 'device' variable.
BuildDevice;

% Connect the device to the hopper.
thighAttachment = '/Dennis/thigh/deviceAttachmentPoint';
shankAttachment = '/Dennis/shank/deviceAttachmentPoint';
ConnectDeviceToModel(device, hopper, thighAttachment, shankAttachment);

% Use the vastus muscle's activation as the control signal for the device.
device.updComponent('controller').updInput('activation').connect(...
    hopper.getComponent('vastus').getOutput('activation'));

% Configure the outputs we wish to display during the simulation.
reporter = ConsoleReporter();
reporter.setName([char(hopper.getName()) '_' char(device.getName()) '_results']);
reporter.set_report_time_interval(0.2) % seconds.
reporter.addToReport(...
    hopper.getComponent('/Dennis/slider/yCoord').getOutput('value'), 'height');
reporter.addToReport(device.getComponent('cableAtoB').getOutput('actuation'));
reporter.addToReport(device.getComponent('controller').getOutput('myo_control'));
hopper.addComponent(reporter);

sHD = hopper.initSystem();
Simulate(hopper, sHD);
