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

% Connect the device to the hopper to increase hop height.

import org.opensim.modeling.*;

% This script defines the 'hopper' variable.
hopper = BuildHopperModel();

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
reporter.set_report_time_interval(0.2); % seconds.
reporter.addToReport(...
    hopper.getComponent('/Dennis/slider/yCoord').getOutput('value'), 'height');
reporter.addToReport(device.getComponent('cableAtoB').getOutput('actuation'));
reporter.addToReport(device.getComponent('controller').getOutput('myo_control'));
hopper.addComponent(reporter);

sHD = hopper.initSystem();
% The last argument determines if the simbody-visualizer should be used.
Simulate(hopper, sHD, true);

% This line helps prevent MATLAB from crashing when using simbody-visualizer.
java.lang.System.gc();
