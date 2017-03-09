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

% This function builds the hopper model; no need to edit it.
hopper = BuildHopper();

% This function builds the device component; no need to edit it.
device = BuildDevice();

%% Connect the device to the hopper.
% ----------------------------------

% TODO: Print the names of the device's subcomponents, and locate the
%       subcomponents named 'anchorA' and 'anchorB'. Also, print the names of
%       the hopper's subcomponents, and locate the two subcomponents named
%       'deviceAttachmentPoint'.
% [Step 2, Task A]
% ANSWER{
device.printSubcomponentInfo();
hopper.printSubcomponentInfo();
% }

% TODO: Get the 'anchor' joints in the device, and downcast them to the
%       WeldJoint class. Get the 'deviceAttachmentPoint' frames in the hopper
%       model, and downcast them to the PhysicalFrame class.
% [Step 2, Task B]
% ANSWER{
anchorA = WeldJoint.safeDownCast(device.updComponent('anchorA'));
anchorB = WeldJoint.safeDownCast(device.updComponent('anchorB'));
thighAttach = PhysicalFrame.safeDownCast(...
        hopper.getComponent('thigh/deviceAttachmentPoint'));
shankAttach = PhysicalFrame.safeDownCast(...
        hopper.getComponent('shank/deviceAttachmentPoint'));
% }

% TODO: Connect the parent frame sockets of the device's anchor joints to the
%       attachment frames on the hopper; attach anchorA to the thigh, and
%       anchorB to the shank.
% [Step 2, Task C]
% ANSWER{
anchorA.connectSocket_parent_frame(thighAttach);
anchorB.connectSocket_parent_frame(shankAttach);
% }

% TODO: Add the device to the hopper model.
% [Step 2, Task D]
% ANSWER{
hopper.addComponent(device);
% }

% (Done for you) Configure the device to wrap over the patella.
if hopper.hasComponent('device')
    cable = PathActuator.safeDownCast(hopper.updComponent('device/cableAtoB'));
    patellaPath = 'thigh/patellaFrame/patella';
    wrapObject = WrapCylinder.safeDownCast(hopper.updComponent(patellaPath));
    cable.updGeometryPath().addPathWrap(wrapObject);
end

% TODO: Print the names of the outputs of the device's PathActuator and
%       ToyPropMyoController subcomponents.
% [Step 2, Task E]
% ANSWER{
device.getComponent('cableAtoB').printOutputInfo();
device.getComponent('controller').printOutputInfo();
% TODO device.getComponent('controller').dumpInputInfo();
% }

% TODO: Use the vastus muscle's activation output as the ToyPropMyoController's  
%       activation input.
% [Step 2, Task F]
% ANSWER{
device.updComponent('controller').updInput('activation').connect(...
    hopper.getComponent('vastus').getOutput('activation'));
% }


%% Report quantities of interest.
% -------------------------------
% Configure the outputs we wish to display during the simulation.
% TODO: Create a TableReporter, give it a name, and set its reporting interval
%       to 0.2 seconds. Wire the following outputs to the reporter:
%         - hopper's height,
%         - vastus muscle activation,
%         - device controller's control signal output.
%       Then add the reporter to the hopper.
% [Step 2, Task G]
% ANSWER{
reporter = TableReporter();
reporter.setName('hopper_device_results');
reporter.set_report_time_interval(0.2); % seconds.
reporter.addToReport(...
    hopper.getComponent('slider/yCoord').getOutput('value'), 'height');
reporter.addToReport(...
    hopper.getComponent('vastus').getOutput('activation'));
reporter.addToReport(device.getComponent('controller').getOutput('myo_control'));
hopper.addComponent(reporter);
% }

sHD = hopper.initSystem();
% The last argument determines if the simbody-visualizer should be used.
Simulate(hopper, sHD, true);

if exist('reporter') == 1
    % (Done for you) Display the TableReporter's data, and save it to a file.
    table = reporter.getTable();
    disp(table.toString());
    csv = CSVFileAdapter();
    csv.write(table, 'hopper_device_results.csv');
      
    % (Done for you) Convert the TableReporter's Table to a MATLAB struct and plot
    % the the hopper's height over the motion.
    results = opensimTimeSeriesTableToMatlab(table);
    fieldnames(results)
    if isfield(results, 'height')
        plot(results.time, results.height);
        xlabel('time');
        ylabel('height');
    end
end
