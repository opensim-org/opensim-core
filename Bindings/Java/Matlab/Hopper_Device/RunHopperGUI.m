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

function RunHopperGUI(varargin)

p = inputParser();

defaultVisualize = true;
defaultMuscleActivation = [0.0 1.0 2.0 3.9;
                           0.0 0.3 1.0 0.1];
defaultAddPassiveDevice = false;
defaultPassivePatellaWrap = false;
defaultSpringStiffness = 1;
defaultAddActiveDevice = false;
defaultActivePatellaWrap = false;
defaultIsActivePropMyo = false;
defaultDeviceActivation = [0.0 1.0 2.0 3.9;
                           0.0 0.3 1.0 0.1];

addOptional(p,'visualize',defaultVisualize)
addOptional(p,'muscleActivation',defaultMuscleActivation)
addOptional(p,'addPassiveDevice',defaultAddPassiveDevice)
addOptional(p,'passivePatellaWrap',defaultPassivePatellaWrap)
addOptional(p,'springStiffness',defaultSpringStiffness)
addOptional(p,'addActiveDevice',defaultAddActiveDevice)
addOptional(p,'activePatellaWrap',defaultActivePatellaWrap)
addOptional(p,'isActivePropMyo',defaultIsActivePropMyo)
addOptional(p,'deviceActivation',defaultDeviceActivation)

parse(p,varargin{:});

visualize = p.Results.visualize;
muscleActivation = p.Results.muscleActivation;
addPassiveDevice = p.Results.addPassiveDevice;
passivePatellaWrap = p.Results.passivePatellaWrap;
springStiffness = p.Results.springStiffness;
addActiveDevice = p.Results.addActiveDevice;
activePatellaWrap = p.Results.activePatellaWrap;
isActivePropMyo = p.Results.isActivePropMyo;
deviceActivation = p.Results.deviceActivation;

import org.opensim.modeling.*;

% Build hopper model
hopper = BuildHopper('activation',muscleActivation);
hopper.printSubcomponentInfo();

% Build devices
devices = cell(0);
deviceNames = cell(0);
patellaWrap = cell(0);

if addPassiveDevice
    passive = BuildDevice('deviceType','passive','springStiffness',springStiffness);
    devices{1,length(devices)+1} = passive;
    deviceNames{1,length(deviceNames)+1} = 'passive';
    patellaWrap{1,length(patellaWrap)+1} = passivePatellaWrap;
    
end

if addActiveDevice
    if isActivePropMyo
        active = BuildDevice('deviceType','active','isPropMyo',true);
    else
        active = BuildDevice('deviceType','active','isPropMyo',false,'activation',deviceActivation);
    end
    
    devices{1,length(devices)+1} = active;
    deviceNames{1,length(deviceNames)+1} = 'active';
    patellaWrap{1,length(patellaWrap)+1} = activePatellaWrap;
    
end

%% Connect the devices to the hopper.
% ----------------------------------

for d = 1:length(devices)
    
    % Print the names of the device's subcomponents, and locate the
    % subcomponents named 'anchorA' and 'anchorB'. Also, print the names of
    % the hopper's subcomponents, and locate the two subcomponents named
    % 'deviceAttachmentPoint'.
    device = devices{d};
    device.printSubcomponentInfo();
    
    
    % Get the 'anchor' joints in the device, and downcast them to the
    % WeldJoint class. Get the 'deviceAttachmentPoint' frames in the hopper
    % model, and downcast them to the PhysicalFrame class.
    anchorA = WeldJoint.safeDownCast(device.updComponent('anchorA'));
    anchorB = WeldJoint.safeDownCast(device.updComponent('anchorB'));
    thighAttach = PhysicalFrame.safeDownCast(...
        hopper.getComponent('thigh/deviceAttachmentPoint'));
    shankAttach = PhysicalFrame.safeDownCast(...
        hopper.getComponent('shank/deviceAttachmentPoint'));
    
    % Connect the parent frame sockets of the device's anchor joints to the
    % attachment frames on the hopper; attach anchorA to the thigh, and
    % anchorB to the shank.
    anchorA.connectSocket_parent_frame(thighAttach);
    anchorB.connectSocket_parent_frame(shankAttach);
    
    % Add the device to the hopper model.
    hopper.addComponent(device);
    
    % Configure the device to wrap over the patella.
    if patellaWrap{d} && hopper.hasComponent(deviceNames{d})
        cable = PathActuator.safeDownCast(hopper.updComponent([deviceNames{d} '/cableAtoB']));
        patellaPath = 'thigh/patellaFrame/patella';
        wrapObject = WrapCylinder.safeDownCast(hopper.updComponent(patellaPath));
        cable.updGeometryPath().addPathWrap(wrapObject);
    end
    
    % Print the names of the outputs of the device's PathActuator and
    % ToyPropMyoController subcomponents.
    device.getComponent('cableAtoB').printOutputInfo();
    device.getComponent('controller').printOutputInfo();
    
    % Use the vastus muscle's activation output as the ToyPropMyoController's
    % activation input.
    if strcmp(deviceNames{d},'active') && isActivePropMyo
        device.updComponent('controller').updInput('activation').connect(...
            hopper.getComponent('vastus').getOutput('activation'));
    end
    
end
%% Report quantities of interest.
% -------------------------------
% Configure the outputs we wish to display during the simulation.
% Create a TableReporter, give it a name, and set its reporting interval
%   to 0.2 seconds. Wire the following outputs to the reporter:
%   - hopper's height,
%   - vastus muscle activation,
%   - device controller's control signal output.
% Then add the reporter to the hopper.
reporter = TableReporter();
reporterVector = TableReporterVector();
reporter.setName('hopper_device_results');
reporter.set_report_time_interval(0.2); % seconds.
reporter.addToReport(...
    hopper.getComponent('slider/yCoord').getOutput('value'), 'height');
reporter.addToReport(...
    hopper.getComponent('vastus').getOutput('activation'))
reporterVector.addToReport(...
      hopper.getComponent('Umberger').getOutput('probe_outputs'));
%reporter.addToReport(device.getComponent('controller').getOutput('myo_control'));
hopper.addComponent(reporter);

sHD = hopper.initSystem();

% The last argument determines if the simbody-visualizer should be used.
Simulate(hopper, sHD, visualize);

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

% if exist('reporterVector') == 1
%     % (Done for you) Display the TableReporter's data, and save it to a file.
%     table = reporterVector.getTable();
%     keyboard
%     disp(table.toString());
%     csv = CSVFileAdapter();
%     csv.write(table, 'hopper_device_results_Vector.csv');
%     
%     % (Done for you) Convert the TableReporter's Table to a MATLAB struct and plot
%     % the the hopper's height over the motion.
%     results = opensimTimeSeriesTableToMatlab(table);
%     
%     fieldnames(results)
%     if isfield(results, 'height')
%         plot(results.time, results.height);
%         xlabel('time');
%         ylabel('height');
%     end
% end

end