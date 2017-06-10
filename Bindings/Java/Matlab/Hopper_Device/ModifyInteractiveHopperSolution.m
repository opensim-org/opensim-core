function [hopper] = ModifyInteractiveHopperSolution(hopper, varargin)
% This function is used to modify an InteractiveHopper GUI solution.

%-----------------------------------------------------------------------%
% The OpenSim API is a toolkit for musculoskeletal modeling and         %
% simulation. See http://opensim.stanford.edu and the NOTICE file       %
% for more information. OpenSim is developed at Stanford University     %
% and supported by the US National Institutes of Health (U54 GM072970,  %
% R24 HD065690) and by DARPA through the Warrior Web program.           %
%                                                                       %
% Copyright (c) 2017 Stanford University and the Authors                %
% Author(s): Nick Bianco                                                %
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

%% INPUT PARSING 

% Create input parser
p = inputParser();

% Default values if no input is specified
defaultMuscle = 'averageJoe';
defaultMuscleExcitation = [0.0 1.99 2.0 3.89 3.9 4.0;
                          0.3 0.3  1.0 1.0  0.1 0.1];
defaultAddPassiveDevice = false;
defaultPassivePatellaWrap = false;
defaultPassiveParameter = 50;
defaultAddActiveDevice = false;
defaultActivePatellaWrap = false;
defaultIsActivePropMyo = false;
defaultActiveParameter = 50;
defaultDeviceControl = [0.0 2.5 5.0;
                        0.0 0.75 0.0];

% Create optional values for input parser                   
addOptional(p,'muscle',defaultMuscle)
addOptional(p,'muscleExcitation',defaultMuscleExcitation)
addOptional(p,'addPassiveDevice',defaultAddPassiveDevice)
addOptional(p,'passivePatellaWrap',defaultPassivePatellaWrap)
addOptional(p,'passiveParameter',defaultPassiveParameter)
addOptional(p,'addActiveDevice',defaultAddActiveDevice)
addOptional(p,'activePatellaWrap',defaultActivePatellaWrap)
addOptional(p,'isActivePropMyo',defaultIsActivePropMyo)
addOptional(p,'activeParameter',defaultActiveParameter)
addOptional(p,'deviceControl',defaultDeviceControl)

% Parse inputs
parse(p,varargin{:});

% Retrieve inputs and/or default values
muscle = p.Results.muscle;
muscleExcitation = p.Results.muscleExcitation;
addPassiveDevice = p.Results.addPassiveDevice;
passivePatellaWrap = p.Results.passivePatellaWrap;
passiveParameter = p.Results.passiveParameter;
addActiveDevice = p.Results.addActiveDevice;
activePatellaWrap = p.Results.activePatellaWrap;
isActivePropMyo = p.Results.isActivePropMyo;
activeParameter = p.Results.activeParameter;
deviceControl = p.Results.deviceControl;

import org.opensim.modeling.*;

%% HOPPER AND DEVICE SETTINGS

additionalMass = 0;

% Retrieve muscle settings based on user selection 
%   default: "The Average Joe"
[muscleFunc] = InteractiveHopperSettings(muscle);
[maxIsometricForce,tendonStiffness,tendonSlackLength,muscleMass] = muscleFunc();
MillardTendonParams = [0.049 tendonStiffness 0.67 0.5 tendonSlackLength];
additionalMass = additionalMass + muscleMass;

% Retreive passive device settings if passive device specified
%   default: no passive device
%            if device --> passiveParameter = 50
if addPassiveDevice
    passive = InteractiveHopperSettings('passive');
    [passiveMass,springStiffness] = passive(passiveParameter);
    additionalMass = additionalMass + passiveMass;
end

% Retreive passive device settings if passive device specified
%   default: no active device
%            if device --> activeParameter = 50
if addActiveDevice
    if isActivePropMyo
        activePropMyo = InteractiveHopperSettings('activePropMyo');
        [activeMass,gain] = activePropMyo(activeParameter);
    else
        activeControl = InteractiveHopperSettings('activeControl');
        [activeMass,maxTension] = activeControl(activeParameter);
    end
    additionalMass = additionalMass + activeMass;
end

%% MODIFY HOPPER MODEL

% Build hopper
hopper = ModifyHopper(hopper, ...
                     'excitation',muscleExcitation, ...
                     'additionalMass',additionalMass, ...
                     'MillardTendonParams', MillardTendonParams, ...
                     'maxIsometricForce', maxIsometricForce);
hopper.printSubcomponentInfo();

%% MODIFY DEVICES
devices = cell(0);
deviceNames = cell(0);
patellaWrap = cell(0);

% Passive device
if addPassiveDevice
    passive = ModifyDevice(hopper, ...
                          'deviceType','passive', ... 
                          'springStiffness',springStiffness, ...
                          'passivePatellaWrap',passivePatellaWrap);
    devices{1,length(devices)+1} = passive;
    deviceNames{1,length(deviceNames)+1} = 'device_passive';
    patellaWrap{1,length(patellaWrap)+1} = passivePatellaWrap;  
end

% Active device
if addActiveDevice
    if isActivePropMyo
        active = ModifyDevice(hopper, ...
                             'deviceType','active', ...
                             'isPropMyo',true, ...
                             'gain',gain);
    else
        active = ModifyDevice(hopper, ...
                             'deviceType','active', ...
                             'isPropMyo',false, ... 
                             'control',deviceControl, ...
                             'maxTension',maxTension);
    end
    
    devices{1,length(devices)+1} = active;
    deviceNames{1,length(deviceNames)+1} = 'device_active';
    patellaWrap{1,length(patellaWrap)+1} = activePatellaWrap;   
end

end

function ModifyHopper(hopper, varargin)



end

function ModifyDevices(hopper, varargin)


end
