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
function ConnectDeviceToModel(device, model, modelFrameAname, modelFrameBname)
% Attaches the device to any two PhysicalFrames in a model.

import org.opensim.modeling.*;

% Get the 'anchor' joints in the device.
anchorA = WeldJoint.safeDownCast(device.updComponent('anchorA'));
anchorB = WeldJoint.safeDownCast(device.updComponent('anchorB'));

% Recall that the child frame of each anchor (WeldJoint) was attached to the
% corresponding cuff. We will now attach the parent frames of the anchors to
% modelFrameA and modelFrameB. First get references to the two specified
% PhysicalFrames in the model (i.e., modelFrameAname and modelFrameBname), then
% connect them to the parent frames of each anchor.
frameA = PhysicalFrame.safeDownCast(model.getComponent(modelFrameAname));
anchorA.connectSocket_parent_frame(frameA);
frameB = PhysicalFrame.safeDownCast(model.getComponent(modelFrameBname));
anchorB.connectSocket_parent_frame(frameB);

% Add the device to the model.
model.addComponent(device);

% Configure the device to wrap over the patella (if one exists; there is no
% patella in the testbed).
patellaPath = 'thigh/patellaFrame/patella';
if model.hasComponent(patellaPath)
    cable = PathActuator.safeDownCast(model.updComponent('device/cableAtoB'));
    wrapObject = WrapCylinder.safeDownCast(model.updComponent(patellaPath));
    cable.updGeometryPath().addPathWrap(wrapObject);
end

end
