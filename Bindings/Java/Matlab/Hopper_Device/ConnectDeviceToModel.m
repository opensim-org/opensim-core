function ConnectDeviceToModel(device, model, modelFrameAname, modelFrameBname)
% Attaches the device to any two PhysicalFrames ina  model.
% TODO license

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
