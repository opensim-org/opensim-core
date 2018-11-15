function hopper = BuildHopper(varargin)
% Build a model of a one-leg hopper, with one muscle.
%
% Optional parameters
% -------------------
% maxIsometricForce: The max isometric force of the muscle (in Newtons).
% optimalFiberLength: The optimal fiber length of the muscle (in meters).
% tendonSlackLength: The slack length of the tendon (in meters).
% excitation: The excitation control signal for the muscle. This should be
%   a matrix where the first row contains time and the second row contains the
%   excitation signal.
% printModel (bool): Print the resulting model to 'hopper.osim'?
% additionalMass: Add additional mass (in units of kg) to the pelvis.
%
% The arguments must be passed as key-value pairs; for example:
%
%   BuildDevice('maxIsometricForce', 3000.0, 'printModel', true);
%
% Consult the code for this function to learn the parameters' default values.

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

p = inputParser();
defaultMaxIsometricForce = 4000.0;
defaultOptimalFiberLength = 0.55;
defaultTendonSlackLength = 0.25;
defaultExcitation = [0.0 1.99 2.0 3.89 3.9 4.0;
                     0.3 0.3  1.0 1.0  0.1 0.1];
defaultPrintModel = false;
defaultAdditionalMass = 0;

addOptional(p, 'maxIsometricForce', defaultMaxIsometricForce)
addOptional(p, 'optimalFiberLength', defaultOptimalFiberLength)
addOptional(p, 'tendonSlackLength', defaultTendonSlackLength);
addOptional(p, 'excitation', defaultExcitation)
addOptional(p, 'printModel', defaultPrintModel)
addOptional(p, 'additionalMass', defaultAdditionalMass)

parse(p,varargin{:});

maxIsometricForce = p.Results.maxIsometricForce;
optimalFiberLength = p.Results.optimalFiberLength;
tendonSlackLength = p.Results.tendonSlackLength;
excitation = p.Results.excitation;
printModel = p.Results.printModel;
additionalMass = p.Results.additionalMass;

import org.opensim.modeling.*;

hopper = Model();
hopper.setName('Dennis')

%% Bodies and joints.
% -------------------
% Create the pelvis, thigh, and shank bodies.
pelvisMass = 25.0 + additionalMass;
pelvisHalfLength = 0.1;
pelvisInertia = Inertia(Vec3(pelvisMass * 2/3*pelvisHalfLength^2));
pelvis = Body('pelvis', pelvisMass, Vec3(0), pelvisInertia);

linkMass = 2.5; linkHalfLength = 0.25; linkRadius = 0.035;
linkIxx = linkMass * (linkRadius^2 / 4 + linkHalfLength^2 / 3);
linkInertia = Inertia(Vec3(linkIxx, linkMass * linkRadius^2 / 2, linkIxx));
thigh = Body('thigh', linkMass, Vec3(0), linkInertia);
shank = Body('shank', linkMass, Vec3(0), linkInertia);

% Add the bodies to the model (the model takes ownership).
hopper.addBody(pelvis);
hopper.addBody(thigh);
hopper.addBody(shank);

% Attach the pelvis to ground with a vertical slider joint, and attach the
% pelvis, thigh, and shank bodies to each other with pin joints.
sliderOrientation = Vec3(0, 0, pi/2);
sliderToGround = SliderJoint('slider', ...
        hopper.getGround(), Vec3(0), sliderOrientation, ...
        pelvis,             Vec3(0), sliderOrientation);
linkDistalPoint = Vec3(0, -linkHalfLength, 0);
linkProximalPoint = Vec3(0, linkHalfLength, 0);
% Hip. Define the pelvis as the parent so the reported value is hip flexion.
hipThighOffset = PhysicalOffsetFrame('hip_thigh_offset', thigh, ...
        Transform(linkProximalPoint));
hip = PinJoint();
hip.setName('hip');
hip.connectSocket_parent_frame(pelvis);
hip.connectSocket_child_frame(hipThighOffset);
hip.addComponent(hipThighOffset);
% Knee. Define the shank as the parent so the reported value is knee flexion.
kneeShankOffset = PhysicalOffsetFrame('knee_shank_offset', shank, ...
        Transform(linkProximalPoint));
kneeThighOffset = PhysicalOffsetFrame('knee_thigh_offset', thigh, ...
        Transform(linkDistalPoint));
knee = PinJoint('knee', kneeShankOffset, kneeThighOffset);
knee.addComponent(kneeShankOffset);
knee.addComponent(kneeThighOffset);

%/ Add the joints to the model.
hopper.addJoint(sliderToGround);
hopper.addJoint(hip);
hopper.addJoint(knee);

% Set the coordinate names and default values.
sliderCoord = sliderToGround.upd_coordinates(0);
sliderCoord.setName('yCoord');
sliderCoord.setDefaultValue(1.);

hipCoord = hip.upd_coordinates(0);
hipCoord.setName('hipFlexion');
hipCoord.setDefaultValue(0.35);

kneeCoord = knee.upd_coordinates(0);
kneeCoord.setName('kneeFlexion');
kneeCoord.setDefaultValue(0.75);

%% Constraint.
% ------------
% Create a constraint to keep the foot (distal end of the shank) directly
% beneath the pelvis (the Y-axis points upwards).
constraint = PointOnLineConstraint(hopper.getGround(), Vec3(0, 1 ,0), ...
        Vec3(0), shank, linkDistalPoint);
shank.addComponent(constraint);


%% Passive force components.
% --------------------------
% Limit the range of motion for the hip and knee joints.
hipRange = [110., -90.];
hipStiff = [20., 20.]; hipDamping = 5.; hipTransition = 10.;
hipLimitForce = CoordinateLimitForce('hipFlexion', hipRange(1), ...
    hipStiff(1), hipRange(2), hipStiff(2), hipDamping, hipTransition);
hipLimitForce.setName('hipLigaments');
hip.addComponent(hipLimitForce);

kneeRange = [140., 10.];
kneeStiff = [50., 40.]; kneeDamping = 2.; kneeTransition = 10.;
kneeLimitForce = CoordinateLimitForce('kneeFlexion', kneeRange(1), ...
    kneeStiff(1), kneeRange(2), kneeStiff(2), kneeDamping, kneeTransition);
kneeLimitForce.setName('kneeLigaments');
knee.addComponent(kneeLimitForce);

% Use a contact model to prevent the foot (ContactSphere) from passing
% through the floor (ContactHalfSpace).
floor = ContactHalfSpace(Vec3(0), Vec3(0, 0, -pi/2), hopper.getGround(), ...
        'floor');
footRadius = 0.07;
foot = ContactSphere(footRadius, linkDistalPoint, shank, 'foot');

stiffness = 1.e8; dissipation = 0.5; friction = [0.9, 0.9, 0.6];
contactForce = HuntCrossleyForce();
contactForce.setStiffness(stiffness);
contactForce.setDissipation(dissipation);
contactForce.setStaticFriction(friction(1));
contactForce.setDynamicFriction(friction(2));
contactForce.setViscousFriction(friction(3));
contactForce.addGeometry('floor');
contactForce.addGeometry('foot');

% Add the contact-related components to the model.
hopper.addContactGeometry(floor);
hopper.addContactGeometry(foot);
hopper.addForce(contactForce);

%% Actuator.
% ----------
% Create the vastus muscle and set its origin and insertion points.
pennationAngle = 0.0;
vastus = Millard2012EquilibriumMuscle('vastus', maxIsometricForce, ...
            optimalFiberLength, tendonSlackLength, pennationAngle);
    
vastus.addNewPathPoint('origin', thigh, Vec3(linkRadius, 0.1, 0));
vastus.addNewPathPoint('insertion', shank, Vec3(linkRadius, 0.15, 0));
hopper.addForce(vastus);

% Attach a cylinder (patella) to the distal end of the thigh over which the
% vastus muscle can wrap. 
patellaFrame = PhysicalOffsetFrame('patellaFrame', thigh, ...
        Transform(linkDistalPoint));
patella = WrapCylinder();
patella.setName('patella');
patella.set_radius(0.08);
patella.set_length(linkRadius*2.);
patella.set_quadrant('x');
patellaFrame.addWrapObject(patella);
thigh.addComponent(patellaFrame);
% Configure the vastus muscle to wrap over the patella.
vastus.updGeometryPath().addPathWrap(patella);


%% Controller.
% ------------
% Create a controller to excite the vastus muscle.
brain = PrescribedController();
brain.setActuators(hopper.updActuators());
controlFunction = PiecewiseLinearFunction();

for i = 1:size(excitation,2)
    controlFunction.addPoint(excitation(1,i), excitation(2,i));
end

brain.prescribeControlForActuator('vastus', controlFunction);
hopper.addController(brain);

% Device attachment frames.
% -------------------------
% Create frames on the thigh and shank segments for attaching the device.
thighAttachment = PhysicalOffsetFrame('deviceAttach', thigh, ...
        Transform(Vec3(linkRadius, 0.15, 0)));
shankAttachment = PhysicalOffsetFrame('deviceAttach', shank, ...
        Transform(Vec3(linkRadius, 0, 0)));
thigh.addComponent(thighAttachment);
shank.addComponent(shankAttachment);


%% Display geometry.
% ------------------
% Attach geometry to the bodies and enable the visualizer.
pelvisGeometry = Brick(Vec3(pelvisHalfLength));
pelvisGeometry.setColor(Vec3(0.8, 0.1, 0.1));
pelvis.attachGeometry(pelvisGeometry);

linkGeometry = Cylinder(linkRadius, linkHalfLength);
linkGeometry.setColor(Vec3(0.8, 0.1, 0.1));
thigh.attachGeometry(linkGeometry);
shank.attachGeometry(linkGeometry.clone());

hopper.finalizeConnections();

%% Print model
if printModel
    hopper.print('hopper.osim');
end

end
