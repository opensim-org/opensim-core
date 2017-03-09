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
function hopper = BuildHopper(varargin)
% Build a model of a one-leg hopper, with one muscle.

p = inputParser();
defaultMuscleModel = 'Thelen2003';
defaultMaxIsometricForce = 4000.0;
defaultMillardTendonParams = [0.049 28.1 0.67 0.5 0.25];
defaultActivation = [0.0 2.0 3.9;
                     0.3 1.0 0.1];
defaultPrintModel = false; 

addOptional(p,'muscleModel',defaultMuscleModel)
addOptional(p,'maxIsometricForce',defaultMaxIsometricForce)
addOptional(p,'MillardTendonParams',defaultMillardTendonParams)
addOptional(p,'activation',defaultActivation)
addOptional(p,'printModel',defaultPrintModel)

parse(p,varargin{:});

muscleModel = p.Results.muscleModel;
maxIsometricForce = p.Results.maxIsometricForce;
MillardTendonParams = p.Results.MillardTendonParams;
activation = p.Results.activation;
printModel = p.Results.printModel;

import org.opensim.modeling.*;

hopper = Model();
hopper.setName('Dennis')

%% Bodies and joints.
% -------------------
% Create the pelvis, thigh, and shank bodies.
pelvisMass = 30.0; pelvisHalfLength = 0.1;
pelvisInertia = Inertia(Vec3(pelvisMass * 2/3*pelvisHalfLength^2));
pelvis = Body('pelvis', pelvisMass, Vec3(0), pelvisInertia);

linkMass = 10.0; linkHalfLength = 0.25; linkRadius = 0.035;
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
% Define the pelvis as the parent so the reported value is hip flexion.
hip = PinJoint('hip', pelvis, Vec3(0),           Vec3(0), ...
                      thigh,  linkProximalPoint, Vec3(0));
% Define the shank as the parent so the reported value is knee flexion.
knee = PinJoint('knee', shank, linkProximalPoint, Vec3(0), ...
                        thigh, linkDistalPoint,   Vec3(0));

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


%% Passive force components.
% --------------------------
% Limit the range of motion for the hip and knee joints.
hipRange = [110., -90.];
hipStiff = [20., 20.]; hipDamping = 5.; hipTransition = 10.;
hipLimitForce = CoordinateLimitForce('hipFlexion', hipRange(1), ...
    hipStiff(1), hipRange(2), hipStiff(2), hipDamping, hipTransition);
hip.addComponent(hipLimitForce);

kneeRange = [140., 10.];
kneeStiff = [50., 40.]; kneeDamping = 2.; kneeTransition = 10.;
kneeLimitForce = CoordinateLimitForce('kneeFlexion', kneeRange(1), ...
    kneeStiff(1), kneeRange(2), kneeStiff(2), kneeDamping, kneeTransition);
knee.addComponent(kneeLimitForce);

% Create a constraint to keep the foot (distal end of the shank) directly
% beneath the pelvis (the Y-axis points upwards).
constraint = PointOnLineConstraint(hopper.getGround(), Vec3(0, 1 ,0), ...
        Vec3(0), shank, linkDistalPoint);
shank.addComponent(constraint);

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
mclFmax = maxIsometricForce; mclOptFibLen = 0.55; mclTendonSlackLen = 0.25;
mclPennAng = 0.;
switch muscleModel
    case 'Thelen2003'
        vastus = Thelen2003Muscle('vastus', mclFmax, mclOptFibLen, ...
            mclTendonSlackLen, mclPennAng);
    case 'Millard2012Equilibrium'
        vastus = Millard2012EquilibriumMuscle('vastus', mclFmax, mclOptFibLen, ...
            mclTendonSlackLen, mclPennAng);
        
        eIso = MillardTendonParams(1);
        kIso = MillardTendonParams(2);
        fToe = MillardTendonParams(3);
        curviness = MillardTendonParams(4);
        tendonFL = TendonForceLengthCurve(eIso,kIso,fToe,curviness);
        vastus.setTendonForceLengthCurve(tendonFL);
        
        lTs = MillardTendonParams(5);
        vastus.setTendonSlackLength(lTs);     
end
    
vastus.addNewPathPoint('origin', thigh, Vec3(linkRadius, 0.1, 0));
vastus.addNewPathPoint('insertion', shank, Vec3(linkRadius, 0.15, 0));
hopper.addForce(vastus);

%/ Attach a cylinder (patella) to the distal end of the thigh over which the
%/ vastus muscle can wrap. 
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
controlFunction = PiecewiseConstantFunction();

% controlFunction.addPoint(0.0, 0.3);
% controlFunction.addPoint(2.0, 1.0);
% controlFunction.addPoint(3.9, 0.1);

for i = 1:size(activation,2)
    controlFunction.addPoint(activation(1,i), activation(2,i));
end

brain.prescribeControlForActuator('vastus', controlFunction);
hopper.addController(brain);

% Device attachment frames.
% -------------------------
% Create frames on the thigh and shank segments for attaching the device.
thighAttachment = PhysicalOffsetFrame('deviceAttachmentPoint', thigh, ...
        Transform(Vec3(linkRadius, 0.15, 0)));
shankAttachment = PhysicalOffsetFrame('deviceAttachmentPoint', shank, ...
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

%% Print model
if printModel
    hopper.print('hopper.osim');
end

end
