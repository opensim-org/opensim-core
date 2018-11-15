/* -------------------------------------------------------------------------- *
 *                       OpenSim:  buildHopperModel.cpp                       *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Chris Dembia, Shrinidhi K. Lakshmikanth, Ajay Seth,             *
 *            Thomas Uchida                                                   *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

/* Build an OpenSim model of a single-legged hopping mechanism. The mechanism
consists of a brick (pelvis) and two cylinders (thigh and shank segments), all
pin-connected, and is constrained to remain upright. A contact sphere (foot)
prevents the end of the lower cylinder from passing through the floor. A vastus
muscle wraps around a cylinder (patella) at the knee and a controller (brain)
generates the muscle excitation signal required to produce a small hop when the
mechanism is simulated. Two PhysicalOffsetFrames have been defined for attaching
an assistive device (to help the hopper hop higher); their absolute path names
are:

    - on the thigh: "/Dennis/bodyset/thigh/deviceAttachmentPoint"
    - on the shank: "/Dennis/bodyset/shank/deviceAttachmentPoint"

You don't need to add anything in this file, but you should know what
buildHopper() is doing. */

#include <OpenSim/OpenSim.h>

namespace OpenSim {

Model buildHopper(bool showVisualizer) {
    using SimTK::Vec3;
    using SimTK::Inertia;

    // Create a new OpenSim model on earth.
    auto hopper = Model();
    hopper.setName("Dennis");
    hopper.setGravity(Vec3(0, -9.80665, 0));

    // Create the pelvis, thigh, and shank bodies.
    double pelvisMass = 30., pelvisSideLength = 0.2;
    auto pelvisInertia = pelvisMass * Inertia::brick(Vec3(pelvisSideLength/2.));
    auto pelvis = new Body("pelvis", pelvisMass, Vec3(0), pelvisInertia);

    double linkMass = 10., linkLength = 0.5, linkRadius = 0.035;
    auto linkInertia = linkMass *
                       Inertia::cylinderAlongY(linkRadius, linkLength/2.);
    auto thigh = new Body("thigh", linkMass, Vec3(0), linkInertia);
    auto shank = new Body("shank", linkMass, Vec3(0), linkInertia);

    // Add the bodies to the model (the model takes ownership).
    hopper.addBody(pelvis);
    hopper.addBody(thigh);
    hopper.addBody(shank);

    // Attach the pelvis to ground with a vertical slider joint, and attach the
    // pelvis, thigh, and shank bodies to each other with pin joints.
    Vec3 sliderOrientation(0, 0, SimTK::Pi/2.);
    auto sliderToGround = new SliderJoint("slider", hopper.getGround(), Vec3(0),
                        sliderOrientation, *pelvis, Vec3(0), sliderOrientation);
    Vec3 linkDistalPoint(0, -linkLength/2., 0);
    Vec3 linkProximalPoint(0, linkLength/2., 0);
    // Define the pelvis as the parent so the reported value is hip flexion.
    auto hip = new PinJoint("hip", *pelvis, Vec3(0), Vec3(0), *thigh,
                            linkProximalPoint, Vec3(0));
    // Define the shank as the parent so the reported value is knee flexion.
    auto knee = new PinJoint("knee", *shank, linkProximalPoint, Vec3(0), *thigh,
                             linkDistalPoint, Vec3(0));

    // Add the joints to the model.
    hopper.addJoint(sliderToGround);
    hopper.addJoint(hip);
    hopper.addJoint(knee);

    // Set the coordinate names and default values. Note that we need "auto&"
    // here so that we get a reference to the Coordinate rather than a copy.
    auto& sliderCoord =
        sliderToGround->updCoordinate(SliderJoint::Coord::TranslationX);
    sliderCoord.setName("yCoord");
    sliderCoord.setDefaultValue(1.);

    auto& hipCoord = hip->updCoordinate(PinJoint::Coord::RotationZ);
    hipCoord.setName("hipFlexion");
    hipCoord.setDefaultValue(0.35);

    auto& kneeCoord = knee->updCoordinate(PinJoint::Coord::RotationZ);
    kneeCoord.setName("kneeFlexion");
    kneeCoord.setDefaultValue(0.75);

    // Limit the range of motion for the hip and knee joints.
    double hipRange[2] = {110., -90.};
    double hipStiff[2] = {20., 20.}, hipDamping = 5., hipTransition = 10.;
    auto hipLimitForce = new CoordinateLimitForce("hipFlexion", hipRange[0],
        hipStiff[0], hipRange[1], hipStiff[1], hipDamping, hipTransition);
    hip->addComponent(hipLimitForce);

    double kneeRange[2] = {140., 10.};
    double kneeStiff[2] = {50., 40.}, kneeDamping = 2., kneeTransition = 10.;
    auto kneeLimitForce = new CoordinateLimitForce("kneeFlexion", kneeRange[0],
        kneeStiff[0], kneeRange[1], kneeStiff[1], kneeDamping, kneeTransition);
    knee->addComponent(kneeLimitForce);

    // Create a constraint to keep the foot (distal end of the shank) directly
    // beneath the pelvis (the Y-axis points upwards).
    auto constraint = new PointOnLineConstraint(hopper.getGround(), Vec3(0,1,0),
                      Vec3(0), *shank, linkDistalPoint);
    shank->addComponent(constraint);

    // Use a contact model to prevent the foot (ContactSphere) from passing
    // through the floor (ContactHalfSpace).
    auto floor = new ContactHalfSpace(Vec3(0), Vec3(0, 0, -SimTK::Pi/2.),
                                      hopper.getGround(), "floor");
    double footRadius = 0.1;
    auto foot = new ContactSphere(footRadius, linkDistalPoint, *shank, "foot");

    double stiffness = 1.e8, dissipation = 0.5, friction[3] = {0.9, 0.9, 0.6};
    auto contactParams = new HuntCrossleyForce::ContactParameters(stiffness,
                         dissipation, friction[0], friction[1], friction[2]);
    contactParams->addGeometry("floor");
    contactParams->addGeometry("foot");
    auto contactForce = new HuntCrossleyForce(contactParams);

    // Add the contact-related components to the model.
    hopper.addContactGeometry(floor);
    hopper.addContactGeometry(foot);
    hopper.addForce(contactForce);

    // Create the vastus muscle and set its origin and insertion points.
    double mclFmax = 4000., mclOptFibLen = 0.55, mclTendonSlackLen = 0.25,
           mclPennAng = 0.;
    auto vastus = new Thelen2003Muscle("vastus", mclFmax, mclOptFibLen,
                                       mclTendonSlackLen, mclPennAng);
    vastus->addNewPathPoint("origin", *thigh, Vec3(linkRadius, 0.1, 0));
    vastus->addNewPathPoint("insertion", *shank, Vec3(linkRadius, 0.15, 0));
    hopper.addForce(vastus);

    // Attach a cylinder (patella) to the distal end of the thigh over which the
    // vastus muscle can wrap. 
    auto patellaFrame = new PhysicalOffsetFrame("patellaFrame",
        *thigh, SimTK::Transform(linkDistalPoint));
    auto patella = new WrapCylinder();
    patella->setName("patella");
    patella->set_radius(0.08);
    patella->set_length(linkRadius*2.);
    patella->set_quadrant("x");

    patellaFrame->addWrapObject(patella);
    thigh->addComponent(patellaFrame);

    // Configure the vastus muscle to wrap over the patella.
    vastus->updGeometryPath().addPathWrap(*patella);

    // Create a controller to excite the vastus muscle.
    auto brain = new PrescribedController();
    brain->setActuators(hopper.updActuators());
    double t[3] = {0.0, 2.0, 3.9}, x[3] = {0.3, 1.0, 0.1};
    auto controlFunction = new PiecewiseConstantFunction(3, t, x);
    brain->prescribeControlForActuator("vastus", controlFunction);
    hopper.addController(brain);

    // Create frames on the thigh and shank segments for attaching the device.
    auto thighAttachment = new PhysicalOffsetFrame("deviceAttachmentPoint",
                           *thigh, SimTK::Transform(Vec3(linkRadius, 0.15, 0)));
    auto shankAttachment = new PhysicalOffsetFrame("deviceAttachmentPoint",
                           *shank, SimTK::Transform(Vec3(linkRadius, 0, 0)));
    thigh->addComponent(thighAttachment);
    shank->addComponent(shankAttachment);

    // Attach geometry to the bodies and enable the visualizer.
    auto pelvisGeometry = new Brick(Vec3(pelvisSideLength/2.));
    pelvisGeometry->setColor(Vec3(0.8, 0.1, 0.1));
    pelvis->attachGeometry(pelvisGeometry);

    auto linkGeometry = new Cylinder(linkRadius, linkLength/2.);
    linkGeometry->setColor(Vec3(0.8, 0.1, 0.1));
    thigh->attachGeometry(linkGeometry);
    shank->attachGeometry(linkGeometry->clone());

    if(showVisualizer)
        hopper.setUseVisualizer(true);
    
    return hopper;
}

} // end of namespace OpenSim
