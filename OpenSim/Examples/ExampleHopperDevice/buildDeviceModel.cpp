/* -------------------------------------------------------------------------- *
 *                       OpenSim:  buildDeviceModel.cpp                       *
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

/* Build an OpenSim model of a device for assisting our hopping mechanism. The
device consists of two bodies ("cuffA" and "cuffB") connected by a PathActuator
("cableAtoB") that can wrap around the hopper's patella (similar to the vastus
muscle in the hopper). The actuator receives its control signal from a
PropMyoController. Each cuff is attached to the "child" frame of a WeldJoint;
the "parent" frames of these joints will be connected to PhysicalFrames on the
hopper or testbed.

Several lines of code need to be added to this file; see exampleHopperDevice.cpp
and the "TODO" comments below for instructions. */

#include <OpenSim/OpenSim.h>
#include "defineDeviceAndController.h"

static const double OPTIMAL_FORCE{ 4000. };
static const double GAIN{ 1.0 };

namespace OpenSim {

// [Step 2, Task C]
Device* buildDevice() {
    using SimTK::Vec3;
    using SimTK::Inertia;

    // Create the device.
    auto device = new Device();
    device->setName("device");

    // The device's mass is distributed between two identical cuffs that attach
    // to the hopper via WeldJoints (to be added below).
    double deviceMass = 2.0;
    auto cuffA = new Body("cuffA", deviceMass/2., Vec3(0), Inertia(0.5));
    //TODO: Repeat for cuffB.

    //TODO: Add the cuff Components to the device.

    // Attach a sphere to each cuff for visualization.
    auto sphere = new Sphere(0.01);
    sphere->setName("sphere");
    sphere->setColor(SimTK::Red);
    cuffA->attachGeometry(sphere);
    //cuffB->attachGeometry(sphere->clone());

    // Create a WeldJoint to anchor cuffA to the hopper.
    auto anchorA = new WeldJoint();
    anchorA->setName("anchorA");
    //TODO: Connect the "child_frame" (a PhysicalFrame) Socket of anchorA to
    //      cuffA. Note that only the child frame is connected now; the parent
    //      frame will be connected in exampleHopperDevice.cpp.

    //TODO: Add anchorA to the device.

    //TODO: Create a WeldJoint to anchor cuffB to the hopper. Connect the
    //      "child_frame" Socket of anchorB to cuffB and add anchorB to the
    //      device.

    // Attach a PathActuator between the two cuffs.
    auto pathActuator = new PathActuator();
    pathActuator->setName("cableAtoB");
    pathActuator->set_optimal_force(OPTIMAL_FORCE);
    pathActuator->addNewPathPoint("pointA", *cuffA, Vec3(0));
    //pathActuator->addNewPathPoint("pointB", *cuffB, Vec3(0));
    device->addComponent(pathActuator);

    // Create a PropMyoController.
    auto controller = new PropMyoController();
    controller->setName("controller");
    controller->set_gain(GAIN);

    //TODO: Connect the controller's "actuator" Socket to pathActuator.

    //TODO: Add the controller to the device.

    return device;
}

} // end of namespace OpenSim
