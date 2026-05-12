/* -------------------------------------------------------------------------- *
 *                OpenSim:  exampleExponentialContactForce.cpp                *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2026 Stanford University and the Authors                     *
 * Author(s): Nicholas Bianco, F. C. Anderson                                 *
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

// This example demonstrates how to simulate a bouncing block using the
// ExponentialContactForce class.
//
// ExponentialContactForce uses a normal force model based on exponential spring
// functions. The exponential springs avoid expensive contact detection
// algorithms by always applying a normal force to the body, becoming
// negligible small as soon as the body is more than a few centimeters above
// the contact plane. Addtional speed up is achieved through a spring-based
// frictional model that includes a velocity-dependent damping term and a
// position-dependent elastic term, the latter eliminating drift entirely
// to maintain large integration step sizes.

#include <OpenSim/OpenSim.h>

using namespace OpenSim;

int main() {

    // Create a model with a single body representing a block that can move
    // relative to the ground via a free joint.
    Model model;
    model.setName("bouncing_block");

    // Block body.
    Body* block = new Body();
    block->setName("block");
    block->set_mass(10.0);
    block->set_mass_center(SimTK::Vec3(0));
    block->setInertia(SimTK::Inertia(1.0));
    // Block geometry.
    Brick* blockGeometry = new Brick(SimTK::Vec3(0.1));
    block->attachGeometry(blockGeometry);

    // Ground-to-block free joint.
    FreeJoint* free = new FreeJoint("free_joint",
        model.getGround(), SimTK::Vec3(0), SimTK::Vec3(0),
        *block, SimTK::Vec3(0), SimTK::Vec3(0));
    model.addBody(block);
    model.addJoint(free);

    // Create the transform that defines the ground plane for the contact force.
    // The transform rotates about the x-axis by -90 degrees so that the
    // positive z-axis of the contact plane (i.e., the normal direction)
    // aligns with the positive y-axis of the ground frame, which is the upward
    // direction in OpenSim.
    const SimTK::Rotation floorRotation(-SimTK::Pi/2.0, SimTK::XAxis);
    const SimTK::Transform floorTransform(floorRotation, SimTK::Vec3(0.));

    // Add an ExponentialContactForce at each corner of the block.
    const double hs = 0.1; // half the side length of the block
    SimTK::Vector_<SimTK::Vec3> corners(8, SimTK::Vec3(0));
    corners[0] = SimTK::Vec3( hs, -hs,  hs);
    corners[1] = SimTK::Vec3( hs, -hs, -hs);
    corners[2] = SimTK::Vec3(-hs, -hs, -hs);
    corners[3] = SimTK::Vec3(-hs, -hs,  hs);
    corners[4] = SimTK::Vec3( hs,  hs,  hs);
    corners[5] = SimTK::Vec3( hs,  hs, -hs);
    corners[6] = SimTK::Vec3(-hs,  hs, -hs);
    corners[7] = SimTK::Vec3(-hs,  hs,  hs);
    for (int i = 0; i < 8; ++i) {
        auto* esf = new ExponentialContactForce(floorTransform, *block,
            corners[i]);
        esf->setName("ExpSprForce" + std::to_string(i));
        model.addForce(esf);
    }

    // Finalize the model and initialize the system.
    model.finalizeConnections();
    SimTK::State state = model.initSystem();

    // Set the block initial conditions.
    SimTK::Vec3 pos(-1.5, 2.0 * hs, 1.0);
    SimTK::Vec3 vel(-1.0, 0.0, 0.0);
    SimTK::Vec3 angvel(0.0, 0.0, 2.0 * SimTK::Pi);
    block->getMobilizedBody().setQToFitTranslation(state, pos);
    block->getMobilizedBody().setUToFitLinearVelocity(state, vel);
    block->getMobilizedBody().setUToFitAngularVelocity(state, angvel);

    // Simulate the model.
    Manager manager(model);
    manager.setIntegratorAccuracy(1e-4);
    manager.initialize(state);
    manager.setWriteToStorage(true);
    state = manager.integrate(5.0);

    // Visualize the results.
    TimeSeriesTable statesTable = manager.getStatesTable();
    VisualizerUtilities::showMotion(model, statesTable);
}
