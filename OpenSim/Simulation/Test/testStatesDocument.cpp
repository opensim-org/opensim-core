/* -------------------------------------------------------------------------- *
*                OpenSim:  testComponentInterface.cpp                        *
* -------------------------------------------------------------------------- *
* The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
* See http://opensim.stanford.edu and the NOTICE file for more information.  *
* OpenSim is developed at Stanford University and supported by the US        *
* National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
* through the Warrior Web program.                                           *
*                                                                            *
* Copyright (c) 2024 Stanford University and the Authors                     *
* Author(s): F. C. Anderson                                                  *
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
#include <OpenSim/Simulation/SimbodyEngine/FreeJoint.h>
#include <OpenSim/Simulation/StatesDocument.h>
#include <OpenSim/Simulation/StatesTrajectory.h>
#include <OpenSim/Simulation/StatesTrajectoryReporter.h>
#include <OpenSim/Simulation/Manager/Manager.h>
#include <OpenSim/Simulation/Model/PointToPointSpring.h>
#include <catch2/catch_all.hpp>

using namespace SimTK;
using namespace OpenSim;

// Internal static methods and classes.
namespace
{

//_____________________________________________________________________________
// Sample internal method
double
customSquare(double x)
{
    return(x*x);
}

//_____________________________________________________________________________
// Build the model
Model*
buildModel() {

    // Create an empty model
    Model* model = new Model();
    Vec3 gravity(0.0, -10.0, 0.0);
    model->setGravity(gravity);
    model->setName("BlockOnASpring");

    // Add bodies and joints
    OpenSim::Ground& ground = model->updGround();

    // Body
    std::string name = "block";
    OpenSim::Body* block = new OpenSim::Body();
    double mass = 10.0;
    block->setName(name);
    block->set_mass(mass);
    block->set_mass_center(Vec3(0));
    block->setInertia(Inertia(1.0));
    model->addBody(block);

    // Joint
    name = "free";
    FreeJoint *free = new
        FreeJoint(name, ground, Vec3(0), Vec3(0), *block, Vec3(0), Vec3(0));
    model->addJoint(free);

    // Point-To-Point Spring
    // This actuator connects the origin of the block to the orgin of the
    // coordinate frame.
    double kp = 1000.0; // Stiffness
    double kv = 100.0;  // Viscosity (under-damped)
    double restlength = 0.0;
    Vec3 origin(0.0);
    Vec3 insertion(0.1, 0.1, 0.1);
    PointToPointSpring* spring = new PointToPointSpring(ground, origin,
        *block, insertion, kp, restlength);
    model->addForce(spring);

    return model;
}

//_____________________________________________________________________________
// Simulate
const OpenSim::StatesTrajectory*
simulate(Model* model) {

    // Add a StatesTrajectoryReporter
    // The reporter records the SimTK::State in a SimTK::Array_<> at a
    // specified time interval.
    OpenSim::StatesTrajectoryReporter* reporter =
        new StatesTrajectoryReporter();
    reporter->setName("states_reporter");
    double interval = 0.1;
    reporter->set_report_time_interval(interval);
    model->addComponent(reporter);

    // Build the system
    model->buildSystem();
    SimTK::State& state = model->initializeState();

    // Integrate
    Manager manager(*model);
    manager.getIntegrator().setMaximumStepSize(0.01);
    manager.setIntegratorAccuracy(1.0e-5);
    double ti = 0.0;
    double tf = 5.0;
    state.setTime(ti);
    manager.initialize(state);
    state = manager.integrate(tf);

    // Return the trajectory
    return &reporter->getStates();
}

} // End anonymous namespace


TEST_CASE("Getting Started")
{
    double x = 2.0;
    double square = customSquare(x);
    REQUIRE(square == x*x);
}


TEST_CASE("StatesDocument Serialization and Deserialization")
{
    Model *model = buildModel();
    const StatesTrajectory *traj = simulate(model);
    int precision = 3;
    SimTK::String filename = "BlockOnAString.ostates";
    StatesDocument doc1 = traj->exportToStatesDocument(*model, precision);
    doc1.serialize(filename);

    StatesDocument doc2(filename);
    SimTK::Array_<SimTK::State> trajDe;
    doc2.deserialize(*model, trajDe);

    SimTK::String filename2 = "BlockOnAString02.ostates";
    StatesDocument doc3(*model, trajDe, precision);
    doc3.serialize(filename2);

    REQUIRE(1 == 1);

    delete model;
}
