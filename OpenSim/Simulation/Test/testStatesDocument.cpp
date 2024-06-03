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
using std::cout;
using std::endl;


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
/**
Compute the maximum error that can result from rounding a value at a
specified precision. This method assumes a base-10 representation of the value.
@param value Value to be rounded.
@param precision Number of significant figures that will be retained in the
value.
*/
double
computeMaxRoundingError(double value, int precision) {
    if (value == 0) return 0.0;
    int p = clamp(1, precision, SimTK::LosslessNumDigitsReal);
    double leastSigDigit = trunc(log10(fabs(value))-precision);
    double max_eps = 0.5*pow(10.0, leastSigDigit);
    if(max_eps < SimTK::LeastPositiveReal) return SimTK::LeastPositiveReal;
    return max_eps;
}

//_____________________________________________________________________________
// Test for exact equality of two state trajectories.
// If a state variable fails an equality test, it is likely that that
// variable has not been added to OpenSim's Component heirarchy and therefore
// has not been serialized.
bool
testEquality(const Model& model,
    const Array_<State>& trajA, const Array_<State>& trajB, int precision)
{
    // Check the array size
    REQUIRE(trajA.size() == trajB.size());

    // Continuous Variables
    // Continuous variables gathered efficiently without using any
    // OpenSim::Component methods by using state.getQ(), state.getU(), and
    // state.getZ().
    double tA, tB;
    const State* stateA = trajA.cbegin();
    const State* stateB = trajB.cbegin();
    // Time
    for(int iTime=0; stateA!=trajA.cend(); ++iTime, ++stateA, ++stateB) {

        // Check subsystem consistency
        // This checks that basic parameters like number of subystem, nq, nu,
        // and nz are the same for two state objects.
        REQUIRE(stateA->isConsistent(*stateB));

        // Get time
        tA = stateA->getTime();
        tB = stateB->getTime();
        double max_eps = computeMaxRoundingError(tA, precision);
        CHECK_THAT(tB, Catch::Matchers::WithinAbs(tA, max_eps));

        // Check the number of subsystesm
        int nsubA = stateA->getNumSubsystems();
        int nsubB = stateB->getNumSubsystems();
        REQUIRE(nsubA == nsubB);

        // Q
        const Vector& qA = stateA->getQ();
        const Vector& qB = stateB->getQ();
        Vector diff = qB - qA;
        int nq = qA.size();
        std::cout << "diff= " << diff << std::endl;
        for (int i = 0; i < nq; ++i) {
            max_eps = computeMaxRoundingError(qA[i], precision);
            CHECK_THAT(qB[i], Catch::Matchers::WithinAbs(qA[i], max_eps));
        }
        // U
        const Vector& uA = stateA->getU();
        const Vector& uB = stateB->getU();
        int nu = uA.size();
        for (int i = 0; i < nu; ++i) {
            max_eps = computeMaxRoundingError(uA[i], precision);
            CHECK_THAT(uB[i], Catch::Matchers::WithinAbs(uA[i], max_eps));
        }
        // Z
        const Vector& zA = stateA->getZ();
        const Vector& zB = stateB->getZ();
        int nz = zA.size();
        for (int i = 0; i < nz; ++i) {
            max_eps = computeMaxRoundingError(zA[i], precision);
            CHECK_THAT(zB[i], Catch::Matchers::WithinAbs(zA[i], max_eps));
        }
    }

    // Discrete Variables
    // The SimTK API does not allow an exhaustive, low-level comparison
    // on the SimTK side.
    // The comparision is done only for the discrete variables registered
    // in the OpenSim Component heirarchy. Any discrete variable that is
    // not registered in OpenSim will not be serialized, deserialized, or
    // compared in this unit test.


    // Modeling Options
    // The SimTK API does not allow an exhaustive, low-level comparison
    // across all subsystems.
    // The comparision is done only for the modeling options registered
    // in the OpenSim Component heirarchy.
    // Note that on the SimTK side, modeling options are just a special
    // kind of discrete variable.

    return(true);
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
    Vec3 insertion(0.1, 0.1, 0.025);
    PointToPointSpring* spring = new PointToPointSpring(ground, origin,
        *block, insertion, kp, restlength);
    model->addForce(spring);

    return model;
}

//_____________________________________________________________________________
// Simulate
SimTK::Array_<SimTK::State>
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

    // Return a copy of the state trajectory
    return reporter->getStates().getUnderlyingStateArray();
}

} // End anonymous namespace


TEST_CASE("Getting Started")
{
    double x = 2.0;
    double square = customSquare(x);
    REQUIRE(square == x*x);
}


TEST_CASE("Serialization and Deserialization")
{
    // Build the model and run a simulation
    // The output of simulate() is the state trajectory.
    // Note that a copy of the state trajectory is returned, so we don't have
    // to worry about the reporter (or any other object) going out of scope
    // or being deleted.
    Model *model = buildModel();
    Array_<State>& trajA = simulate(model);

    // Serialize (A)
    int precision = 6;
    SimTK::String filename = "BlockOnAString.ostates";
    StatesDocument docA(*model, trajA, precision);
    docA.serialize(filename);

    // Deserialize (B)
    StatesDocument docB(filename);
    Array_<State> trajB;
    docB.deserialize(*model, trajB);

    // Does A == B?
    testEquality(*model, trajA, trajB, precision);

    REQUIRE(1 == 1);

    delete model;
}
