/* -------------------------------------------------------------------------- *
 *                        OpenSim:  testManager.cpp                           *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Carmichael Ong                                                  *
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

/*=============================================================================

Manager Tests:
1. Calculate the location, velocity, and acceleration of a Station with the same
Manager many times. Previously, this would fail as repeated callls of 
TimeStepper::initialize() would trigger cache validation improperly.

//=============================================================================*/
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/SimbodyEngine/PinJoint.h>
#include <OpenSim/Simulation/SimbodyEngine/FreeJoint.h>
#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>
#include <OpenSim/Simulation/Manager/Manager.h>
#include <OpenSim/Common/LoadOpenSimLibrary.h>

using namespace OpenSim;
using namespace std;
void testStationCalcWithManager();
void testExcitationUpdatesWithManager();
void testCacheWithManager();
void testActivationUpdatesWithManager();

int main()
{
    SimTK::Array_<std::string> failures;

    try { testStationCalcWithManager(); }
    catch (const std::exception& e) {
        cout << e.what() << endl;
        failures.push_back("testStationCalcWithManager");
    }

    try { testExcitationUpdatesWithManager(); }
    catch (const std::exception& e) {
        cout << e.what() << endl;
        failures.push_back("testExcitationUpdatesWithManager");
    }

    try { testActivationUpdatesWithManager(); }
    catch (const std::exception& e) {
        cout << e.what() << endl;
        failures.push_back("testActivationUpdatesWithManager");
    }

    try { testCacheWithManager(); }
    catch (const std::exception& e) {
        cout << e.what() << endl;
        failures.push_back("testCacheWithManager");
    }

    if (!failures.empty()) {
        cout << "Done, with failure(s): " << failures << endl;
        return 1;
    }

    cout << "Done. All cases passed." << endl;

    return 0;
}

//==============================================================================
// Test Cases
//==============================================================================

void testStationCalcWithManager()
{
    using SimTK::Vec3;

    cout << "Running testStationCalcWithManager" << endl;

    Model pendulum;
    pendulum.setName("pendulum");

    auto rod = new Body("rod", 0.54321, SimTK::Vec3(0.1, 0.5, 0.2),
        SimTK::Inertia::cylinderAlongY(0.025, 0.55));
    pendulum.addBody(rod);

    auto pin = new PinJoint("pin", pendulum.getGround(), Vec3(0), Vec3(0), 
        *rod, Vec3(0), Vec3(0));
    pendulum.addJoint(pin);

    // Create station in the extra frame
    Station* myStation = new Station();
    const SimTK::Vec3 point(0.5, 1, -1.5);
    myStation->set_location(point);
    myStation->setParentFrame(*rod);
    pendulum.addModelComponent(myStation);

    // Initialize the system
    SimTK::State state = pendulum.initSystem();

    // set the model coordinates and coordinate speeds
    pin->getCoordinate(PinJoint::Coord::RotationZ).setValue(state, 0.29);
    pin->getCoordinate(PinJoint::Coord::RotationZ).setSpeedValue(state, 0.1);

    // Get the frame's mobilized body
    const OpenSim::PhysicalFrame&  frame = myStation->getParentFrame();
    SimTK::MobilizedBody mb = frame.getMobilizedBody();

    // Do a simulation
    SimTK::RungeKuttaMersonIntegrator integrator(pendulum.getSystem());

    double finalT = 1.0;
    double dt = 0.01;
    int n = int(round(finalT / dt));

    // Hold the computed kinematics from OpenSim and Simbody
    SimTK::Vec3 lo, vo, ao, l, v, a;

    Manager manager(pendulum, integrator);
    manager.setPerformAnalyses(false);
    manager.setWriteToStorage(false);
    state.setTime(0.0);

    for (int i = 1; i <= n; ++i) {
        // Reuse the same Manager to integrate a state forward repeatedly.
        // This would previously cause issues with cache validation.
        manager.integrate(state, i*dt);

        // realize to acceleration to access acceleration stage cache
        pendulum.realizeAcceleration(state);

        // Use Simbody to get the location, velocity & acceleration in ground.
        mb.findStationLocationVelocityAndAccelerationInGround(state,
            point, l, v, a);
        lo = myStation->getLocationInGround(state);
        vo = myStation->getVelocityInGround(state);
        ao = myStation->getAccelerationInGround(state);

        cout << "t = " << state.getTime() << ": os_a = " << ao;
        cout << " | sb_a = " << a << endl;

        cout << "t = " << state.getTime() << ": os_l = " << lo;
        cout << " | sb_l = " << l << endl;

        // Compare Simbody values to values from Station
        SimTK_TEST_EQ(l, lo);
        SimTK_TEST_EQ(v, vo);
        SimTK_TEST_EQ(a, ao);
    }
}

void testCacheWithManager()
{
    using SimTK::Vec3;

    cout << "Running testCacheWithManager" << endl;

    Model model;
    model.setName("block");
    double g = 9.81;
    model.setGravity(Vec3(0, -g, 0));

    auto block = new Body("block", 0.54321, SimTK::Vec3(0, 0, 0),
        SimTK::Inertia(3.14));
    model.addBody(block);

    auto joint = new FreeJoint("joint", model.getGround(), Vec3(0), Vec3(0),
        *block, Vec3(0), Vec3(0));
    model.addJoint(joint);

    // Create station in the extra frame
    Station* myStation = new Station();
    const SimTK::Vec3 point(0, 0, 0);
    myStation->set_location(point);
    myStation->setParentFrame(*block);
    model.addModelComponent(myStation);

    // Initialize the system
    SimTK::State state = model.initSystem();

    // Get the frame's mobilized body
    const OpenSim::PhysicalFrame& frame = myStation->getParentFrame();
    SimTK::MobilizedBody mb = frame.getMobilizedBody();

    // Hold the computed kinematics from OpenSim and Simbody
    SimTK::Vec3 lo, vo, ao, l, v, a;

    Manager manager(model);
    manager.setPerformAnalyses(false);
    manager.setWriteToStorage(false);
    state.setTime(0.0);

    double finalT = 1.0;
    double dt = 0.01;
    int n = int(round(finalT / dt));

    for (int i = 1; i <= n; ++i) {
        const Coordinate& coordY = joint->getCoordinate(FreeJoint::Coord::TranslationY);
        double initialHeight = 3.14*(i-1);
        coordY.setValue(state, initialHeight);
        coordY.setSpeedValue(state, 0.0);
        double expectedHeightFall = 0.5*g*pow(dt, 2);
        double expectedHeightAfterIntegration = initialHeight - expectedHeightFall;

        // Reuse the same Manager to integrate a state forward repeatedly.
        // This would previously cause issues with cache validation.
        manager.integrate(state, i*dt);

        // realize to acceleration to access acceleration stage cache
        model.realizeAcceleration(state);

        // Use Simbody to get the location, velocity & acceleration in ground.
        Vec3 l = mb.findStationLocationInGround(state, point);
        lo = myStation->getLocationInGround(state);

        cout << "t = " << state.getTime() << ": os_l = " << lo;
        cout << " | sb_l = " << l << endl;

        // Compare Simbody values to values from Station
        SimTK_TEST_EQ(l[1], expectedHeightAfterIntegration);
        SimTK_TEST_EQ(lo[1], expectedHeightAfterIntegration);
    }
}

void testExcitationUpdatesWithManager()
{
    cout << "Running testExcitationUpdatesWithManager" << endl;

    LoadOpenSimLibrary("osimActuators");
    Model arm("arm26.osim");
    arm.setUseVisualizer(true);
    SimTK::State& state = arm.initSystem();
    Manager manager(arm);
    const Set<Muscle> &muscleSet = arm.getMuscles();
    double stepsize = 0.01;

    for (int i = 0; i < 10; ++i)
    {
        state.updQ() = 1;
        for (int j = 0; j < muscleSet.getSize(); ++j)
        {
            muscleSet.get(j).setExcitation(state, 1.0);
        }
        cout << state.getTime() << " ";
        manager.integrate(state, stepsize*(i + 1));
        arm.realizeDynamics(state);
        cout << muscleSet.get(0).getActivation(state) << " " << state.getQ() << endl;
    }
}

void testActivationUpdatesWithManager()
{
    cout << "Running testActivationUpdatesWithManager" << endl;

    LoadOpenSimLibrary("osimActuators");
    Model arm("arm26.osim");
    arm.setUseVisualizer(true);
    SimTK::State& state = arm.initSystem();
    Manager manager(arm);
    const Set<Muscle> &muscleSet = arm.getMuscles();
    double stepsize = 0.01;

    for (int i = 0; i < 10; ++i)
    {
        state.updQ() = 1;
        for (int j = 0; j < muscleSet.getSize(); ++j)
        {
            muscleSet.get(j).setActivation(state, 1.0);
        }
        cout << state.getTime() << " ";
        manager.integrate(state, stepsize*(i + 1));
        arm.realizeDynamics(state);
        cout << muscleSet.get(0).getActivation(state) << " " << state.getQ() << endl;
    }
}