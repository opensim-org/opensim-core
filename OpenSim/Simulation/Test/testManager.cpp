/* -------------------------------------------------------------------------- *
 *                          OpenSim:  testPoints.cpp                          *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2016 Stanford University and the Authors                *
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
#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>
#include <OpenSim/Simulation/Manager/Manager.h>

using namespace OpenSim;
using namespace std;
void testStationCalcWithManager();

int main()
{
    SimTK::Array_<std::string> failures;

    try { testStationCalcWithManager(); }
    catch (const std::exception& e) {
        cout << e.what() << endl;
        failures.push_back("testStationCalcWithManager");
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

    for (int i = 1; i <= n; ++i) {
        // Reuse the same Manager to integrate a state forward repeatedly.
        // This would previously cause issues with cache validation.
        manager.setInitialTime((i-1)*dt);
        manager.setFinalTime(i*dt);
        manager.integrate(state);

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

        // Compare Simbody values to values from Station
        SimTK_TEST_EQ(l, lo);
        SimTK_TEST_EQ(v, vo);
        SimTK_TEST_EQ(a, ao);
    }
}
