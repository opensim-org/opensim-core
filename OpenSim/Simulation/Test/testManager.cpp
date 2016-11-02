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
 * Author(s): Ajay Seth, James Dunne                                          *
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
1. Station on OffsetFrame that is integrated with the same Manager many times.
This is a copy of the test from testPoints that does the same with a TimeStepper
except with a Manager. Previously, this would fail as TimeStepper::initialize()
would trigger cache validation improperly.

//=============================================================================*/
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/SimbodyEngine/EllipsoidJoint.h>
#include <OpenSim/Simulation/SimbodyEngine/GimbalJoint.h>
#include <OpenSim/Simulation/Model/PhysicalOffsetFrame.h>
#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>
#include <OpenSim/Simulation/Manager/Manager.h>

using namespace OpenSim;
using namespace std;
void testStationOnOffsetFrame();

int main()
{
    SimTK::Array_<std::string> failures;

    try { testStationOnOffsetFrame(); }
    catch (const std::exception& e) {
        cout << e.what() << endl;
        failures.push_back("testStationOnOffsetFrame");
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

void testStationOnOffsetFrame()
{
    using SimTK::Vec3;
    SimTK::Vec3 tolerance(SimTK::Eps);
    SimTK::MultibodySystem system;
    SimTK::SimbodyMatterSubsystem matter(system);
    SimTK::GeneralForceSubsystem forces(system);

    cout << "Running testStationOnOffsetFrame" << endl;

    Model pendulum;
    pendulum.setName("pendulum3D");

    auto rod1 = new Body("rod1", 0.54321, SimTK::Vec3(0.1, 0.5, 0.2),
        SimTK::Inertia::cylinderAlongY(0.025, 0.55));
    rod1->attachGeometry(new Cylinder(0.025, 0.55));

    auto rod2 = rod1->clone();
    rod2->setName("rod2");

    pendulum.addBody(rod1);
    pendulum.addBody(rod2);

    auto hip = new GimbalJoint("hip", pendulum.getGround(), Vec3(0), Vec3(1, 2, 3),
                        *rod1, Vec3(0, 0.25, 0), Vec3(0.9, 0.5, 0.2));

    auto knee = new EllipsoidJoint("knee", *rod1, Vec3(0, -0.25, 0), Vec3(0.2, 3.3, 0.7),
        *rod2, Vec3(0, 0.25, 0), Vec3(0.2, 0.5, 0.1), Vec3(0.03, 0.04, 0.05));

    pendulum.addJoint(hip);
    pendulum.addJoint(knee);

    // Define and add a frame to the rod2 body
    SimTK::Transform X_RO;
    X_RO.setP(SimTK::Vec3(1.234, -0.2667, 0));
    X_RO.updR().setRotationFromAngleAboutAxis(SimTK::Pi/3.33 , SimTK::ZAxis);
    PhysicalOffsetFrame* offsetFrame = new PhysicalOffsetFrame(*rod2, X_RO);
    offsetFrame->setName("myExtraFrame");
    pendulum.addFrame(offsetFrame);

    // Create station in the extra frame
    Station* myStation = new Station();
    const SimTK::Vec3 point(0.5, 1, -1.5);
    myStation->set_location(point);
    myStation->setParentFrame(*offsetFrame);
    pendulum.addModelComponent(myStation);

    // optionally turn on visualizer to help debug
    //pendulum.setUseVisualizer(true);

    // Initialize the system
    SimTK::State state = pendulum.initSystem();

    // set the model coordinates and coordinate speeds
    hip->getCoordinate(GimbalJoint::Coord::Rotation1X).setValue(state, 0.29);
    hip->getCoordinate(GimbalJoint::Coord::Rotation1X).setSpeedValue(state, 0.1);
    hip->getCoordinate(GimbalJoint::Coord::Rotation2Y).setValue(state, -0.38);
    hip->getCoordinate(GimbalJoint::Coord::Rotation2Y).setSpeedValue(state, -0.13);

    // Get the frame's mobilized body
    const OpenSim::PhysicalFrame&  frame = myStation->getParentFrame();
    SimTK::MobilizedBody  mb = frame.getMobilizedBody();

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
        manager.setInitialTime((i - 1)*dt);
        manager.setFinalTime(i*dt);
        manager.integrate(state);

        // realize to acceleration to access acceleration stage cache
        pendulum.realizeAcceleration(state);

        // Use Simbody to get the location, velocity & acceleration in ground.
        // Need to map the point into the base frame to use MobilizedBody's
        // station methods otherwise we exclude the effect of the offset frame
        SimTK::Vec3 pointInBase = frame.findTransformInBaseFrame()*point;
        mb.findStationLocationVelocityAndAccelerationInGround(state,
            pointInBase, l, v, a);

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
