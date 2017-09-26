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
1. testStationCalcWithManager: Calculate the location, velocity, and
   acceleration of a Station with the same Manager many times. Previously, this
   would fail as repeated calls of TimeStepper::initialize() would trigger cache
   validation improperly.
2. testStateChangesBetweenIntegration: Change the initial value and speed of a
   falling ball between integrating using the same State. This ensures that
   integrating with the same State with different Managers triggers the cache
   updates correctly.
3. testExcitationUpdatesWithManager: Update the excitation of a muscle in the
   arm26 model between subsequent integrations.

//=============================================================================*/
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/SimbodyEngine/PinJoint.h>
#include <OpenSim/Simulation/SimbodyEngine/FreeJoint.h>
#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>
#include <OpenSim/Simulation/Manager/Manager.h>
#include <OpenSim/Common/LoadOpenSimLibrary.h>
#include <OpenSim/Simulation/Control/PrescribedController.h>
#include <OpenSim/Common/Constant.h>

using namespace OpenSim;
using namespace std;
void testStationCalcWithManager();
void testStateChangesBetweenIntegration();
void testExcitationUpdatesWithManager();

int main()
{
    SimTK::Array_<std::string> failures;

    try { testStationCalcWithManager(); }
    catch (const std::exception& e) {
        cout << e.what() << endl;
        failures.push_back("testStationCalcWithManager");
    }

    try { testStateChangesBetweenIntegration(); }
    catch (const std::exception& e) {
        cout << e.what() << endl;
        failures.push_back("testStateChangesBetweenIntegration();");
    }

    try { testExcitationUpdatesWithManager(); }
    catch (const std::exception& e) {
        cout << e.what() << endl;
        failures.push_back("testActivationUpdatesWithManager();");
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
    manager.initialize(state);

    for (int i = 1; i <= n; ++i) {
        // Reuse the same Manager to integrate a state forward repeatedly.
        // This would previously cause issues with cache validation.
        state = manager.integrate(i*dt);

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

void testStateChangesBetweenIntegration()
{
    cout << "Running testStateChangesBetweenIntegration" << endl;

    using SimTK::Vec3;

    Model model;
    model.setName("ball");

    auto ball = new Body("ball", 0.7, Vec3(0.1),
        SimTK::Inertia::sphere(0.5));
    model.addBody(ball);

    auto freeJoint = new FreeJoint("freeJoint", model.getGround(), Vec3(0), Vec3(0),
        *ball, Vec3(0), Vec3(0));
    model.addJoint(freeJoint);

    double g = 9.81;
    model.setGravity(Vec3(0, -g, 0));

    Station* myStation = new Station();
    const SimTK::Vec3 point(0);
    myStation->set_location(point);
    myStation->setParentFrame(*ball);
    model.addModelComponent(myStation);

    SimTK::State& state = model.initSystem();

    const Coordinate& sliderCoord = 
        freeJoint->getCoordinate(FreeJoint::Coord::TranslationY);

    std::vector<double> integInitTimes = {0.0, 1.0, 3.0};
    std::vector<double> integFinalTimes = {1.0, 3.0, 6.0};
    std::vector<double> initHeights = {0.0, 13.3, 6.5};
    std::vector<double> initSpeeds = {0.0, 0.5, -0.5};
    state.setTime(integInitTimes[0]);
    size_t n = integFinalTimes.size();

    for (size_t i = 0; i < n; ++i) {
        // Set initial state for integration and check that it's correct
        sliderCoord.setValue(state, initHeights[i]);
        sliderCoord.setSpeedValue(state, initSpeeds[i]);

        Manager manager(model);
        manager.initialize(state);
        const SimTK::State& initState = manager.getState();

        SimTK_TEST_EQ(initState.getTime(), integInitTimes[i]);
        SimTK_TEST_EQ(sliderCoord.getValue(initState), initHeights[i]);
        SimTK_TEST_EQ(sliderCoord.getSpeedValue(initState), initSpeeds[i]);

        // Use Station to get the location, velocity & acceleration in ground.
        double stationHeight = myStation->getLocationInGround(initState)[1];
        double stationSpeed = myStation->getVelocityInGround(initState)[1];

        SimTK_TEST_EQ(stationHeight, initHeights[i]);
        SimTK_TEST_EQ(stationSpeed, initSpeeds[i]);

        state = manager.integrate(integFinalTimes[i]);

        model.realizeVelocity(state);

        double duration = integFinalTimes[i] - integInitTimes[i];
        double finalHeight = 
            initHeights[i] + initSpeeds[i]*duration - 0.5*g*duration*duration;
        double finalSpeed = initSpeeds[i] - g*duration;
        double sliderHeight = sliderCoord.getValue(state);
        double sliderSpeed = sliderCoord.getSpeedValue(state);
        cout << "Slider: t = " << state.getTime() << ", h = " << sliderHeight
            << ", v = " << sliderSpeed << " | Eq: t = " << integFinalTimes[i]
            << ", h = " << finalHeight << ", v = " << finalSpeed;

        SimTK_TEST_EQ(state.getTime(), integFinalTimes[i]);
        SimTK_TEST_EQ(sliderHeight, finalHeight);
        SimTK_TEST_EQ(sliderSpeed, finalSpeed);

        // Use Station to get the location, velocity & acceleration in ground.
        stationHeight = myStation->getLocationInGround(state)[1];
        stationSpeed = myStation->getVelocityInGround(state)[1];

        cout << " | Station: t = " << state.getTime() << ", h = " 
            << stationHeight << ", v = " << stationSpeed << endl;

        // Compare Station values with equation values
        SimTK_TEST_EQ(stationHeight, finalHeight);
        SimTK_TEST_EQ(stationSpeed, finalSpeed);
    }

}

void testExcitationUpdatesWithManager()
{
    cout << "Running testExcitationUpdatesWithManager" << endl;
    LoadOpenSimLibrary("osimActuators");
    Model arm("arm26.osim");

    const Set<Muscle> &muscleSet = arm.getMuscles();
    PrescribedController* controller = new PrescribedController();
    controller->addActuator(muscleSet.get(0));
    Constant* fn = new Constant(0);
    controller->prescribeControlForActuator(0, fn);
    arm.addController(controller);

    SimTK::State& state = arm.initSystem();
    state.setTime(0);
    double stepsize = 0.01;
    muscleSet.get(0).setActivation(state, 0.1);
    for (int i = 0; i < 3; ++i)
    {
        arm.realizeDynamics(state);
        double initAct = muscleSet.get(0).getActivation(state);
        double excitation = 0.2 + i*0.2;

        FunctionSet& fnset = controller->upd_ControlFunctions();
        Constant* fn = dynamic_cast<Constant*>(&fnset[0]);
        fn->setValue(excitation);

        Manager manager(arm);
        manager.initialize(state);
        state = manager.integrate(stepsize*(i + 1));

        arm.realizeDynamics(state);
        double finalAct = muscleSet.get(0).getActivation(state);
        double finalExcitation = muscleSet.get(0).getExcitation(state);
        cout << state.getTime() << " " << excitation;
        cout << " " << finalAct << endl;
        // Check if excitation is correct
        SimTK_TEST_EQ(excitation, finalExcitation);
        // If excitation is not set correctly, it will go back to the default
        // after the first integration. We can check if this is happening by
        // ensuring the activation is increasing on each integration.
        SimTK_TEST(finalAct > initAct);
    }
}