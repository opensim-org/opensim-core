/* -------------------------------------------------------------------------- *
 *                         OpenSim:  testForward.cpp                          *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Frank C. Anderson, Ajay Seth                                    *
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
// forward.cpp
// author:  Frank C. Anderson, Ajay Seth

// INCLUDE
#include <OpenSim/Simulation/Control/Controller.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Tools/ForwardTool.h>
#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>
#include "SimTKmath.h"

using namespace OpenSim;
using namespace std;

void testPendulum();	// test manager/integration process
void testPendulumExternalLoad(); // test application of external loads point in pendulum
void testPendulumExternalLoadWithPointInGround(); // test application of external loads point in ground
void testArm26();		// now add computation of controls and generation of muscle forces
void testGait2354();    // controlled muscles and ground reactions forces
void testGait2354WithController(); // included additional controller

int main() {
    Object::renameType("Thelen2003Muscle", "Thelen2003Muscle_Deprecated");

    SimTK::Array_<std::string> failures;

    // test manager/integration process
    try {
        testPendulum();
        cout << "\nPendulum test PASSED " << endl;
    }
    catch (const std::exception& e)
    {
        cout << e.what() <<endl;
        failures.push_back("testPendulum");
    }

    // test application of external loads
    try {
        testPendulumExternalLoad();
        cout << "\nPendulum with external load test PASSED " << endl;
    }
    catch (const std::exception& e)
    {
        cout << e.what() <<endl;
        failures.push_back("testPendulumExternalLoad");
    }

    // test application of external loads
    try {
        testPendulumExternalLoadWithPointInGround();
        cout << "\nPendulum with external load and point in ground PASSED " << endl;
    }
    catch (const std::exception& e)
    {
        cout << e.what() <<endl;
        failures.push_back("testPendulumExternalLoadWithPointInGround");
    }

    // now add computation of controls and generation of muscle forces
    try {
        testArm26();
        cout << "\narm26 test PASSED " << endl;
    }
    catch (const std::exception& e)
    {
        cout << e.what() <<endl;
        failures.push_back("testArm26");
    }

    // include applied ground reactions forces
    try {
        testGait2354();
        cout << "\ngait2354 test PASSED " << endl;
    }
    catch (const std::exception& e)
    {
        cout << e.what() <<endl;
        failures.push_back("testGait2354");
    }

    // finally include a controller
    try {
        testGait2354WithController();
        cout << "\ngait2354 with correction controller test PASSED " << endl;
    }
    catch (const std::exception& e)
    {
        cout << e.what() <<endl;
        failures.push_back("testGait2354WithController");
    }

    if (!failures.empty()) {
        cout << "Done, with failure(s): " << failures << endl;
        return 1;
    }

    cout << "Done" << endl;
    return 0;
}

void testPendulum() {
    ForwardTool forward("pendulum_Setup_Forward.xml");
    forward.run();
    Storage storage("Results/pendulum_states.sto");
    ASSERT(storage.getFirstTime() == 0.0);
    ASSERT(storage.getLastTime() == 1.0);

    // Since the pendulum is only swinging through small angles, it should be very
    // close to a simple harmonic oscillator.
    double previousTime = -1.0;
    double amp = SimTK::Pi/20;
    double k = sqrt(9.80665000/0.5);
    for (int j = 0; j < storage.getSize(); ++j) {
        StateVector* state = storage.getStateVector(j);
        double time = state->getTime();
        ASSERT(time > previousTime);
        previousTime = time;
        ASSERT_EQUAL(-amp*cos(k*time), state->getData()[0], 1.0e-2);
        ASSERT_EQUAL(amp*k*sin(k*time), state->getData()[1],1.0e-2);
    }
    ASSERT(previousTime == 1.0);
}


void testPendulumExternalLoad() {
    ForwardTool forward("pendulum_ext_gravity_Setup_Forward.xml");
    forward.run();
    Storage results("Results/pendulum_ext_gravity_states.sto");
    ASSERT(results.getFirstTime() == 0.0);
    ASSERT(results.getLastTime() == 1.0);

    Storage standard("Results/pendulum_states.sto");


    Array<double> data;
    int i = results.getSize() - 1;
    StateVector* state = results.getStateVector(i);
    double time = state->getTime();
    data.setSize(state->getSize());
    standard.getDataAtTime(time, state->getSize(), data);
    int nc = forward.getModel().getNumCoordinates();
    for (int j = 0; j < nc; ++j) {
        stringstream message;
        message << "t=" << time <<" state# "<< j << " " << standard.getColumnLabels()[j+1] << " std=" << data[j] <<"  computed=" << state->getData()[j];
        ASSERT_EQUAL(data[j], state->getData()[j], 1e-2, __FILE__, __LINE__, "ASSERT_EQUAL FAILED " + message.str());
        cout << "ASSERT_EQUAL PASSED " << message.str() << endl;
    }
}


void testPendulumExternalLoadWithPointInGround() {
    ForwardTool forward("pendulum_ext_point_in_ground_Setup_Forward.xml");
    cout << endl;
    forward.run();
    Storage results("Results/pendulum_ext_gravity_point_in_ground_states.sto");
    ASSERT(results.getFirstTime() == 0.0);
    ASSERT(results.getLastTime() == 1.0);

    Storage standard("Results/pendulum_states.sto");


    Array<double> data;
    int i = results.getSize() - 1;
    StateVector* state = results.getStateVector(i);
    double time = state->getTime();
    data.setSize(state->getSize());
    standard.getDataAtTime(time, state->getSize(), data);
    int nc = forward.getModel().getNumCoordinates();
    for (int j = 0; j < nc; ++j) {
        stringstream message;
        message << "t=" << time <<" state# "<< j << " " << standard.getColumnLabels()[j+1] << " std=" << data[j] <<"  computed=" << state->getData()[j];
        ASSERT_EQUAL(data[j], state->getData()[j], 1e-2, __FILE__, __LINE__, "ASSERT_EQUAL FAILED " + message.str());
        cout << "ASSERT_EQUAL PASSED " << message.str() << endl;
    }
}


void testArm26() {
    ForwardTool forward("arm26_Setup_Forward.xml");
    forward.run();

    Storage results("Results/arm26_states.sto");
    Storage* standard = new Storage();
    string statesFileName("std_arm26_states.sto");
    forward.loadStatesStorage( statesFileName, standard );
    StateVector* state = results.getStateVector(0);
    double time = state->getTime();
    Array<double> data;
    data.setSize(state->getSize());
    standard->getDataAtTime(time, state->getSize(), data);
    for (int j = 0; j < state->getSize(); ++j) {
        stringstream message;
        message << "t=" << time <<" state# "<< j << " " << standard->getColumnLabels()[j+1] << " std=" << data[j] <<"  computed=" << state->getData()[j] << endl;
        ASSERT_EQUAL(data[j], state->getData()[j], 1.0e-3, __FILE__, __LINE__, "ASSERT_EQUAL FAILED " + message.str());
        cout << "ASSERT_EQUAL PASSED " << message.str();
    }

    int i = results.getSize()-1;
    state = results.getStateVector(i);
    time = state->getTime();
    data.setSize(state->getSize());
    standard->getDataAtTime(time, state->getSize(), data);
    for (int j = 0; j < state->getSize(); ++j) {
        stringstream message;
        message << "t=" << time <<" state# "<< j << " " << standard->getColumnLabels()[j+1] << " std=" << data[j] <<"  computed=" << state->getData()[j] << endl;
        ASSERT_EQUAL(data[j], state->getData()[j], 1.0e-3, __FILE__, __LINE__, "ASSERT_EQUAL FAILED " + message.str());
        cout << "ASSERT_EQUAL PASSED " << message.str();
    }
}

void testGait2354()
{
    ForwardTool forward("subject01_Setup_Forward.xml");
    forward.run();
    Storage results("Results/subject01_states.sto");

    //Storage standard("std_subject01_walk1_states.sto");
    Storage* standard = new Storage();
    string statesFileName("std_subject01_walk1_states.sto");
    forward.loadStatesStorage( statesFileName, standard );

    int nstates = forward.getModel().getNumStateVariables();
    int nq = forward.getModel().getNumCoordinates();
    Array<double> rms_tols(0.001, 2*nstates); //activations and fiber-lengths

    for(int i=0; i<nq; ++i) {
        rms_tols[2*i] = 0.035; // coordinates at less than 2degrees
        rms_tols[2*i+1] = 2.5; // speeds can deviate by a lot due to open-loop test
    }

    CHECK_STORAGE_AGAINST_STANDARD(results, *standard, rms_tols, __FILE__, __LINE__, "testGait2354 failed");
}

void testGait2354WithController() {
    ForwardTool forward("subject01_Setup_Forward_Controller.xml");
    forward.run();
    Storage results("ResultsCorrectionController/subject01_states.sto");
    //Storage standard("std_subject01_walk1_states.sto");
    Storage* standard = new Storage();
    string statesFileName("std_subject01_walk1_states.sto");
    forward.loadStatesStorage( statesFileName, standard );

    int nstates = forward.getModel().getNumStateVariables();
    int nq = forward.getModel().getNumCoordinates();
    Array<double> rms_tols(0.001, 2*nstates); //activations and fiber-lengths

    for(int i=0; i<nq; ++i) {
        rms_tols[2*i] = 0.01; // coordinates at less than 0.6 degree
        rms_tols[2*i+1] = 0.1; // speeds should deviate less with feedback controller
    }

    CHECK_STORAGE_AGAINST_STANDARD(results, *standard, rms_tols, __FILE__, __LINE__, "testGait2354WithController failed");
}
