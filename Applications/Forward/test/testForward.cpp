/* -------------------------------------------------------------------------- *
 *                         OpenSim:  testForward.cpp                          *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2025 Stanford University and the Authors                *
 * Author(s): Frank C. Anderson, Ajay Seth, Nicholas Bianco                   *
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

#include <OpenSim/Simulation/Control/Controller.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Manager/Manager.h>
#include <OpenSim/Tools/ForwardTool.h>
#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>
#include <OpenSim/Actuators/ModelFactory.h>

#include <catch2/catch_all.hpp>

using namespace OpenSim;
using namespace std;

TEST_CASE("testPendulum") {
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

TEST_CASE("testPendulumExternalLoad") {
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
        message << "t=" << time <<" state# "<< j << " "
            << standard.getColumnLabels()[j+1] << " std=" << data[j]
            <<"  computed=" << state->getData()[j];
        ASSERT_EQUAL(data[j], state->getData()[j], 1e-2,
            __FILE__, __LINE__, "ASSERT_EQUAL FAILED " + message.str());
        cout << "ASSERT_EQUAL PASSED " << message.str() << endl;
    }
}

TEST_CASE("testPendulumExternalLoadWithPointInGround") {
    ForwardTool forward("pendulum_ext_point_in_ground_Setup_Forward.xml");
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
        message << "t=" << time <<" state# "<< j << " " << standard.getColumnLabels()[j+1]
            << " std=" << data[j] <<"  computed=" << state->getData()[j];
        cout << message.str() << endl;
        ASSERT_EQUAL(data[j], state->getData()[j], 1e-2,
            __FILE__, __LINE__, "ASSERT_EQUAL FAILED " + message.str());
        cout << "ASSERT_EQUAL PASSED " << endl;
    }
}

TEST_CASE("testArm26") {
    Object::renameType("Thelen2003Muscle", "Thelen2003Muscle_Deprecated");

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
        ASSERT_EQUAL(data[j], state->getData()[j], 5.0e-3,
            __FILE__, __LINE__, "ASSERT_EQUAL FAILED " + message.str());
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
        ASSERT_EQUAL(data[j], state->getData()[j], 5.0e-3,
            __FILE__, __LINE__, "ASSERT_EQUAL FAILED " + message.str());
        cout << "ASSERT_EQUAL PASSED " << message.str();
    }
}

TEST_CASE("testGait2354") {
    Object::renameType("Thelen2003Muscle", "Thelen2003Muscle_Deprecated");

    ForwardTool forward("subject01_Setup_Forward.xml");
    forward.run();
    Storage results("Results/subject01_states.sto");

    //Storage standard("std_subject01_walk1_states.sto");
    Storage* standard = new Storage();
    string statesFileName("std_subject01_walk1_states.sto");
    forward.loadStatesStorage( statesFileName, standard );

    int nstates = forward.getModel().getNumStateVariables();
    int nq = forward.getModel().getNumCoordinates();
    std::vector<double> rms_tols(2*nstates, 0.001); //activations and fiber-lengths

    for(int i=0; i<nq; ++i){
        rms_tols[2*i] = 0.035; // coordinates at less than 2degrees
        rms_tols[2*i+1] = 2.5; // speeds can deviate by a lot due to open-loop test
    }

    CHECK_STORAGE_AGAINST_STANDARD(results, *standard, rms_tols,
        __FILE__, __LINE__, "testGait2354 failed");
}

TEST_CASE("testGait2354WithController") {
    Object::renameType("Thelen2003Muscle", "Thelen2003Muscle_Deprecated");

    ForwardTool forward("subject01_Setup_Forward_Controller.xml");
    forward.run();
    Storage results("ResultsCorrectionController/subject01_states.sto");
    //Storage standard("std_subject01_walk1_states.sto");
    Storage* standard = new Storage();
    string statesFileName("std_subject01_walk1_states.sto");
    forward.loadStatesStorage( statesFileName, standard );

    int nstates = forward.getModel().getNumStateVariables();
    int nq = forward.getModel().getNumCoordinates();
    std::vector<double> rms_tols(2*nstates, 0.001); //activations and fiber-lengths

    for(int i=0; i<nq; ++i){
        rms_tols[2*i] = 0.01; // coordinates at less than 0.6 degree
        rms_tols[2*i+1] = 0.1; // speeds should deviate less with feedback controller
    }

    CHECK_STORAGE_AGAINST_STANDARD(results, *standard, rms_tols,
        __FILE__, __LINE__, "testGait2354WithController failed");
}

TEST_CASE("testGait2354WithControllerGUI") {
    Object::renameType("Thelen2003Muscle", "Thelen2003Muscle_Deprecated");

    // The following lines are the steps from ForwardToolModel.java that
    // associates the a previous (orgiModel) model with the ForwardTool
    // instead of the Tool loading the model specified in the setup
    ForwardTool forward("subject01_Setup_Forward_Controller.xml");
    Model origModel(forward.getModelFilename());

    Model* model = new Model(origModel);
    model->initSystem();

    const std::string resultsDir{ "ResultsCorrectionControllerGUI" };

    forward.setResultsDir(resultsDir);
    forward.updateModelForces(*model, "");
    forward.setModel(*model);

    forward.run();

    // For good measure we'll make sure we still get the identical results
    Storage results(resultsDir+"/subject01_states.sto");
    //Storage standard("std_subject01_walk1_states.sto");
    Storage standard("ResultsCorrectionController/subject01_states.sto");

    int nstates = forward.getModel().getNumStateVariables();
    int nq = forward.getModel().getNumCoordinates();
    std::vector<double> rms_tols(2 * nstates, SimTK::SqrtEps);

    CHECK_STORAGE_AGAINST_STANDARD(results, standard, rms_tols,
        __FILE__, __LINE__, "testGait2354WithControllerGUI failed");

    delete model;
}

TEST_CASE("testForwardToolVersusManager") {
    Model model = ModelFactory::createNLinkPendulum(3);
    SimTK::State state = model.initSystem();

    Manager manager(model);
    manager.setIntegratorMinimumStepSize(1e-8);
    manager.setIntegratorMaximumStepSize(10.0);
    manager.setIntegratorAccuracy(1e-4);
    manager.initialize(state);
    manager.integrate(1.0);
    manager.getStateStorage().print(
            "testForwardToolVersusManager_manager_states.sto");

    ForwardTool forward;
    forward.setModel(model);
    forward.setName("testForwardToolVersusManager_forward");
    forward.setMinDT(1e-8);
    forward.setMaxDT(10.0);
    forward.setErrorTolerance(1e-4);
    forward.run();

    Storage managerStates("testForwardToolVersusManager_manager_states.sto");
    Storage forwardStates("testForwardToolVersusManager_forward_states.sto");
    CHECK(managerStates.getSize() == forwardStates.getSize());

    std::vector<double> rms_tols(state.getNY(), SimTK::SqrtEps);
    CHECK_STORAGE_AGAINST_STANDARD(forwardStates, managerStates, rms_tols,
        __FILE__, __LINE__, "testForwardToolVersusManager failed");

}
