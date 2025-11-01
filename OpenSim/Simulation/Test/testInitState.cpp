/* -------------------------------------------------------------------------- *
 *                        OpenSim:  testInitState.cpp                         *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Peter Eastman, Ajay Seth                                        *
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
#include <stdint.h>
#include <OpenSim/Simulation/Manager/Manager.h>
#include <OpenSim/Simulation/Control/ControlSetController.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Actuators/PointActuator.h>
#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>
#include <catch2/catch_all.hpp>

using namespace OpenSim;
using namespace std;

//==============================================================================
// testInitState tests that a Model consistently generates the same default 
// state from its initSystem() method. It also tests that when the properties
// are updated (after a simulation) that the defaults match the values in the 
// new state.
//==============================================================================

TEST_CASE("testStates")
{
    using namespace SimTK;

    // The following model(s) contains Actuators that are registered when the
    // osimActuators library is loaded. But unless we call at least one
    // function defined in the osimActuators library, some linkers will omit
    // its dependency from the executable and it will not be loaded at
    // startup.
    { PointActuator t; }

    //==========================================================================
    // Setup OpenSim model
    std::string modelFile = "arm26.osim";
    Model model(modelFile);
    ControlSetController* controller = new ControlSetController();
    controller->setControlSetFileName("arm26_StaticOptimization_controls.xml");

    model.addController( controller );
    // original default state
    State& state = model.initSystem();

    // hold on to original default continuous state variables
    Vector y1 = state.getY();
    y1 = state.getY();
    //y1.dump("y1: Initial state:");

    // update state to contain muscle states that yield muscle equilibrium
    model.equilibrateMuscles(state);
    state.getY().dump("y1: State after equilibrateMuscles:");
    //==========================================================================
    // Compute the force and torque at the specified times.
    Manager manager(model);
    state.setTime(0.0);
    manager.initialize(state);

    // update state after a short simulation forward in time
    manager.integrate(0.05);

    // continuous state variables after simulation
    Vector y2 = state.getY();
    //y2.dump("y2: State after integration:");

    // reset model working state to default state
    State& state2 = model.initializeState();

    // another version of default continuous state variables 
    // should be unaffected by simulation of the system
    Vector y3 = state2.getY();
    //y3.dump("y3: Model reset to Initial state:");

    // update state to contain muscle states that yield muscle equilibrium
    model.equilibrateMuscles(state2);
    state.getY().dump("y3: State after equilibrateMuscles:");
    //==========================================================================
    // Compute the force and torque at the specified times.
    Manager manager2(model);
    state2.setTime(0.0);
    manager2.initialize(state2);

    // update state after a short simulation forward in time
    manager2.integrate(0.05);

    // get the default continuous state variables updated
    // from the state after the simulation
    Vector y4 = state2.getY();
    
    //y4.dump("y4: Default State after second simulation:");

    for (int i = 0; i < y1.size(); i++) 
    {
        cout << i <<" : y1[i] = " << y1[i] << " :: y3[i] = " << y3[i] << endl;
        ASSERT_EQUAL(y1[i], y3[i], 1e-5,__FILE__, __LINE__, 
            "Model failed to maintain default state after simulation.");
        cout << i <<" : y2[i] = " << y2[i] << " :: y4[i] = " << y4[i] << endl;
        ASSERT_EQUAL(y2[i], y4[i], 1e-5,__FILE__, __LINE__, 
            "Model failed to properly update default state after simulation.");
    }
    ASSERT(max(abs(y1-y2)) > 1e-4);
}

