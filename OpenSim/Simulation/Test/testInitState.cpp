/* -------------------------------------------------------------------------- *
 *                        OpenSim:  testInitState.cpp                         *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
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
#include <OpenSim/Common/LoadOpenSimLibrary.h>
#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>

using namespace OpenSim;
using namespace std;

//==============================================================================
// testInitState tests that a Model consistently generates the same default 
// state from its initSystem() method. It also tests that when the properties
// are updated (after a simulation) that the defaults match the values in the 
// new state.
//==============================================================================
void testStates(const string& modelFile);
//==============================================================================
// testMemoryUsage tests that repeated initilization of the state does not  
// cause the memory footprint of the process to increase significantly.
//==============================================================================
void testMemoryUsage(const string& modelFile);

static const int MAX_N_TRIES = 100;

int main()
{
    try {
        LoadOpenSimLibrary("osimActuators");
        testStates("arm26.osim");
        testMemoryUsage("arm26.osim");
        testMemoryUsage("PushUpToesOnGroundWithMuscles.osim");
    }
    catch (const Exception& e) {
        cout << "testInitState failed: ";
        e.print(cout); 
        return 1;
    }
    catch (const std::exception& e) {
        cout << "testInitState failed: " << e.what() << endl;
        return 1;
    }
    cout << "Done" << endl;
    return 0;
}

//==============================================================================
// Test Cases
//==============================================================================
void testStates(const string& modelFile)
{
    using namespace SimTK;

    //==========================================================================
    // Setup OpenSim model
    Model model(modelFile);
    ControlSetController* controller = new ControlSetController();
    controller->setControlSetFileName("arm26_StaticOptimization_controls.xml");
  
    model.addController( controller );
    // original default state
    State& state = model.initSystem();

    // hold on to original default continuous state variables
    Vector y1 = state.getY();
    y1 = state.getY();
    y1.dump("y1: Initial state:");

    // update state to contain muscle states that yield muscle equilibirium
    model.equilibrateMuscles(state);
    state.getY().dump("y1: State after equilibrateMuscles:");
    //==========================================================================
    // Compute the force and torque at the specified times.

    RungeKuttaMersonIntegrator integrator(model.getMultibodySystem());
    Manager manager(model, integrator);
    manager.setInitialTime(0.0);
    manager.setFinalTime(0.05);

    // update state after a short simulation forward in time
    manager.integrate(state);

    // continuous state variables after simulation
    Vector y2 = state.getY();
    y2.dump("y2: State after integration:");

    // reset model working state to default state
    State& state2 = model.initializeState();

    // another version of default continuous state variables 
    // should be unaffected by simulation of the system
    Vector y3 = state2.getY();
    y3.dump("y3: Model reset to Initial state:");

    // update state to contain muscle states that yield muscle equilibirium
    model.equilibrateMuscles(state2);
    state.getY().dump("y3: State after equilibrateMuscles:");
    //==========================================================================
    // Compute the force and torque at the specified times.

    RungeKuttaMersonIntegrator integrator2(model.getMultibodySystem());
    Manager manager2(model, integrator);
    manager2.setInitialTime(0.0);
    manager2.setFinalTime(0.05);

    // update state after a short simulation forward in time
    manager2.integrate(state2);

    // get the default continuous state variables updated
    // from the state after the simulation
    Vector y4 = state2.getY();
    
    y4.dump("y4: Default State after second simulation:");

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

void testMemoryUsage(const string& modelFile)
{
    using namespace SimTK;

    //=========================================================================
    // Setup OpenSim model
    // base footprint
    size_t mem0 = getCurrentRSS( );
    Model model(modelFile);

    size_t model_size = getCurrentRSS( )-mem0;

    State state = model.initSystem();

    // initial footprint
    size_t mem1 = getCurrentRSS( );

    // also time how long initializing the state takes
    clock_t startTime = clock();

    //cout << "Initial memory use: " << mem1/1024 << "KB." << endl;


    for(int i=0; i< MAX_N_TRIES; ++i){
        state = model.initializeState();
    }

    // new footprint after MAX_N_TRIES
    size_t mem2 = getCurrentRSS( );
    // change
    int64_t delta = mem2-mem1;
    int64_t leak = delta/MAX_N_TRIES;
    long double leak_percent = 100.0 * leak/model_size;

    long double dT = (long double)(clock()-startTime) / CLOCKS_PER_SEC;
    long double meanT = 1.0e3 * dT/MAX_N_TRIES; // in ms
    
    cout << "*********************** testMemoryUsage ***********************" << endl;
    cout << "MODEL: "<< modelFile <<" uses "<< model_size/1024 << "KB" << endl;
    cout << delta/1024 << "KB change in memory use after " << MAX_N_TRIES
         << " state initializations." << endl;
    cout << "Approximate leak size: " << leak/1024.0 << "KB or " << 
             leak_percent << "% of model size." << endl;
    cout << "Average initialization time: " << meanT << "ms" << endl;

    // If we are leaking more than 1/2% of the model's footprint that is significant
    ASSERT( (leak_percent) < 0.5, __FILE__, __LINE__, 
        "testMemoryUsage: state initialization leak > 0.5% of model memory footprint.");

    // If we ever leak over 100MB total we should know about it.
    ASSERT( delta < 1e8, __FILE__, __LINE__, 
        "testMemoryUsage: total estimated memory leaked > 100MB.");
}
