// testInitState.cpp
// Author:  Peter Eastman, Ajay Seth
/*
* Copyright (c) 2012, Stanford University. All rights reserved. 
* Use of the OpenSim software in source form is permitted provided that the following
* conditions are met:
* 	1. The software is used only for non-commercial research and education. It may not
*     be used in relation to any commercial activity.
* 	2. The software is not distributed or redistributed.  Software distribution is allowed 
*     only through https://simtk.org/home/opensim.
* 	3. Use of the OpenSim software or derivatives must be acknowledged in all publications,
*      presentations, or documents describing work in which OpenSim or derivatives are used.
* 	4. Credits to developers may not be removed from executables
*     created from modifications of the source.
* 	5. Modifications of source code must retain the above copyright notice, this list of
*     conditions and the following disclaimer. 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
*  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
*  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
*  SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
*  TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
*  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR BUSINESS INTERRUPTION) OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
*  WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

//==========================================================================================================
//	testInitState tests that a Model consistently generates the same default state
//  from its initSystem() method. It also tests that when the properties
//  are updated (after a simulation) that the defaults match the values in the new state.
//==========================================================================================================
#include <OpenSim/Simulation/Manager/Manager.h>
#include <OpenSim/Simulation/Control/ControlSetController.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Common/LoadOpenSimLibrary.h>
#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>

using namespace OpenSim;
using namespace std;

void testStates();

int main()
{
	try {
		LoadOpenSimLibrary("osimActuators");
		testStates();
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

//==========================================================================================================
// Test Cases
//==========================================================================================================
void testStates()
{
	using namespace SimTK;

	//==========================================================================================================
	// Setup OpenSim model
	Model model("arm26.osim");
    ControlSetController* controller = new ControlSetController();
    controller->setControlSetFileName( "arm26_StaticOptimization_controls.xml" );
  
    model.addController( controller );
	// original default state
    State state = model.initSystem();

	// hold on to original default continuous state variables
    Vector y1 = state.getY();
	y1 = state.getY();
	y1.dump("y1: Initial state:");

	// update state to contain muscle states that yield muscle equilibirium
    model.equilibrateMuscles(state);
	state.getY().dump("y1: State after equilibrateMuscles:");
	//==========================================================================================================
	// Compute the force and torque at the specified times.

    RungeKuttaMersonIntegrator integrator(model.getMultibodySystem());
    Manager manager(model, integrator);
    manager.setInitialTime(0.0);
    manager.setFinalTime(0.05);

	// update state after a short simulation forward in time
    manager.integrate(state);

	// continuous state variables after simulation
    Vector y2 = state.getY();

	// another default state from the system
    State state2 = model.initSystem();
	// another version of default continuous state variables 
	// should be unaffected by simulation of the system
    Vector y3 = state2.getY();
	y3.dump("y3: Model reset to Initial state:");

	// update the default (properties) values from the state
	// after the simulation
    model.setPropertiesFromState(state);

	// get a new default state that should reflect the states 
	// after the simulation
    state2 = model.initSystem();
	// get the default continuous state variables updated
	// from the state after the simulation
    Vector y4 = state2.getY();
	y2.dump("y2: State after integration:");
	y4.dump("y4: Default State after update:");

    for (int i = 0; i < y1.size(); i++) 
    {
		cout << i <<" : y1[i] = " << y1[i] << " :: y3[i] = " << y3[i] << endl;
        ASSERT_EQUAL(y1[i], y3[i], 1e-5,__FILE__, __LINE__, "Model failed to maintain default state after simulation.");
		cout << i <<" : y2[i] = " << y2[i] << " :: y4[i] = " << y4[i] << endl;
        ASSERT_EQUAL(y2[i], y4[i], 1e-5,__FILE__, __LINE__, "Model failed to properly update default state after simulation.");
    }
    ASSERT(max(abs(y1-y2)) > 1e-4);
}