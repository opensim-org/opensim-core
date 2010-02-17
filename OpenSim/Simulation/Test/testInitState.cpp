// testInitState.cpp
// Author:  Peter Eastman
/*
* Copyright (c) 2009, Stanford University. All rights reserved. 
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
//	testJoints builds OpenSim models using the OpenSim API and builds an equivalent
//  Simbody system using the Simbody API for each test case. A test fails if the
//  OpenSim and Simbody final states of the simulation are not equivelent (norm-err
//  less than 10x integration error tolerance)
//
//	Tests Include:
//      1. CustomJoint against Simbody built-in Pin and Universal joints
//      2. CustomJoint versus Simbody FunctionBased with spline based functions
//		3. WeldJoint versus Weld Mobilizer by welding bodies to those in test 1.
//		4. Randomized order of bodies in the BodySet (in 3.) to test connectBodies()
//		
//		TODO random branching toplogy.
//     Add tests here as new joint types are added to OpenSim
//
//==========================================================================================================
#include <iostream>
#include <OpenSim/Common/IO.h>
#include <OpenSim/Common/Exception.h>
#include <OpenSim/Simulation/Manager/Manager.h>
#include <OpenSim/Simulation/Control/ControlSetController.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Common/LoadOpenSimLibrary.h>
#include "SimTKsimbody.h"
#include "SimTKsimbody_aux.h"
#include "SimTKmath.h"

using namespace OpenSim;
using namespace SimTK;
using namespace std;

#define ASSERT(cond) {if (!(cond)) throw(exception());}
#define ASSERT_EQUAL(expected, found, tolerance) {double tol = std::max((tolerance), std::abs((expected)*(tolerance))); if ((found)<(expected)-(tol) || (found)>(expected)+(tol)) throw(exception());}

//==========================================================================================================
// Test Cases
//==========================================================================================================
int testStates()
{
	//==========================================================================================================
	// Setup OpenSim model
	Model model("arm26.osim");
    ControlSetController* controller = new ControlSetController();
    controller->setControlSetFileName( "arm26_StaticOptimization_controls.xml" );
  
    model.updControllerSet().append( controller );
    State state = model.initSystem();
    Vector y1 = state.getY();
    model.equilibrateMuscles(state);

	//==========================================================================================================
	// Compute the force and torque at the specified times.

    RungeKuttaMersonIntegrator integrator(model.getSystem());
    Manager manager(model, integrator);
    manager.setInitialTime(0.0);
    manager.setFinalTime(0.05);
    manager.integrate(state);
    Vector y2 = state.getY();
    State state2 = model.initSystem();
    Vector y3 = state2.getY();
    model.setDefaultsFromState(state);
    state2 = model.initSystem();
    Vector y4 = state2.getY();
    for (int i = 0; i < y1.size(); i++) 
    {
        ASSERT_EQUAL(y1[i], y3[i], 1e-5);
        ASSERT_EQUAL(y2[i], y4[i], 1e-5);
    }
    ASSERT(max(abs(y1-y2)) > 1e-4);
    return 0;
}

int main()
{
    LoadOpenSimLibrary("osimActuators");
    testStates();
	return 0;
}
