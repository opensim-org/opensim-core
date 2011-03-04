// testAssemblySolver.cpp
// Author:  Ajay Seth
/*
* Copyright (c) 2010, Stanford University. All rights reserved. 
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
//	testAssemblySolver loads models with constraints to verify that constraints are
//  adequately satified or that an appropriate exception is thrown.
//
//==========================================================================================================
#include <iostream>
#include <OpenSim/Common/IO.h>
#include <OpenSim/Common/Exception.h>
#include <OpenSim/Simulation/Manager/Manager.h>
#include <OpenSim/Simulation/Control/ControlSetController.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/ConstraintSet.h>
#include <OpenSim/Common/LoadOpenSimLibrary.h>

#include "SimTKsimbody.h"
#include "SimTKmath.h"

using namespace OpenSim;
using namespace SimTK;
using namespace std;

#define ASSERT(cond) {if (!(cond)) throw(exception());}
#define ASSERT_EQUAL(expected, found, tolerance) {double tol = std::max((tolerance), std::abs((expected)*(tolerance))); if ((found)<(expected)-(tol) || (found)>(expected)+(tol)) throw(exception());}

//==========================================================================================================
// Test Cases
//==========================================================================================================
int testAssembleModelWithConstraints(string modelFile)
{
	//==========================================================================================================
	// Setup OpenSim model
	Model model(modelFile);

	const CoordinateSet &coords = model.getCoordinateSet();
	
	cout << "*********** Coordinates before initSystem ******************** " << endl;
	for(int i=0; i< coords.getSize(); i++) {
		cout << "Coordinate " << coords[i].getName() << " default value = " << coords[i].getDefaultValue() << endl;
	}

    State state = model.initSystem();

	cout << "*********** Coordinates after initSystem ******************** "  << endl;
	for(int i=0; i< coords.getSize(); i++) {
		cout << "Coordinate " << coords[i].getName() << " get value = " << coords[i].getValue(state) << endl;
	}

	// Verify that the reaction forces at the constraints are not rediculously large
	// They should sum to body-weight (more or less)
	model.getMultibodySystem().realize(state, SimTK::Stage::Acceleration);

	const ConstraintSet &constraints = model.getConstraintSet();

	SimTK::Vector_<SimTK::SpatialVec> constraintBodyForces(constraints.getSize());
	SimTK::Vector mobilityForces(0);

	double totalYforce = 0;
	
	for(int i=0; i< constraints.getSize(); i++) {
		constraints[i].calcConstraintForces(state, constraintBodyForces, mobilityForces);
		cout << "Constraint " << i << ":  " << constraints[i].getName() << endl;
		//constraintBodyForces.dump("Consrtaint Body Forces");
		totalYforce += constraintBodyForces(1)(1)(1);
	}
	
	cout << "Total Vertical Constraint Force:" << totalYforce << " N " << endl;

	double bw = -model.getSimbodyEngine().getMass()*(model.getGravity()[1]);

	ASSERT_EQUAL(totalYforce, bw, 1.0);


	//const CoordinateSet &coords = model.getCoordinateSet();
	double q_error = 0;
	for(int i=0; i< coords.getSize(); i++) {
		q_error += fabs(coords[i].getValue(state)-coords[i].getDefaultValue());
	}

	cout << "Average Change in  Default Configuration:" << q_error/coords.getSize() << endl;

	//==========================================================================================================
	// Integrate forward and init the state and update defaults to make sure
	// assembler is not effecting anything more than the pose.

    RungeKuttaMersonIntegrator integrator(model.getMultibodySystem());
    Manager manager(model, integrator);
    manager.setInitialTime(0.0);
    manager.setFinalTime(0.05);

	model.equilibrateMuscles(state);
	Vector y1 = state.getY();
 
	// defaults should capture an accurate snapshot of the model
	model.setDefaultsFromState(state);
    manager.integrate(state);
    Vector y2 = state.getY();

	// recreate system with states from defaults
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
	int status = 0;

    status = testAssembleModelWithConstraints("PushUpToesOnGroundExactConstraints.osim");

	status = testAssembleModelWithConstraints("PushUpToesOnGroundLessPreciseConstraints.osim");

	status = testAssembleModelWithConstraints("PushUpToesOnGroundWithMuscles.osim");

	return status;
}
