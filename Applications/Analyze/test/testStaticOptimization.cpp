// testStaticOptimization.cpp
// Author: Ayman Habib
/*
* Copyright (c)  2005, Stanford University. All rights reserved. 
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
* 
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

// INCLUDE
#include <string>
#include <iostream>
#include <OpenSim/version.h>
#include <OpenSim/Common/IO.h>
#include <OpenSim/Common/LoadOpenSimLibrary.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/BodySet.h>
#include <OpenSim/Tools/AnalyzeTool.h>
#include <OpenSim/Analyses/StaticOptimization.h>
#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>

using namespace OpenSim;
using namespace std;

void testArm26(bool useDeprecatedMuscle);

int main()
{
	try {
		// test with arm26 model and updated Thelen2003Muscle model
		//testArm26(false);

		// test with arm26 model and deprecated Thelen2003Muscle model
		testArm26(true);
	}
	catch (const std::exception& e) {
        cout << "Failed: "<< e.what() << endl;
        return 1;
    }
    cout << "Done" << endl;
    return 0;
}

void testArm26(bool useDeprecatedMuscle)
{
	string resultsDir = "Results";
	if(	useDeprecatedMuscle ){
		Object::renameType("Thelen2003Muscle", "Thelen2003Muscle_Deprecated");
		resultsDir = "Results_Deprecated";
	}
	AnalyzeTool analyze1("arm26_Setup_StaticOptimization.xml");
	analyze1.setResultsDir(resultsDir);
	analyze1.run();
	Storage resultActivation1(resultsDir+"/arm26_StaticOptimization_activation.sto"), standardActivation1("std_arm26_StaticOptimization_activation.sto");
	Storage resultForce1(resultsDir+"/arm26_StaticOptimization_force.sto"), standardForce1("std_arm26_StaticOptimization_force.sto");
	CHECK_STORAGE_AGAINST_STANDARD(resultActivation1, standardActivation1, Array<double>(0.025, 6), __FILE__, __LINE__, "testArm failed");
	CHECK_STORAGE_AGAINST_STANDARD(resultForce1, standardForce1, Array<double>(2.0, 6), __FILE__, __LINE__, "testArm failed");
	cout << resultsDir <<": testArm passed" << endl;

	AnalyzeTool analyze2("arm26_bounds_Setup_StaticOptimization.xml");
	analyze2.setResultsDir(resultsDir);
	analyze2.run();
	Storage resultActivation2(resultsDir+"/arm26_bounds_StaticOptimization_activation.sto"), standardActivation2("std_arm26_bounds_StaticOptimization_activation.sto");
	Storage resultForce2(resultsDir+"/arm26_bounds_StaticOptimization_force.sto"), standardForce2("std_arm26_bounds_StaticOptimization_force.sto");
	CHECK_STORAGE_AGAINST_STANDARD(resultActivation2, standardActivation2, Array<double>(0.03, 6), __FILE__, __LINE__, "testArm with bounds failed");
	CHECK_STORAGE_AGAINST_STANDARD(resultForce2, standardForce2, Array<double>(2.5, 6), __FILE__, __LINE__, "testArm with bounds failed");
	cout << resultsDir << ": testArm with bounds passed" << endl;
}