// testStaticOptimization.cpp
// Author: Ayman Habib, Matt Millard, Ajay Seth
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
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Tools/AnalyzeTool.h>
#include <OpenSim/Analyses/StaticOptimization.h>
#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>

using namespace OpenSim;
using namespace std;

/** @param muscleModelClassName selects from:
        Thelen2003Muscle_Deprecated
        Thelen2003Muscle
        Millard2012EquilibriumMuscle
        Millard2012AccelerationMuscle
*/
void testArm26(const string& muscleModelClassName);

int main()
{
	Array<string> muscleModelNames; 
	muscleModelNames.append("Thelen2003Muscle_Deprecated");
	muscleModelNames.append("Thelen2003Muscle");
	muscleModelNames.append("Millard2012EquilibriumMuscle");
	muscleModelNames.append("Millard2012AccelerationMuscle");

    cout << "=========================================================" << endl;
    cout << "                       WARNING                           " << endl;
    cout << "Athough this file says it testsStaticOptimization, it is" << endl;
    cout << "not a valid test. It merely checks to see if the current" << endl;
    cout << "static results agree with past static results.          " << endl;
    cout << endl;
    cout << "                This is not a test                      " << endl;
    cout << endl;
    cout << "A valid test might be done by instead checking the results" <<endl;
    cout << "of static against an analytical system, say an analytical " <<endl;
    cout << "muscle model for which the solution to the optimization   " <<endl;
    cout << "problem is known by construction.  M.Millard 2012" << endl;
    cout << "=========================================================" << endl;
	
	SimTK::Array_<std::string> failures;
	
	for(int i=0; i< muscleModelNames.getSize(); ++i){
		try { testArm26(muscleModelNames[i]); }
		catch (const std::exception& e) {
			cout << e.what() <<endl; 
			failures.push_back("testArm26_"+muscleModelNames[i]);
		}
	}
  
    if (!failures.empty()) {
        cout << "Done, with failure(s): " << failures << endl;
        return 1;
    }

    cout << "Done" << endl;
    return 0;
}

void testArm26(const string& muscleModelClassName)
{
	Object::renameType( "Thelen2003Muscle", muscleModelClassName);

	cout << "==============================================" << endl;
	cout << "       "<< muscleModelClassName << endl;
	cout << "==============================================" << endl;

	string resultsDir = "Results_"+muscleModelClassName;
	const string& muscName = muscleModelClassName;

	AnalyzeTool analyze1("arm26_Setup_StaticOptimization.xml");
	analyze1.setResultsDir(resultsDir);
	analyze1.run();

	Storage activations1(resultsDir+"/arm26_StaticOptimization_activation.sto");
	Storage stdActivations1("std_arm26_StaticOptimization_activation.sto");
	// Uncomment to use muscle model specific standard
	//Storage stdActivations1("std_arm26_"+muscName+"_SO_activation.sto");

	Storage forces1(resultsDir+"/arm26_StaticOptimization_force.sto");
    Storage stdForces1("std_arm26_StaticOptimization_force.sto");
	// Uncomment to use muscle model specific standard
	//Storage stdForces1("std_arm26_"+muscName+"_SO_force.sto");

	CHECK_STORAGE_AGAINST_STANDARD(activations1, stdActivations1, 
									Array<double>(0.025, 6), __FILE__, __LINE__, 
                                   "Arm26 activations "+muscName+" failed");

	CHECK_STORAGE_AGAINST_STANDARD(forces1, stdForces1, 
                                    Array<double>(2.0, 6), __FILE__, __LINE__, 
                                    "Arm26 forces "+muscName+" failed.");

	cout << resultsDir <<": test Arm26 passed." << endl;
	cout << "==========================================================\n" << endl;

	AnalyzeTool analyze2("arm26_bounds_Setup_StaticOptimization.xml");
	analyze2.setResultsDir(resultsDir);
	analyze2.run();

	Storage activations2(
		resultsDir+"/arm26_bounds_StaticOptimization_activation.sto");
	Storage stdActivations2("std_arm26_bounds_StaticOptimization_activation.sto"); 
	// Uncomment to use muscle model specific standard
	//Storage stdActivations2("std_arm26_bounds_"+muscName+"_SO_activation.sto");

	Storage forces2(resultsDir+"/arm26_bounds_StaticOptimization_force.sto");
    Storage stdForces2("std_arm26_bounds_StaticOptimization_force.sto");
	// Uncomment to use muscle model specific standard
	//Storage stdForces2("std_arm26_bounds_"+muscName+"_SO_force.sto");

	CHECK_STORAGE_AGAINST_STANDARD(activations2, stdActivations2,
        Array<double>(0.03, 6), __FILE__, __LINE__, 
        "Arm26 activation "+muscName+" with bounds failed.");

	CHECK_STORAGE_AGAINST_STANDARD(forces2, stdForces2, 
        Array<double>(2.5, 6), __FILE__,  __LINE__, 
        "Arm26 forces "+muscName+" with bounds failed.");

	cout << resultsDir << ": testbArm26 with bounds passed" << endl;
	cout << "==========================================================\n" << endl;
}