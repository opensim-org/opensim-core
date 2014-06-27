/* -------------------------------------------------------------------------- *
 *                           OpenSim:  testCMCSingleMuscle.cpp                *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2014 Stanford University and the Authors                *
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

// INCLUDE
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/AnalysisSet.h>
#include <OpenSim/Tools/CMCTool.h>
#include <OpenSim/Tools/ForwardTool.h>
#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>

using namespace OpenSim;
using namespace std;

void testSingleMuscle();

int main() {

    SimTK::Array_<std::string> failures;

    try {testSingleMuscle();}
    catch (const std::exception& e)
		{  cout << e.what() <<endl; failures.push_back("testSingleMuscle"); }

	// redo with the Millard2012EquilibriumMuscle 
	Object::renameType("Thelen2003Muscle", "Millard2012EquilibriumMuscle");

    try {testSingleMuscle();}
    catch (const std::exception& e)
		{	cout << e.what() <<endl; 
			failures.push_back("testSingleMuscle_Millard"); }
    
    if (!failures.empty()) {
        cout << "Done, with failure(s): " << failures << endl;
        return 1;
    }

	cout << "Done" << endl;

    return 0;
}

void testSingleMuscle() {
	cout<<"\n******************************************************************" << endl;
	cout << "*                         testSingleMuscle                       *" << endl;
	cout << "******************************************************************\n" << endl;
	ForwardTool forward("block_hanging_from_muscle_Setup_Forward.xml");
	forward.run();

	CMCTool cmc("block_hanging_from_muscle_Setup_CMC.xml");
	cmc.run();

	Storage fwd_result("block_hanging_from_muscle_ForwardResults/block_hanging_from_muscle_states.sto");
	Storage cmc_result("block_hanging_from_muscle_ResultsCMC/block_hanging_from_muscle_states.sto");

	Array<double> tols(0.0005, 4);
	const string& muscleType = cmc.getModel().getMuscles()[0].getConcreteClassName();
	string base = "testSingleMuscle "+ muscleType;

	CHECK_STORAGE_AGAINST_STANDARD(cmc_result, fwd_result, tols, __FILE__, __LINE__, base+" failed");
	
	cout << "\n" << base << " passed\n" << endl;
}
