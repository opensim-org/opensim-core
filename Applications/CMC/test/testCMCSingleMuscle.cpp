/* -------------------------------------------------------------------------- *
 *                           OpenSim:  testCMCSingleMuscle.cpp                *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
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

    try { testSingleMuscle();}
    catch (const std::exception& e)
        {   cout << e.what() <<endl; 
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
    CMCTool cmc("block_hanging_from_muscle_Setup_CMC.xml");

    const string& muscleType = 
        cmc.getModel().getMuscles()[0].getConcreteClassName();
    string base = "testSingleMuscle_" + muscleType;

    // test MUST fail if forward simulation fails to run to completion.
    OPENSIM_THROW_IF(!forward.run(), Exception, base + ": Failed running ForwardTool.");
    // test MUST fail if CMC fails to track to completion.
    OPENSIM_THROW_IF(!cmc.run(), Exception, base + ": Failed running CMCTool.");

    Storage fwd_controls("block_hanging_from_muscle_ForwardResults/block_hanging_from_muscle_controls.sto");
    Storage cmc_controls("block_hanging_from_muscle_ResultsCMC/block_hanging_from_muscle_controls.sto");

    Storage fwd_force("block_hanging_from_muscle_ForwardResults/block_hanging_from_muscle_ForceReporter_forces.sto");
    Storage cmc_force("block_hanging_from_muscle_ResultsCMC/block_hanging_from_muscle_Actuation_force.sto");

    Storage fwd_states("block_hanging_from_muscle_ForwardResults/block_hanging_from_muscle_states.sto");
    Storage cmc_states("block_hanging_from_muscle_ResultsCMC/block_hanging_from_muscle_states.sto");

    std::vector<double> control_tols(1, 4.0e-3); // peak control is 0.2 so this is 2% of peak
    std::vector<double> force_tols(1, 1.0e-1);   // 0.1 N 
    std::vector<double> state_tols(4, 1.0e-3);   // errors: q<1mm, u<1mm/s, a<0.001, fl<1mm

    CHECK_STORAGE_AGAINST_STANDARD(cmc_controls, fwd_controls, control_tols,
        __FILE__, __LINE__, base + " controls failed");

    CHECK_STORAGE_AGAINST_STANDARD(cmc_force, fwd_force, force_tols,
        __FILE__, __LINE__, base + " forces failed");

    CHECK_STORAGE_AGAINST_STANDARD(fwd_states, cmc_states, state_tols,
        __FILE__, __LINE__, base+" states failed");
    
    cout << "\n" << base << " passed\n" << endl;
}
