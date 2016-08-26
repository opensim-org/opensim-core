/* -------------------------------------------------------------------------- *
 *                           OpenSim:  testCMCTwoMusclesOnBlock.cpp           *
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

void testTwoMusclesOnBlock() {
    cout<<"\n******************************************************************" << endl;
    cout << "*                testTwoMusclesOnBlock_Millard                    *" << endl;
    cout << "******************************************************************\n" << endl;

    ForwardTool forward("twoMusclesOnBlock_Setup_Forward.xml");
    forward.setResultsDir("twoMusclesOnBlock_ForwardResults_Millard");
    forward.run();
    
    CMCTool cmc("twoMusclesOnBlock_Setup_CMC.xml");
    cmc.setResultsDir("twoMusclesOnBlock_ResultsCMC_Millard");
    cmc.setDesiredKinematicsFileName("twoMusclesOnBlock_ForwardResults_Millard/twoMusclesOnBlock_forward_states.sto");
    cmc.run();

    Storage fwd_result("twoMusclesOnBlock_ForwardResults_Millard/twoMusclesOnBlock_forward_states.sto");
    Storage cmc_result("twoMusclesOnBlock_ResultsCMC_Millard/twoMusclesOnBlock_tugOfWar_states.sto");

    Array<double> rms_tols(0.001, 6);
    rms_tols[1] = 0.0025; // block_u
    rms_tols[2] = 0.05;  // muscle 1 activation
    rms_tols[4] = 0.05;  // muscle 2 activation

    const string& muscleType = cmc.getModel().getMuscles()[0].getConcreteClassName();
    string base = "testTwoMusclesOnBlock "+ muscleType;

    CHECK_STORAGE_AGAINST_STANDARD(cmc_result, fwd_result, rms_tols, __FILE__, __LINE__,
        base+" failed");
    
    cout << "\n" << base << " passed\n" << endl;
}

int main() {

    SimTK::Array_<std::string> failures;

    Object::renameType("Thelen2003Muscle", "Millard2012EquilibriumMuscle");

    try{
        testTwoMusclesOnBlock();
    } catch(const std::exception& e) {  
        cout << e.what() <<endl; 
        failures.push_back("testTwoMusclesOnBlock_Millard"); 
    }
    
    if (!failures.empty()) {
        cout << "Done, with failure(s): " << failures << endl;
        return 1;
    }

    cout << "Done" << endl;

    return 0;
}



