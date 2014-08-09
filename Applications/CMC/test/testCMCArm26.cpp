/* -------------------------------------------------------------------------- *
 *                           OpenSim:  testCMCArm26.cpp                       *
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

void testArm26();

int main() {

    SimTK::Array_<std::string> failures;

    try {testArm26();}
    catch (const std::exception& e)
        {  cout << e.what() <<endl; failures.push_back("testArm26"); }

    // redo with the Millard2012EquilibriumMuscle 
    Object::renameType("Thelen2003Muscle", "Millard2012EquilibriumMuscle");
    
    try {testArm26();}
    catch (const std::exception& e)
        {  cout << e.what() <<endl; failures.push_back("testArm26_Millard"); }

    if (!failures.empty()) {
        cout << "Done, with failure(s): " << failures << endl;
        return 1;
    }

    cout << "Done" << endl;

    return 0;
}

void testArm26() {
    cout<<"\n******************************************************************" << endl;
    cout << "*                             testArm26                          *" << endl;
    cout << "******************************************************************\n" << endl;
    CMCTool cmc("arm26_Setup_CMC.xml");
    cmc.run();

    Storage results("Results_Arm26/arm26_states.sto"), temp("std_arm26_states.sto");
    Storage *standard = new Storage();
    cmc.getModel().formStateStorage(temp, *standard);

    Array<double> rms_tols(0.02, 2*2+2*6); // activations within 2%, angles within .6 degrees
    const string& muscleType = cmc.getModel().getMuscles()[0].getConcreteClassName();
    string base = "testArm26 "+ muscleType;

    if(muscleType != "Thelen2003Muscle"){
        rms_tols[6] = 0.05;
        rms_tols[8] = 0.05;
        rms_tols[10] = 0.05;
        rms_tols[12] = 0.05;
        rms_tols[14] = 0.05;
    }

    CHECK_STORAGE_AGAINST_STANDARD(results, *standard, rms_tols, __FILE__, __LINE__, 
        base+" failed");

    
    cout << "\n" << base <<" passed\n" << endl;
}
