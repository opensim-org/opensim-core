/* -------------------------------------------------------------------------- *
 *                           OpenSim:  testCMCTwoMusclesOnBlock.cpp           *
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

void testTwoMusclesOnBlock();

int main() {

    SimTK::Array_<std::string> failures;

    try {testTwoMusclesOnBlock();}
    catch (const std::exception& e)
        {  cout << e.what() <<endl; failures.push_back("testTwoMusclesOnBlock"); }

    // redo with the Millard2012EquilibriumMuscle 
    Object::renameType("Thelen2003Muscle", "Millard2012EquilibriumMuscle");

    try {testTwoMusclesOnBlock();}
    catch (const std::exception& e)
        {   cout << e.what() <<endl; 
            failures.push_back("testTwoMusclesOnBlock_Millard"); }
    
    if (!failures.empty()) {
        cout << "Done, with failure(s): " << failures << endl;
        return 1;
    }

    cout << "Done" << endl;

    return 0;
}

void testSingleRigidTendonMuscle() {
    cout << "\n******************************************************************" << endl;
    cout << "*                   testSingleRigidTendonMuscle                  *" << endl;
    cout << "******************************************************************\n" << endl;

    Model model("block_hanging_RigidTendonMuscle.osim");
    Model* modelCopy = model.clone();
    modelCopy->setup();
    ASSERT(model == *modelCopy);

    ForwardTool forward("block_hanging_from_muscle_Setup_Forward.xml");
    forward.setModel(model);
    forward.run();

    // Use copy of the model because forward adds a ControlSetController to the model and the controls from CMC
    // are added in with those "feedforward" controls. Instead we want to verify that CMC can compute these 
    // same controls
    CMCTool cmc("block_hanging_from_muscle_Setup_CMC.xml");
    cmc.setModel(*modelCopy);
    cmc.run();

    Storage fwd_result("block_hanging_from_muscle_ForwardResults/block_hanging_from_muscle_states.sto");
    Storage cmc_result("block_hanging_from_muscle_ResultsCMC/block_hanging_from_muscle_states.sto");

    // Tolerance of 2mm or position error and 2mm/s translational velocity of the block
    CHECK_STORAGE_AGAINST_STANDARD(cmc_result, fwd_result, 
        std::vector<double>(4, 0.0025), __FILE__, __LINE__,
        "testSingleRigidTendonMuscle failed");
    
    cout << "testSingleRigidTendonMuscle passed\n" << endl;
}


void testSingleMillardRigidTendonMuscle() {
    cout<<"\n******************************************************************" << endl;
    cout << "*               testSingleMillardRigidTendonMuscle               *" << endl;
    cout << "******************************************************************\n" << endl;
    ForwardTool forward("block_hanging_from_muscle_Setup_Forward.xml");
    Model& fwdModel = forward.getModel();
    fwdModel.getMuscles()[0].set_ignore_tendon_compliance(true); //make tendon rigid
    forward.run();

    CMCTool cmc("block_hanging_from_muscle_Setup_CMC.xml");
    Model& cmcModel = cmc.getModel();
    cmcModel.getMuscles()[0].set_ignore_tendon_compliance(true); //make tendon rigid
    cmc.run();

    Storage fwd_result("block_hanging_from_muscle_ForwardResults/block_hanging_from_muscle_states.sto");
    Storage cmc_result("block_hanging_from_muscle_ResultsCMC/block_hanging_from_muscle_states.sto");

    CHECK_STORAGE_AGAINST_STANDARD(cmc_result, fwd_result, 
        std::vector<double>(3, 0.002), __FILE__, __LINE__,
        "testSingleMillardRigidTendonMuscle failed");

    cout << "testSingleMillardRigidTendonMuscle passed\n" << endl;
}

void testTwoMusclesOnBlock() {
    cout<<"\n******************************************************************" << endl;
    cout << "*                       testTwoMusclesOnBlock                    *" << endl;
    cout << "******************************************************************\n" << endl;

    ForwardTool forward("twoMusclesOnBlock_Setup_Forward.xml");
    forward.run();
    
    CMCTool cmc("twoMusclesOnBlock_Setup_CMC.xml");
    cmc.run();

    Storage fwd_result("twoMusclesOnBlock_ForwardResults/twoMusclesOnBlock_forward_states.sto");
    Storage cmc_result("twoMusclesOnBlock_ResultsCMC/twoMusclesOnBlock_tugOfWar_states.sto");

    std::vector<double> rms_tols(6, 0.001);
    rms_tols[1] = 0.0025; // block_u
    rms_tols[2] = 0.05;  // muscle 1 activation
    rms_tols[4] = 0.05;  // muscle 2 activation

    const string& muscleType = cmc.getModel().getMuscles()[0].getConcreteClassName();
    string base = "testTwoMusclesOnBlock "+ muscleType;

    CHECK_STORAGE_AGAINST_STANDARD(cmc_result, fwd_result, rms_tols, 
        __FILE__, __LINE__, base+" failed");
    
    cout << "\n" << base << " passed\n" << endl;
}

