/* -------------------------------------------------------------------------- *
 *                           OpenSim:  testCMCSingleRigidTendonMuscle.cpp     *
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

void testSingleRigidTendonMuscle();
void testSingleMillardRigidTendonMuscle();

int main() {

    SimTK::Array_<std::string> failures;

    try {testSingleRigidTendonMuscle();}
    catch (const std::exception& e)
        {  cout << e.what() <<endl; failures.push_back("testSingleRigidTendonMuscle"); }

    // redo with the Millard2012EquilibriumMuscle 
    Object::renameType("Thelen2003Muscle", "Millard2012EquilibriumMuscle");

    try {testSingleMillardRigidTendonMuscle();}
    catch (const std::exception& e)
        {   cout << e.what() <<endl;
            failures.push_back("testSingleMillardRigidTendonMuscle"); }
    
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

    ForwardTool forward("block_hanging_from_muscle_Setup_Forward.xml");
    forward.setResultsDir("block_hanging_from_rigid_thelen_muscle_ForwardResults");
    forward.run();

    // run CMC on kinematics from Forward simulation
    CMCTool cmc("block_hanging_from_muscle_Setup_CMC.xml");
    cmc.setResultsDir("block_hanging_from_rigid_thelen_muscle_ResultsCMC");
    cmc.setDesiredKinematicsFileName(
            "block_hanging_from_rigid_thelen_muscle_ForwardResults/"
            "block_hanging_from_muscle_states.sto");
    // int n_acts = cmc.getModel().getActuators().getSize();
    cmc.run();

    Storage fwd_result("block_hanging_from_rigid_thelen_muscle_ForwardResults/block_hanging_from_muscle_states.sto");
    Storage cmc_result("block_hanging_from_rigid_thelen_muscle_ResultsCMC/block_hanging_from_muscle_states.sto");

    // Tolerance of 2mm or position error and 2mm/s translational velocity of the block
    CHECK_STORAGE_AGAINST_STANDARD(cmc_result, fwd_result, 
        std::vector<double>(4, 0.002), __FILE__, __LINE__,
        "testSingleRigidTendonMuscle failed");
    
    cout << "testSingleRigidTendonMuscle passed\n" << endl;
}


void testSingleMillardRigidTendonMuscle() {
    cout<<"\n******************************************************************" << endl;
    cout << "*               testSingleMillardRigidTendonMuscle               *" << endl;
    cout << "******************************************************************\n" << endl;
    ForwardTool forward("block_hanging_from_muscle_Setup_Forward.xml");
    forward.setResultsDir("block_hanging_from_rigid_millard_muscle_ForwardResults");
    Model& fwdModel = forward.getModel();
    fwdModel.getMuscles()[0].set_ignore_tendon_compliance(true); //make tendon rigid
    forward.run();

    CMCTool cmc("block_hanging_from_muscle_Setup_CMC.xml");
    cmc.setResultsDir("block_hanging_from_rigid_millard_muscle_ResultsCMC");
    cmc.setDesiredKinematicsFileName(
            "block_hanging_from_rigid_millard_muscle_ForwardResults/"
            "block_hanging_from_muscle_states.sto");
    Model& cmcModel = cmc.getModel();
    cmcModel.getMuscles()[0].set_ignore_tendon_compliance(true); //make tendon rigid
    cmc.run();

    Storage fwd_result("block_hanging_from_rigid_millard_muscle_ForwardResults/block_hanging_from_muscle_states.sto");
    Storage cmc_result("block_hanging_from_rigid_millard_muscle_ResultsCMC/block_hanging_from_muscle_states.sto");

    CHECK_STORAGE_AGAINST_STANDARD(cmc_result, fwd_result, 
        std::vector<double>(3, 0.002), __FILE__, __LINE__,
        "testSingleMillardRigidTendonMuscle failed");

    cout << "testSingleMillardRigidTendonMuscle passed\n" << endl;
}
