/* -------------------------------------------------------------------------- *
 *                           OpenSim:  testCMC.cpp                            *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
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
void testSingleMuscle();
void testTwoMusclesOnBlock();
void testArm26();
void testEMGDrivenArm();
void testGait10dof18musc();

int main() {

    SimTK::Array_<std::string> failures;

	try {testSingleRigidTendonMuscle();}
    catch (const std::exception& e)
		{  cout << e.what() <<endl; failures.push_back("testSingleRigidTendonMuscle"); }
    try {testSingleMuscle();}
    catch (const std::exception& e)
		{  cout << e.what() <<endl; failures.push_back("testSingleMuscle"); }

    try {testTwoMusclesOnBlock();}
    catch (const std::exception& e)
		{  cout << e.what() <<endl; failures.push_back("testTwoMusclesOnBlock"); }
   
    try {testArm26();}
    catch (const std::exception& e)
		{  cout << e.what() <<endl; failures.push_back("testArm26"); }

	try {testEMGDrivenArm();}
    catch (const std::exception& e)
		{  cout << e.what() <<endl; failures.push_back("testEMGDrivenArm"); }

	// redo with the Millard2012EquilibriumMuscle 
	Object::renameType("Thelen2003Muscle", "Millard2012EquilibriumMuscle");

    try {testSingleMillardRigidTendonMuscle();}
    catch (const std::exception& e)
		{	cout << e.what() <<endl;
			failures.push_back("testSingleMillardRigidTendonMuscle"); }

    try {testSingleMuscle();}
    catch (const std::exception& e)
		{	cout << e.what() <<endl; 
			failures.push_back("testSingleMuscle_Millard"); }

    try {testTwoMusclesOnBlock();}
    catch (const std::exception& e)
		{	cout << e.what() <<endl; 
			failures.push_back("testTwoMusclesOnBlock_Millard"); }
    
    try {testArm26();}
    catch (const std::exception& e)
		{  cout << e.what() <<endl; failures.push_back("testArm26_Millard"); }

	try {testEMGDrivenArm();}
    catch (const std::exception& e)
		{  cout << e.what() <<endl; failures.push_back("testEMGDrivenArm_Millard"); }

	try {testGait10dof18musc();}
    catch (const std::exception& e)
		{  cout << e.what() <<endl; failures.push_back("testGait10dof18musc_Millard"); }

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
	// samecontrols
	CMCTool cmc("block_hanging_from_muscle_Setup_CMC.xml");
	cmc.setModel(*modelCopy);
	cmc.run();

	Storage fwd_result("block_hanging_from_muscle_ForwardResults/block_hanging_from_muscle_states.sto");
	Storage cmc_result("block_hanging_from_muscle_ResultsCMC/block_hanging_from_muscle_states.sto");

	// Tolerance of 2mm or position error and 2mm/s translational velocity of the block
	CHECK_STORAGE_AGAINST_STANDARD(cmc_result, fwd_result, Array<double>(0.0025, 4), __FILE__, __LINE__, "testSingleRigidTendonMuscle failed");
	
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

	CHECK_STORAGE_AGAINST_STANDARD(cmc_result, fwd_result, Array<double>(0.002, 3), __FILE__, __LINE__, "testSingleMillardRigidTendonMuscle failed");

	cout << "testSingleMillardRigidTendonMuscle passed\n" << endl;
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

	Array<double> rms_tols(0.0025, 6);
	rms_tols[1] = 0.001; // block_u
	rms_tols[2] = 0.05;  // muscle 1 activation
	rms_tols[3] = 0.001; // muscle 1 fiber length 
	rms_tols[4] = 0.05;  // muscle 2 activation
	rms_tols[5] = 0.001; // muscle 2 fiber length 

	const string& muscleType = cmc.getModel().getMuscles()[0].getConcreteClassName();
	string base = "testTwoMusclesOnBlock "+ muscleType;

	CHECK_STORAGE_AGAINST_STANDARD(cmc_result, fwd_result, rms_tols, __FILE__, __LINE__,
		base+" failed");
	
	cout << "\n" << base << " passed\n" << endl;
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

void testEMGDrivenArm() {
	cout<<"\n******************************************************************" << endl;
	cout << "*                         testEMGDrivenArm                       *" << endl;
	cout << "******************************************************************\n" << endl;
	CMCTool cmc("arm26_Setup_ComputedMuscleControl_EMG.xml");
	cmc.run();

	Storage results("Results_Arm26_EMG/arm26_states.sto"), temp("std_arm26_states.sto");
	Storage *standard = new Storage();
	cmc.getModel().formStateStorage(temp, *standard);

	Array<double> rms_tols(0.02, 2*2+2*6);
	rms_tols[4] = 0.10;  // trilong
	rms_tols[6] = 0.25;  // trilat normally off but because of bicep long EMG tracking it turns on
	rms_tols[8] = 0.25;  // trimed normally off but because of bicep long EMG tracking it turns on
	rms_tols[10] = 0.50;  // biceps long normally low but because of EMG tracking should be on more
	rms_tols[12] = 0.50;  // biceps short normally on but because of EMG tracking should be lower

	CHECK_STORAGE_AGAINST_STANDARD(results, *standard, rms_tols, __FILE__, __LINE__, "testEMGDrivenArm failed");

	const string& muscleType = cmc.getModel().getMuscles()[0].getConcreteClassName();
	cout << "\ntestEMGDrivenArm "+muscleType+ " passed\n" << endl;
}

void testGait10dof18musc() {
	cout<<"\n******************************************************************" << endl;
	cout << "*                      testGait10dof18musc                       *" << endl;
	cout << "******************************************************************\n" << endl;
	CMCTool cmc("gait10dof18musc_Setup_CMC.xml");
	cmc.run();

	Storage results("gait10dof18musc_ResultsCMC/walk_subject_states.sto");
	Storage temp("gait10dof18musc_std_walk_subject_states.sto");

	Storage *standard = new Storage();
	cmc.getModel().formStateStorage(temp, *standard);

	Array<double> rms_tols(0.02, 2*10+2*18); // activations within 2%
	for(int i=0; i<20; ++i){
		rms_tols[i] = 0.01;  // angles and speeds within .6 degrees .6 degs/s 
	}

	CHECK_STORAGE_AGAINST_STANDARD(results, *standard, rms_tols, __FILE__, __LINE__, "testArm26 failed");

	const string& muscleType = cmc.getModel().getMuscles()[0].getConcreteClassName();
	cout << "\ntestGait10dof18musc "+muscleType+" passed\n" << endl;
}