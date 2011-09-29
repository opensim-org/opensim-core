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
#include <OpenSim/Common/IO.h>
#include <OpenSim/Common/LoadOpenSimLibrary.h>
#include "OpenSim/Common/PiecewiseLinearFunction.h"
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/LoadModel.h>
#include <OpenSim/Simulation/Model/AnalysisSet.h>
#include <OpenSim/Tools/CMCTool.h>
#include <OpenSim/Tools/ForwardTool.h>
#include <OpenSim/Analyses/Kinematics.h>
#include <OpenSim/Analyses/Actuation.h>
#include <OpenSim/Analyses/PointKinematics.h>
#include <OpenSim/Analyses/BodyKinematics.h>
#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>

using namespace OpenSim;
using namespace std;

void testSingleMuscle();
void testTwoMusclesOnBlock();
void testArm26();
void testGait2354();
void testEMGDrivenArm();
void testHamnerRunningModel();

int main() {
    try {
		testSingleMuscle();
		testTwoMusclesOnBlock();
		testArm26();
		testGait2354();
		testEMGDrivenArm();
		//testHamnerRunningModel();
    }
    catch (const Exception& e) {
        e.print(cerr);
        return 1;
    }
    cout << "Done" << endl;
    return 0;
}


void testSingleMuscle() {

	ForwardTool forward("block_hanging_from_muscle_Setup_Forward.xml");
	forward.run();

	CMCTool cmc("block_hanging_from_muscle_Setup_CMC.xml");
	cmc.run();

	Storage fwd_result("block_hanging_from_muscle_ForwardResults/block_hanging_from_muscle_states.sto");
	Storage cmc_result("block_hanging_from_muscle_ResultsCMC/block_hanging_from_muscle_states.sto");

	CHECK_STORAGE_AGAINST_STANDARD(cmc_result, fwd_result, Array<double>(0.0005, 4), __FILE__, __LINE__, "testSingleMuscle failed");
	cout << "testSingleMuscle passed\n" << endl;
}

void testTwoMusclesOnBlock() {

	ForwardTool forward("twoMusclesOnBlock_Setup_Forward.xml");
	forward.run();
	
	CMCTool cmc("twoMusclesOnBlock_Setup_CMC.xml");
	cmc.run();

	Storage fwd_result("twoMusclesOnBlock_ForwardResults/twoMusclesOnBlock_forward_states.sto");
	Storage cmc_result("twoMusclesOnBlock_ResultsCMC/twoMusclesOnBlock_tugOfWar_states.sto");

	Array<double> rms_tols(0.0005, 6);
	rms_tols[1] = 0.001; // block_u
	rms_tols[2] = 0.05;  // muscle 1 activation
	rms_tols[3] = 0.001; // muscle 1 fiber length 
	rms_tols[4] = 0.05;  // muscle 2 activation
	rms_tols[5] = 0.001; // muscle 2 fiber length 

	CHECK_STORAGE_AGAINST_STANDARD(cmc_result, fwd_result, rms_tols, __FILE__, __LINE__, "testTwoMusclesOnBlock failed");
	cout << "testTwoMusclesOnBlock passed\n" << endl;
			}


void testArm26() {

	CMCTool cmc("arm26_Setup_CMC.xml");
	cmc.run();
	Storage results("Results_Arm26/arm26_states.sto"), standard("std_arm26_states.sto");
	CHECK_STORAGE_AGAINST_STANDARD(results, standard, Array<double>(6.0e-3, 16), __FILE__, __LINE__, "testArm failed");

	Array<double> rms_tols(5.0e-3, 2*2+2*6);
	rms_tols[0] = 0.001; // shoulder q
	rms_tols[1] = 0.001;  // elbow q

	CHECK_STORAGE_AGAINST_STANDARD(results, standard, rms_tols, __FILE__, __LINE__, "testArm26 failed");

	cout << "\ntestArm26 passed\n" << endl;
}

void testGait2354() {

	CMCTool cmc("subject01_Setup_CMC.xml");
	cmc.run();

	Storage results("subject01_ResultsCMC/subject01_walk1_states.sto");
	Storage standard("std_subject01_walk1_states.sto");

	Array<string> col_labels = standard.getColumnLabels();
	Array<double> rms_tols(0.025, col_labels.getSize()-1);
	for (int i = 23; i < 46; ++i){
		rms_tols[i] = 0.75; // velocities
	}
	for (int i = 46; i < rms_tols.getSize(); ++i){
		rms_tols[i] = 0.15; // muscle activations and fiber-lengths
	}

	CHECK_STORAGE_AGAINST_STANDARD(results, standard, rms_tols, __FILE__, __LINE__, "testGait2354 failed");

	Storage trackingError("subject01_ResultsCMC/subject01_walk1_pErr.sto");

	cout << "\n testGait2354 passed\n" << endl;
	}


void testEMGDrivenArm() {

	CMCTool cmc("arm26_Setup_ComputedMuscleControl_EMG.xml");
	//cmc.run();

	Storage results("Results_Arm26_EMG/arm26_states.sto"), standard("std_arm26_states.sto");

	Array<double> rms_tols(0.25, 2*2+2*6);
	rms_tols[0] = 0.001; // shoulder q
	rms_tols[1] = 0.001;  // elbow q
	rms_tols[6] = 0.95;  // tri_lat normally off but because of EMG tracking it is on max

	CHECK_STORAGE_AGAINST_STANDARD(results, standard, rms_tols, __FILE__, __LINE__, "testEMGDrivenArm failed");

	cout << "\n testEMGDrivenArm passed\n" << endl;
	}

void testHamnerRunningModel()
{
	CMCTool cmc("subject02_Setup_CMC_cycle02_v24.xml");
	cmc.run();

	Storage results("CMC_Running_Results/subject02_running_CMC_Kinematics_q.sto");
	Storage standard("subject02_running_RRA_Kinematics_q.sto");

	ASSERT(results.getFirstTime() >= standard.getFirstTime());
	ASSERT(results.getLastTime() <= standard.getLastTime());

}
