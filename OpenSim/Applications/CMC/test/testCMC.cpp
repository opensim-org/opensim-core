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

using namespace OpenSim;
using namespace std;

#define ASSERT(cond) {if (!(cond)) throw(exception());}

#define ASSERT_EQUAL(expected, found, tol) {if ((found)<(expected)-(tol) || (found)>(expected)+(tol)) throw(exception());}

void testGait2354() {

	CMCTool cmc("subject01_Setup_CMC.xml");
	cmc.run();

	Storage results("subject01_ResultsCMC/subject01_walk1_Kinematics_q.sto");
	Storage standard("subject01_walk1_RRA_Kinematics_initial_q.sto");

	ASSERT(results.getFirstTime() >= standard.getFirstTime());
	ASSERT(results.getLastTime() <= standard.getLastTime());

	// Compare the output file to the target trajectory and make sure they
	// are sufficiently close.

	double meanError = 0.0;
	double maxError = 0.0;
	int count = 0;
	Array<double> data;
    string  label;
	string max_label;
	double maxErrTime = -1.0;
	

	for (int i = 0; i < results.getSize(); ++i) {
		StateVector* state = results.getStateVector(i);
		double time = state->getTime();

		data.setSize(state->getSize());
		standard.getDataAtTime(time, state->getSize(), data);
		for (int j = 0; j < state->getSize(); ++j) {
            // not checking the mtp and subtalar because they were locked when the reference 
            // was generated but not in the test.  The are not locked in the test  because 
            // the optimizer is not converging in 2.0 or 1.9 if these angles are locked
            label = results.getColumnLabels()[j+1];

            double diff = std::abs(data[standard.getStateIndex(label)]-state->getData()[j]);
    	    meanError += diff;
			if(diff > maxError){
				maxError = diff;
				max_label = label;
				maxErrTime = time;
			}

			count++;
		}
	}
	meanError = meanError/count;
    cout << "meanError = " << meanError << "   maxError = " << maxError << " for " << max_label << " at time = " << maxErrTime << endl;
	// average error should be below 0.25 degrees
	ASSERT(meanError < 0.25);
}

void testSingleMuscle() {

	ForwardTool forward("block_hanging_from_muscle_Setup_Forward.xml");
	forward.run();

	CMCTool cmc("block_hanging_from_muscle_Setup_CMC.xml");
	cmc.run();

	Storage fwd_result("block_hanging_from_muscle_ForwardResults/block_hanging_from_muscle_states.sto");
	Storage cmc_result("block_hanging_from_muscle_ResultsCMC/block_hanging_from_muscle_states.sto");

	// Compare the controls calculated by CMC to the input ones, and see if they
	// are sufficiently close.
	Array<double> t_fwd(0), u_fwd(0);
	Array<double> t_cmc(0), u_cmc(0);

	// Get speed of block from forward and cmc
	fwd_result.getTimeColumn(t_fwd); fwd_result.getDataColumn("block_ty_u", u_fwd);
	cmc_result.getTimeColumn(t_cmc); cmc_result.getDataColumn("block_ty_u", u_cmc);

	// Interpolate data so we can sample uniformly to compare
	PiecewiseLinearFunction u_fwd_function(u_fwd.getSize(), &t_fwd[0] , &u_fwd[0]);
	//PiecewiseLinearFunction tzu_cmc_function(tz_u_cmc.getSize(), &t_cmc[0] , &tz_u_cmc[0]);

	SimTK::Vector tv(1, 0.0);
	for (int i = 0; i < t_cmc.getSize(); ++i) {
		tv[0] = t_cmc[i];
		cout << "time = " << t_cmc[i] << "   error = " << u_fwd_function.calcValue(tv)-u_cmc[i] << endl;
        ASSERT_EQUAL(u_fwd_function.calcValue(tv), u_cmc[i], 0.02);
	}
}

void testTwoMusclesOnBlock() {

	ForwardTool forward("twoMusclesOnBlock_Setup_Forward.xml");
	forward.run();

	CMCTool cmc("twoMusclesOnBlock_Setup_CMC.xml");
	cmc.run();

	Storage fwd_result("twoMusclesOnBlock_ForwardResults/twoMusclesOnBlock_forward_states.sto");
	Storage cmc_result("twoMusclesOnBlock_ResultsCMC/twoMusclesOnBlock_tugOfWar_states.sto");

	// Compare the controls calculated by CMC to the input ones, and see if they
	// are sufficiently close.
	Array<double> t_fwd(0), u_fwd(0);
	Array<double> t_cmc(0), u_cmc(0);

	// Get speed of block from forward and cmc
	fwd_result.getTimeColumn(t_fwd); fwd_result.getDataColumn("tz_block_u", u_fwd);
	cmc_result.getTimeColumn(t_cmc); cmc_result.getDataColumn("tz_block_u", u_cmc);

	// Interpolate data so we can sample uniformly to compare
	PiecewiseLinearFunction u_fwd_function(u_fwd.getSize(), &t_fwd[0] , &u_fwd[0]);
	//PiecewiseLinearFunction tzu_cmc_function(tz_u_cmc.getSize(), &t_cmc[0] , &tz_u_cmc[0]);

	SimTK::Vector tv(1, 0.0);
	for (int i = 0; i < t_cmc.getSize(); ++i) {
		tv[0] = t_cmc[i];
		cout << "time = " << t_cmc[i] << "   error = " << u_fwd_function.calcValue(tv)-u_cmc[i] << endl;
        ASSERT_EQUAL(u_fwd_function.calcValue(tv), u_cmc[i], 0.02);
	}
}

void testEMGDrivenArm() {

	CMCTool cmc("arm26_Setup_ComputedMuscleControl_EMG.xml");
	cmc.run();
	cmc.print("check.xml");

}

int main() {
    try {
		//testEMGDrivenArm();
		testSingleMuscle();
		testTwoMusclesOnBlock();
		testGait2354();
    }
    catch(const std::exception& e) {
        cout << "exception: " << e.what() << endl;
        return 1;
    }
    cout << "Done" << endl;
    return 0;
}
