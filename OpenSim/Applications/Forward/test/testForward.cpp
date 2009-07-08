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
// forward.cpp
// author:  Frank C. Anderson

// INCLUDE
#include <string>
#include <iostream>
#include <OpenSim/Common/Mtx.h>
#include <OpenSim/Common/IO.h>
#include <OpenSim/Common/LoadOpenSimLibrary.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/LoadModel.h>
#include <OpenSim/Simulation/Model/AnalysisSet.h>
#include <OpenSim/Tools/ForwardTool.h>
#include <OpenSim/Analyses/Kinematics.h>
#include <OpenSim/Analyses/Actuation.h>
#include <OpenSim/Analyses/PointKinematics.h>
#include <OpenSim/Analyses/BodyKinematics.h>
#include <OpenSim/Analyses/ActuatorGeneralizedForces.h>




using namespace OpenSim;
using namespace std;

#define ASSERT(cond) {if (!(cond)) throw(exception());}

#define ASSERT_EQUAL(expected, found, tolerance) {double tol = std::max((tolerance), std::abs((expected)*(tolerance))); if ((found)<(expected)-(tol) || (found)>(expected)+(tol)) throw(exception());}

void testPendulum() {
	ForwardTool forward("setup_pend.xml");
	forward.run();
	forward.print("check.xml");
	Storage storage("Results/pendulum_states.sto");
	ASSERT(storage.getFirstTime() == 0.0);
	ASSERT(storage.getLastTime() == 1.0);
	ASSERT(storage.getSize() > 10);

	// Since the pendulum is only swinging through small angles, it should be very
	// close to a simple harmonic oscillator.

	double previousTime = -1.0;
	double k = std::sqrt(9.80665000/0.5);
	for (int i = 0; i < storage.getSize(); ++i) {
		StateVector* state = storage.getStateVector(i);
		double time = state->getTime();
		ASSERT(time > previousTime);
		previousTime = time;
		ASSERT_EQUAL(0.1*std::cos(k*time), state->getData()[0], 1e-3);
		ASSERT_EQUAL(0.0, state->getData()[1], 1e-5);
		ASSERT_EQUAL(0.0, state->getData()[2], 1e-5);
		ASSERT_EQUAL(-k*0.1*std::sin(k*time), state->getData()[3], 1e-3);
		ASSERT_EQUAL(0.0, state->getData()[4], 1e-5);
		ASSERT_EQUAL(0.0, state->getData()[5], 1e-5);
	}
	ASSERT(previousTime == 1.0);
}


void testGait2354() {
	ForwardTool forward("setup_gait2354.xml");
	forward.run();
	forward.print("check.xml");
	Storage results("Results/gait2354_states.sto");
	Storage standard("std_gait2354_states.sto");
	ASSERT(results.getFirstTime() == standard.getFirstTime());
	ASSERT(results.getLastTime() == standard.getLastTime());
	ASSERT(results.getSize() > 100);

	// For each state of the result file, make sure it is sufficiently
	// close to the standard file at the same time.

	double previousTime = -1.0;
	Array<double> data;
	for (int i = 0; i < results.getSize(); ++i) {
		StateVector* state = results.getStateVector(i);
		double time = state->getTime();
		ASSERT(time > previousTime);
		previousTime = time;
		data.setSize(state->getSize());
		standard.getDataAtTime(time, state->getSize(), data);
		for (int j = 0; j < state->getSize(); ++j) {
            ASSERT_EQUAL(data[j], state->getData()[j], 1e-3);
		}
	}
	ASSERT(previousTime == 1.0);
}

int main() {
    try {
#ifndef STATIC_OSIM_LIBS
		 LoadOpenSimLibrary("osimSimbodyEngine");
#endif
		 testPendulum();
		 testGait2354();
    }
    catch(const std::exception& e) {
        cout << "exception: " << e.what() << endl;
        return 1;
    }
    cout << "Done" << endl;
    return 0;
}
