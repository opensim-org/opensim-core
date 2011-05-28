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
// author:  Frank C. Anderson, Ajay Seth

// INCLUDE
#include <string>
#include <iostream>
#include <OpenSim/Simulation/Control/Controller.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/LoadModel.h>
#include <OpenSim/Simulation/Model/AnalysisSet.h>
#include <OpenSim/Tools/ForwardTool.h>
#include <OpenSim/Analyses/Kinematics.h>
#include <OpenSim/Analyses/Actuation.h>
#include <OpenSim/Analyses/PointKinematics.h>
#include <OpenSim/Analyses/BodyKinematics.h>




using namespace OpenSim;
using namespace std;

#define ASSERT(cond) {if (!(cond)) throw(exception());}
#define ASSERT_EQUAL(expected, found, tolerance) {double tol = std::max((tolerance), std::abs((expected)*(tolerance))); if ((found)<(expected)-(tol) || (found)>(expected)+(tol)) throw(exception());}

/*
#define ASSERT_EQUAL(expected, found, tolerance) {double tol = std::max((tolerance), std::abs((expected)*(tolerance))); if ((found)<(expected)-(tol) || (found)>(expected)+(tol)) printf("ASSERT_EQUAL FAILED expected=%f found=%f\n", expected,found);}

*/
#define ASSERT_EQUAL_STATE(expected, found, tolerance) {double tol = std::max((tolerance), std::abs((expected)*(tolerance))); if ((found)<(expected)-(tol) || (found)>(expected)+(tol)) {  \
			cout << "ASSERT_EQUAL FAILED t="<< time <<" state# "<<j << " " << standard->getColumnLabels()[j+1] << " std=" << data[j] <<"  computed=" << state->getData()[j]  << endl ; \
            failed = true;  \
 } else { \
			cout << "ASSERT_EQUAL PASSED t="<< time <<" state# "<<j << " " << standard->getColumnLabels()[j+1] << " std=" << data[j] <<"  computed=" << state->getData()[j]  << endl ; \
} }



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
	for (int j = 0; j < storage.getSize(); ++j) {
		StateVector* state = storage.getStateVector(j);
		double time = state->getTime();
		ASSERT(time > previousTime);
		previousTime = time;
		ASSERT_EQUAL(0.1*std::cos(k*time), state->getData()[0], 1.5e-3);
		ASSERT_EQUAL(0.0, state->getData()[1], 1e-5);
		ASSERT_EQUAL(0.0, state->getData()[2], 1e-5);
		ASSERT_EQUAL(-k*0.1*std::sin(k*time), state->getData()[3], 1.5e-3);
		ASSERT_EQUAL(0.0, state->getData()[4], 1e-5);
		ASSERT_EQUAL(0.0, state->getData()[5], 1e-5);
	}
	ASSERT(previousTime == 1.0);
}


void testArm26() {
    bool failed = false;
	ForwardTool forward("arm26_Setup_Forward.xml");
	forward.run();
	forward.print("check.xml");
	Storage results("Results/arm26_states.sto");
	Storage* standard = new Storage();
    std::string statesFileName("std_arm26_states.sto");
    forward.loadStatesStorage( statesFileName, standard );
	StateVector* state = results.getStateVector(0);
	double time = state->getTime();
	Array<double> data;
	data.setSize(state->getSize());
	standard->getDataAtTime(time, state->getSize(), data);
	for (int j = 0; j < state->getSize(); ++j) {
        ASSERT_EQUAL_STATE(data[j], state->getData()[j], 2.5e-3);
    }

	int i = results.getSize()-1;
    state = results.getStateVector(i);
    time = state->getTime();
	data.setSize(state->getSize());
	standard->getDataAtTime(time, state->getSize(), data);
	for (int j = 0; j < state->getSize(); ++j) {
        ASSERT_EQUAL_STATE(data[j], state->getData()[j], 2.5e-3);
	}
    if( failed ) throw(exception());
}

void testGait2354() {
    bool failed = false;
	//ExternalLoads ext;
	//ext.print("ExternalLoads.xml");
	ForwardTool forward("setup_gait2354.xml");
	forward.run();
	forward.print("check.xml");
	Storage results("Results/gait2354_states.sto");
	Storage* standard = new Storage();
    std::string statesFileName("std_gait2354_states.sto");
    forward.loadStatesStorage( statesFileName, standard );

	Array<double> data;
	StateVector* state = results.getStateVector(0);
	double time = state->getTime();
	data.setSize(state->getSize());
	standard->getDataAtTime(time, state->getSize(), data);

	// At initial time all states should be identical except for locked joints may vary slightly due to 
	// differences in OpenSim's integrator and SimTK's
	int nc = forward.getModel().getNumCoordinates();
	for (int j = 0; j < state->getSize(); ++j) {
		ASSERT_EQUAL_STATE(data[j], state->getData()[j], 5e-2);
	}

	int i = results.getSize()-1;
	state = results.getStateVector(i);
	time = state->getTime();
	standard->getDataAtTime(time, state->getSize(), data);

	// NOTE: Gait model is running forward open-loop. We cannot expect all the states to
	// be "bang on" and we expect a gradual drift in the coordinates.  Check to see that
	// coordinates haven't drifted too far off.
	for (int j = 0; j < nc; ++j) {
	    ASSERT_EQUAL_STATE(data[j], state->getData()[j], 5e-1);
	}

    if( failed ) throw(exception());
}

void testGait2354WithController() {
    bool failed = false;
	ForwardTool forward("subject01_Setup_Forward_Controller.xml");
	forward.run();
	forward.print("check.xml");
	Storage results("ResultsSimpleFeedbackController/subject01_walk1_states.sto");
	Storage* standard = new Storage();
    std::string statesFileName("subject01_walk1_states.sto");
    forward.loadStatesStorage( statesFileName, standard );

	Array<double> data;
	int i = results.getSize() - 1;
	StateVector* state = results.getStateVector(i);
	double time = state->getTime();
	data.setSize(state->getSize());
	standard->getDataAtTime(time, state->getSize(), data);
	int nc = forward.getModel().getNumCoordinates();
	for (int j = 0; j < nc; ++j) {      
	    ASSERT_EQUAL_STATE(data[j], state->getData()[j], 5e-2);
	}
    if( failed ) throw(exception());
   
}

int main() {
    try {

        testPendulum();	// test manager/integration process
		std::cout << "Pendulum test PASSED " << std::endl;

		testArm26();	// now add computation of controls and generation of muscle forces
		std::cout << "arm26 test PASSED " << std::endl;

		testGait2354();    //finally include applied ground reactions forces 
		std::cout << "gait2354 test PASSED " << std::endl;

		testGait2354WithController();
		std::cout << "gait2354 with correction controller test PASSED " << std::endl;
	}
    catch(const std::exception& e) {
        cout << "exception: " << e.what() << endl;
        return 1;
    }
    cout << "Done" << endl;
    return 0;
}
