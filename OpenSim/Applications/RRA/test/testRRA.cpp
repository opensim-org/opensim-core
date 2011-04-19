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
#include <OpenSim/Common/Mtx.h>
#include <OpenSim/Common/IO.h>
#include <OpenSim/Common/LoadOpenSimLibrary.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/BodySet.h>
#include <OpenSim/Simulation/Model/LoadModel.h>
#include <OpenSim/Simulation/Model/AnalysisSet.h>
#include <OpenSim/Tools/RRATool.h>
#include <OpenSim/Tools/ForwardTool.h>
#include <OpenSim/Analyses/Kinematics.h>
#include <OpenSim/Analyses/Actuation.h>
#include <OpenSim/Analyses/PointKinematics.h>
#include <OpenSim/Analyses/BodyKinematics.h>

using namespace OpenSim;
using namespace std;

#define ASSERT(cond) {if (!(cond)) throw(exception());}


void testGait2354() {

	RRATool rra("subject01_Setup_RRA.xml");
	rra.run();

    SimTK::Vec3 com; 
    
    // compare the adjusted center of mass to OpenSim 1.9.1 values
    Model adjusted_model("subject01_RRA_adjusted.osim");
    const BodySet& bodies = adjusted_model.getBodySet();
    const Body& torso = bodies.get(bodies.getIndex("torso"));
    torso.getMassCenter(com);
    cout << "Center of Mass = " << com << endl;
    cout << "Center of Mass 19= -0.00598028440188985017, 0.0, 0.1" <<  endl;
	ASSERT(std::abs(com[0]-0.00598028440188985017) < .0001 );
	ASSERT(std::abs(com[2]-0.1) < .0001 );
    cout << "torso com =" << com << endl;
    
	Storage results("ResultsRRA/subject01_walk1_RRA_Kinematics_q.sto");
	Storage standard("subject01_walk1_RRA_Kinematics_q_standard.sto");

	// Compare the output file to the target trajectory and make sure they
	// are sufficiently close.

	double meanError = 0.0;
	int count = 0;
	Array<double> data;
    std::string  label;

	for (int i = 0; i < results.getSize(); ++i) {
		StateVector* state = results.getStateVector(i);
		double time = state->getTime();

		data.setSize(state->getSize());
		standard.getDataAtTime(time, state->getSize(), data);
		for (int j = 0; j < state->getSize(); ++j) {
            label = results.getColumnLabels()[j+1];

            double diff = data[standard.getStateIndex(label)]-state->getData()[j];
    	    meanError += std::abs(diff);
			count++;
		}
	}
    printf("meanError = %f  count = %d meanError/count=%f \n", meanError, count, meanError/count);
	ASSERT(meanError/count < 0.5);
}
int main() {
    try {
		testGait2354();
    }
    catch(const std::exception& e) {
        cout << "exception: " << e.what() << endl;
        return 1;
    }
    cout << "Done" << endl;
    return 0;
}
