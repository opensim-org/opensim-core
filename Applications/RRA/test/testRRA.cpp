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
#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>

using namespace OpenSim;
using namespace std;

void checkCOM(string resultsFile, string body, const SimTK::Vec3 &standardCOM, const Array<double> &tolerances);

int main() {
    try {
		RRATool rra("subject01_Setup_RRA.xml");
		rra.run();
		checkCOM("subject01_RRA_adjusted.osim", "torso", SimTK::Vec3(0.00598028440188985017, 0.34551, 0.1), Array<double>(1e-4, 3));
		Storage result("ResultsRRA/subject01_walk1_RRA_Kinematics_q.sto"), standard("subject01_walk1_RRA_Kinematics_q_standard.sto");
		CHECK_STORAGE_AGAINST_STANDARD(result, standard, Array<double>(0.5, 24),"",-1,"Exception");
    }
    catch (const Exception& e) {
        e.print(cerr);
        return 1;
    }
    cout << "Done" << endl;
    return 0;
}

void checkCOM(string resultsFile, string body, const SimTK::Vec3 &standardCOM, const Array<double> &tolerances) {

	// compare the adjusted center of mass to OpenSim 1.9.1 values
	Model adjusted_model(resultsFile);
	const BodySet& bodies = adjusted_model.getBodySet();
	const Body& torso = bodies.get(bodies.getIndex(body));
	SimTK::Vec3 com;
	torso.getMassCenter(com);
	cout << "body:           " << body << endl;
	cout << "center of mass: (" << com[0] << ", " << com[1] << ", " << com[2] << ")\n";
	cout << "standard COM:   (" << standardCOM[0] << ", " << standardCOM[1] << ", " << standardCOM[2] << ")\n";
	cout << "tolerances:     (" << tolerances[0] << ", " << tolerances[1] << ", " << tolerances[2] << ")\n" << endl;
	for (int i = 0; i < 3; ++i)
		ASSERT_EQUAL(standardCOM[i], com[i], tolerances[i]);
}
