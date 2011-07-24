// testIK.cpp
// Author: Ayman Habib based on Peter Loan's version
/* Copyright (c)  2005, Stanford University and Peter Loan.
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


// INCLUDES
#include <string>
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Common/ScaleSet.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Tools/InverseKinematicsTool.h>
#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>

using namespace OpenSim;
using namespace std;

int main()
{
	try {
		InverseKinematicsTool ik1("subject01_Setup_InverseKinematics.xml");
		ik1.run();
		checkResultFiles(ik1.getOutputMotionFileName(), "std_subject01_walk1_ik.mot", Array<double>(0.2, 24), __FILE__, __LINE__, "testInverseKinematicsGait2354 failed");
		cout << "testInverseKinematicsGait2354 passed" << endl;

		InverseKinematicsTool ik2("subject01_Setup_InverseKinematics_NoModel.xml");
		Model mdl("subject01_simbody.osim");
		mdl.initSystem();
		ik2.setModel(mdl);
		ik2.run();
		checkResultFiles(ik2.getOutputMotionFileName(), "std_subject01_walk1_ik.mot", Array<double>(0.2, 24), __FILE__, __LINE__, "testInverseKinematicsGait2354 GUI workflow failed");
		cout << "testInverseKinematicsGait2354 GUI workflow passed" << endl;

		InverseKinematicsTool ik3("subjectOld_Setup_InverseKinematics.xml");
		ik3.run();
		checkResultFiles(ik3.getOutputMotionFileName(), "std_subject01_walk1_ik.mot", Array<double>(0.2, 24), __FILE__, __LINE__, "testInverseKinematicsGait2354 Old setup failed");
		cout << "testInverseKinematicsGait2354 Old setup passed" << endl;

		InverseKinematicsTool ik4("uwdynamic_setup_ik.xml");
		ik4.run();
		cout << "testInverseKinematicsUWDynamic passed" << endl;
	}
	catch (const Exception& e) {
        e.print(cerr);
        return 1;
    }
    cout << "Done" << endl;
    return 0;
}