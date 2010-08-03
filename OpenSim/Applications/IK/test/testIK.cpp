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
#include <OpenSim/Tools/IKTool.h>
#include <OpenSim/Tools/InverseKinematicsTool.h>
#include <OpenSim/Tools/IKTrialSet.h>

using namespace std;
using namespace OpenSim;

bool equalStorage(Storage& stdStorage, Storage& actualStorage, double tol)
{
	double dMax = -SimTK::Infinity;

	const Array<std::string> &col_names = actualStorage.getColumnLabels();
	
	bool equal = false;
	double error = 0;
	double max_error = 0;


	for(int i=1; i<col_names.getSize(); i++){
		error = actualStorage.compareColumn(stdStorage, col_names[i], actualStorage.getFirstTime());
		max_error = error > max_error ? error : max_error;
	}

	equal = (max_error <= tol);
		
	return equal;
}

bool testInverseKinematicsGait2354()
{
	// read setup file and construct model
	InverseKinematicsTool* tool = new InverseKinematicsTool("subject01_Setup_InverseKinematics.xml");
	try {
		tool->run();
		Storage actualOutput(tool->getOutputMotionFileName());
		Storage stdStorage("std_subject01_walk1_ik.mot");

		// Check that we can match the input kinematics to within a fifth of a degree
		bool equal = equalStorage(stdStorage, actualOutput, 0.2);
		std::cout << (equal?"Success":"Failure") << endl;
		
		return equal;
	}
	catch(const std::exception& e) {
        cout << "exception: " << e.what() << endl;
        return false;
    }

	return true;
}

bool testInverseKinematicsUWDynamic()
{
	// read setup file and construct model
	InverseKinematicsTool* tool = new InverseKinematicsTool("uwdynamic_setup_ik.xml");
	try {
		tool->run();
	}
	catch(const std::exception& e) {
        cout << "exception: " << e.what() << endl;
        return false;
    }

	tool->print("ik_setup_test.xml");

	return true;
}

//______________________________________________________________________________
/**
* Test program to read test IK.
*
*/
int main()
{
	if(!testInverseKinematicsGait2354()){
		cout << "testInverseKinematicsGait2354 Failed." << endl;
		//return 1;
	}

	if(!testInverseKinematicsUWDynamic()){
		cout << "testInverseKinematicsUWDynamic Failed." << endl;
		return 1;
	}

	// Construct model and read parameters file
	IKTool* tool = new IKTool("subject01_Setup_IK.xml");
	Model& model = tool->getModel();

    SimTK::State& s = model.initSystem();
    model.getSystem().realize(s, SimTK::Stage::Position );
	tool->run();
	Storage *actualOutput = tool->getIKTrialSet()[0].getOutputStorage();
	Storage stdStorage("std_subject_trial_ik.mot");
	bool equal = equalStorage(stdStorage, *actualOutput, 5e-2);
	std::cout << (equal?"Success":"Failure") << endl;
	delete tool;
	return (equal?0:1);
}

