// testStaticOptimization.cpp
// Author: Ayman Habib
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
#include <OpenSim/version.h>
#include <OpenSim/Common/IO.h>
#include <OpenSim/Common/LoadOpenSimLibrary.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/BodySet.h>
#include <OpenSim/Tools/AnalyzeTool.h>
#include <OpenSim/Analyses/StaticOptimization.h>


using namespace OpenSim;
using namespace std;

bool equalStorage(Storage& stdStorage, Storage& actualStorage, double tol)
{
	actualStorage.subtract(&stdStorage);
	bool equal = true;
	Array<double> dData(-SimTK::Infinity);
	for (int i=0; i <actualStorage.getSize() && equal; i++){
		dData = actualStorage.getStateVector(i)->getData();
		double dMax = -SimTK::Infinity;
		int worst=-1;
		for (int j=0; j<dData.getSize(); j++){
			if (fabs(dData[j]) > dMax) worst = j;
			dMax = std::max(dMax, fabs(dData[j]));
		}
		equal = (dMax <= tol);
		//cout << "i, col, colname, diff" << i << worst << stdStorage.getColumnLabels().get(worst) << dMax << endl;
	}
	return equal;
}

int testModel(std::string modelPrefix)
{
	try {

	// CONSTRUCT
	AnalyzeTool analyze(modelPrefix+"_Setup_StaticOptimization.xml");

	// PRINT MODEL INFORMATION
	Model& model = analyze.getModel();
	cout<<"-----------------------------------------------------------------------\n";
	cout<<"Loaded library\n";
	cout<<"-----------------------------------------------------------------------\n";
	cout<<"-----------------------------------------------------------------------\n\n";

	// RUN
	analyze.run();
	
	// also change output file names/path
	//----------------------------
	// Catch any thrown exceptions
	//----------------------------
	} catch(Exception x) {
		x.print(cout);
		return(-1);
	}
	// Compare results to a standard (with muscle physiology)
	Storage currentResult("Results/"+modelPrefix+"_StaticOptimization_activation.sto");
	Storage stdStorage("std_"+modelPrefix+"_StaticOptimization_activation.sto");
	bool equal = equalStorage(stdStorage, currentResult, 1e-5);
	if (equal){
		currentResult = Storage("Results/"+modelPrefix+"_StaticOptimization_force.sto");
		stdStorage = Storage("std_"+modelPrefix+"_StaticOptimization_force.sto");
		bool equal = equalStorage(stdStorage, currentResult, 1e-5);
	}
	return (equal?0:1);
}
int main(int argc,char **argv)
{
	if (testModel("arm26")!=0){	// Arm26 with no bounds (0 to 1.0)
		cout << " testStaticOptimization.testArm  FAILED " << endl;
		return (1);
	}
	if (testModel("arm26_bounds")!=0){	// Arm26 with bounds 0.01 to 1.0 same as versions prior to 2.2.1
		cout << " testStaticOptimization.testArm  with bounds FAILED " << endl;
		return (1);
	}
	return(0);
}

