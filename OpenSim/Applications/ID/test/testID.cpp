// testID.cpp
// Author: Ayman Habib, Ajay Seth 
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
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/ForceSet.h>
#include <OpenSim/Tools/InverseDynamicsTool.h>


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
		//if(!equal)
			cout << "row= " << i << ", worst name: "<< stdStorage.getColumnLabels().get(worst+1) << ", max diff= "<<dMax << endl;
	}
	return equal;
}

int testModel(std::string modelPrefix, double tol)
{
	try {

	// CONSTRUCT
	InverseDynamicsTool id(modelPrefix+"_Setup_InverseDynamics.xml");

	cout<<"-----------------------------------------------------------------------\n";
	cout<<"Starting inverse dynamics\n";
	cout<<"-----------------------------------------------------------------------\n";
	cout<<"-----------------------------------------------------------------------\n\n";

	// RUN
	id.run();

	//----------------------------
	// Catch any thrown exceptions
	//----------------------------
	} catch(Exception x) {
		x.print(cout);
		return(-1);
	}
	// Compare results to a standard 
	Storage currentResult("Results/"+modelPrefix+"_InverseDynamics.sto");
	Storage stdStorage("std_"+modelPrefix+"_InverseDynamics.sto");

	
	bool equal = equalStorage(stdStorage, currentResult, tol);
	return (equal?0:1);
}
int main(int argc,char **argv)
{
	if (testModel("arm26", 1e-2)!=0){
		cout << " testInverseDynamics.testArm  FAILED " << endl;
		return(1);
	}
	if (testModel("subject01", 2.0)!=0){
		cout << " testInverseDynamics.testGait  FAILED " << endl;
		return(1);
	}
	
	// old format setup file
	// keep backup of old setup file as it gets overwritten
	SimTK::Xml::Document doc("subject221_Setup_InverseDynamics.xml");
	doc.writeToFile("save_subject221_Setup_InverseDynamics.xml");
	if (testModel("subject221", 2.0)!=0){
		cout << " testInverseDynamics.subject 221 old setup  FAILED " << endl;
		SimTK::Xml::Document doc2("save_subject221_Setup_InverseDynamics.xml");
		doc2.writeToFile("subject221_Setup_InverseDynamics.xml");
		return(1);
	}
	SimTK::Xml::Document doc2("save_subject221_Setup_InverseDynamics.xml");
	doc2.writeToFile("subject221_Setup_InverseDynamics.xml");
	
	return(0);
}

