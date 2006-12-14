// testIK.cpp
// Author: Ayman Habib based on Peter Loan's version
/* Copyright (c) 2005, Stanford University and Peter Loan.
 * 
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including 
 * without limitation the rights to use, copy, modify, merge, publish, 
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject
 * to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included 
 * in all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */


// INCLUDES
#include <string>
#include <OpenSim/Tools/rdTools.h>
#include <OpenSim/Tools/Storage.h>
#include <OpenSim/Tools/ScaleSet.h>
#include <OpenSim/Simulation/SIMM/AbstractModel.h>
#include <OpenSim/Simulation/SIMM/MarkerSet.h>
#include <OpenSim/Subject/SimmSubject.h>
#include <OpenSim/Simulation/SIMM/SimmMarkerData.h>
#include <OpenSim/Simulation/SIMM/SimmMotionData.h>
#include <OpenSim/Applications/IK/SimmIKSolverImpl.h>
#include <OpenSim/Applications/IK/SimmInverseKinematicsTarget.h>

using namespace std;
using namespace OpenSim;

string filesToCompare[] = {
							"CrouchGaitSP.jnt",
							"CrouchGaitSP.msl",
							"CrouchGaitSP.xml",
							"CrouchGaitScale.xml"

};
//______________________________________________________________________________
/**
 * Test program to read SIMM model elements from an XML file.
 *
 * @param argc Number of command line arguments (should be 1).
 * @param argv Command line arguments:  simmReadXML inFile
 */
int main(int argc,char **argv)
{
	// Construct model and read parameters file
	//Object::RegisterType(VisibleObject());
	Object::RegisterType(SimmSubject());
	SimmSubject::registerTypes();
	SimmSubject* subject = new SimmSubject("CrouchGait.xml");
	AbstractModel* model = subject->createModel();
	if (!subject->isDefaultModelScaler())
	{
		SimmModelScaler& scaler = subject->getModelScaler();
		scaler.processModel(model, subject->getPathToSubject(), subject->getMass());
	}
	else
	{
		cout << "Scaling parameters not set. Model is not scaled." << endl;
	}

	if (!subject->isDefaultMarkerPlacer())
	{
		SimmMarkerPlacer& placer = subject->getMarkerPlacer();
		placer.processModel(model, subject->getPathToSubject());
	}
	else
	{
		cout << "Marker placement parameters not set. No markers have been moved." << endl;
	}

	delete model;
	delete subject;

	/* Compare results with standard*/
	bool success = true;
	for (int i=0; i < 4 && success; i++){
		string command = "cmp "+filesToCompare[i]+" "+"std_"+filesToCompare[i];
		success = success && (system(command.c_str())==0);
	}
	cout << "Path used = " << getenv("PATH") << endl;

	return (success?0:1);
}
	
