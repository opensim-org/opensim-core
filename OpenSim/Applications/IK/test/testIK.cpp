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
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Common/ScaleSet.h>
#include <OpenSim/Tools/ScaleTool.h>
#include <OpenSim/Simulation/Model/AbstractModel.h>

using namespace std;
using namespace OpenSim;

string filesToCompare[] = {
	"subject_trial_ik.mot"
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
// Eran: comment out everything for now because we now use InvestigationIK
#if 0
	// Construct model and read parameters file
	Object::RegisterType(ScaleTool());
	ScaleTool::registerTypes();
	ScaleTool* subject = new ScaleTool("subject_ik_setup.xml");
	AbstractModel* model = subject->createModel();

	//----------------------- Model scaling section
	if (!subject->isDefaultModelScaler())
	{
		ModelScaler& scaler = subject->getModelScaler();
		scaler.processModel(model, subject->getPathToSubject(), subject->getMass());
	}
	else
	{
		cout << "ModelScaler parameters have not been defined. The generic model will not be scaled." << endl;
	}

	//----------------------- Marker placement section
	if (!subject->isDefaultMarkerPlacer())
	{
		MarkerPlacer& placer = subject->getMarkerPlacer();
		placer.processModel(model, subject->getPathToSubject());
	}
	else
	{
		cout << "MarkerPlacer parameters have not been defined. No markers have been moved." << endl;
	}

	//--------------------- IK proper section
	{
		try 
		{
			if (!subject->isDefaultIKSolver())
			{
				SimmIKSolver& solver = subject->getIKSolver();
				solver.processModel(model, subject->getPathToSubject());
			}
			else
			{
				cout << "IK Solver parameters not set. No IK has been performed." << endl;
			}
		}
		catch (Exception &x)
		{
			x.print(cout);
			cout << "Press Return to continue." << endl;
			cout.flush();
			int c = getc( stdin );
			return 1;
		}
	}
	delete subject;

	/* Compare results with standard*/
	bool success = true;
	for (int i=0; i < 1 && success; i++){
		string command = "cmp "+filesToCompare[i]+" "+"std_"+filesToCompare[i];
		success = success && (system(command.c_str())==0);
	}

	return (success?0:1);
#endif
}

