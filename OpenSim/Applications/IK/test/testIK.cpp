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
#include <OpenSim/Tools/IO.h>
#include <OpenSim/Simulation/SIMM/SimmModel.h>
#include <OpenSim/Simulation/SIMM/SimmKinematicsEngine.h>
#include <OpenSim/Simulation/SIMM/SimmMarkerSet.h>
#include <OpenSim/Simulation/SIMM/SimmSubject.h>
#include <OpenSim/Simulation/SIMM/SimmMarkerData.h>
#include <OpenSim/Simulation/SIMM/SimmMotionData.h>
#include <OpenSim/Applications/IK/SimmIKSolverImpl.h>
#include <OpenSim/Applications/Scale/SimmScalerImpl.h>
#include <OpenSim/Applications/IK/SimmInverseKinematicsTarget.h>



using namespace OpenSim;
using namespace std;

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

	// Construct model and read parameters file
	SimmSubject* subject = new SimmSubject("subject_ik_setup.xml");
	SimmModel* model = subject->createModel();
	model->setup();

	//--------------------- IK proper section
	{
		try 
		{	
			/* CLAY- Commented out until we fix, so things would compile...

			// Update markers to correspond to those specified in IKParams block, potentially adding new ones
			model->updateMarkers(subject->getIKParams().getMarkerSet());
			// Initialize coordinates based on user input
			model->updateCoordinates(subject->getIKParams().getCoordinateSet());

			// Get trial params
			SimmIKTrialParams& trialParams = subject->getIKParams().getTrialParams(0);
			// Handle coordinates file
			SimmMotionData* coordinateValues = trialParams.getCoordinateValues(*model);

			// Setup IK problem for trial
			// We need SimmInverseKinematicsTarget, iksolver (SimmIKSolverImpl)
			// Create SimmMarkerData Object from trc file of experimental motion data
			SimmMarkerData motionTrialData(trialParams.getMarkerDataFilename());
			motionTrialData.convertToUnits(model->getLengthUnits());
			Storage inputStorage;
			// Convert read trc fil into "common" rdStroage format
			motionTrialData.makeRdStorage(inputStorage);

			if (coordinateValues != 0) {
				// Adjust the user-defined start and end times to make sure they are in the
				// range of the marker data. This must be done so that you only look in the
				// coordinate data for rows that will actually be solved.
				double firstStateTime = inputStorage.getFirstTime();
				double lastStateTime = inputStorage.getLastTime();
				double startTime = (firstStateTime>trialParams.getStartTime()) ? firstStateTime : trialParams.getStartTime();
				double endTime =  (lastStateTime<trialParams.getEndTime()) ? lastStateTime : trialParams.getEndTime();

				// Add the coordinate data to the marker data. There must be a row of
				// corresponding coordinate data for every row of marker data that will
				// be solved, or it is a fatal error.
				coordinateValues->addToRdStorage(inputStorage, startTime, endTime);
			}

			// Create target
			SimmInverseKinematicsTarget *target = new SimmInverseKinematicsTarget(*model, inputStorage);
			// Create solver
			SimmIKSolverImpl *ikSolver = new SimmIKSolverImpl(*target);
			// Solve
			Storage	outputStorage; 
			ikSolver->solveFrames(trialParams, inputStorage, outputStorage);

			OpenSim::IO::SetPrecision(6);
			outputStorage.setWriteSIMMHeader(true);
			outputStorage.print(trialParams.getOutputMotionFilename().c_str());

			delete coordinateValues;
			delete ikSolver;
			delete target;

			CLAY */
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
}

