// ik.cpp
// Author: Peter Loan
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
#include <OpenSim/Simulation/SIMM/SimmModel.h>
#include <OpenSim/Simulation/SIMM/SimmMarkerSet.h>
#include <OpenSim/Simulation/SIMM/SimmSubject.h>
#include <OpenSim/Simulation/SIMM/SimmMarkerData.h>
#include <OpenSim/Simulation/SIMM/SimmMotionData.h>
#include <OpenSim/Applications/Scale/SimmScalerImpl.h>
#include "SimmIKSolverImpl.h"
#include "SimmInverseKinematicsTarget.h"



using namespace OpenSim;
using namespace std;

static void PrintUsage(ostream &aOStream);
//______________________________________________________________________________
/**
 * Test program to read SIMM model elements from an XML file.
 *
 * @param argc Number of command line arguments (should be 1).
 * @param argv Command line arguments:  simmReadXML inFile
 */
int main(int argc,char **argv)
{
	// PARSE COMMAND LINE
	string inName;
	string option = "";
	if (argc < 2)
	{
		PrintUsage(cout);
		exit(-1);
	}
	else {		// Don't know maybe user needs help or have special needs
		int i;
		for(i=1;i<=(argc-1);i++) {
			option = argv[i];

			// PRINT THE USAGE OPTIONS
			if((option=="-help")||(option=="-h")||(option=="-Help")||(option=="-H")||
				(option=="-usage")||(option=="-u")||(option=="-Usage")||(option=="-U")) {
					PrintUsage(cout);
					return(0);
					// IDENTIFY SETUP FILE
				} else if((option=="-Setup")||(option=="-S")) {
					inName = argv[i+1];
					break;
				}
				else if((option=="-PrintSetup")||(option=="-PS")) {
					SimmSubject *subject = new SimmSubject();
					subject->setName("default");
					// Add in useful objects that may need to be instantiated
					Object::setSerializeAllDefaults(true);
					subject->print("default_subject.xml");
					Object::setSerializeAllDefaults(false);
					cout << "Created file default_subject.xml with default setup" << endl;
					return(0); 
				}
				else {
					cout << "Unrecognized option" << option << "on command line... Ignored" << endl;
					PrintUsage(cout);
					return(0);
				}
		}
	}
	// Construct model and read parameters file
	SimmSubject* subject = new SimmSubject(inName);
	SimmModel* model = subject->createModel();

	if (!subject->isDefaultIKParams()){
		//  If model needs to be created anew, do it here.
		if (subject->getIKParams().getModelFileName()!="Unassigned"){
			delete model;
			model = new SimmModel(subject->getIKParams().getModelFileName());
			model->setup();
		}
		else // Warn model used without scaling or marker placement
			cout << "Inverse kinematics model name not set. Using " << inName << " as model file "<< endl;

		// Update markers to correspond to those specified in IKParams block
		model->updateMarkers(subject->getIKParams().getMarkerSet());
		// Initialize coordinates based on user input
		model->updateCoordinates(subject->getIKParams().getCoordinateSet());
		/* Now perform the IK trials on the updated model. */
		for (int i = 0; i < subject->getIKParams().getNumIKTrials(); i++)
		{
			// Get trial params
			SimmIKTrialParams& trialParams = subject->getIKParams().getTrialParams(i);
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
				/* Adjust the user-defined start and end times to make sure they are in the
				* range of the marker data. This must be done so that you only look in the
				* coordinate data for rows that will actually be solved.
				*/
				double firstStateTime = inputStorage.getFirstTime();
				double lastStateTime = inputStorage.getLastTime();
				double startTime = max<double>(firstStateTime, trialParams.getStartTime());
				double endTime = min<double>(lastStateTime, trialParams.getEndTime());

				/* Add the coordinate data to the marker data. There must be a row of
				* corresponding coordinate data for every row of marker data that will
				* be solved, or it is a fatal error.
				*/
				coordinateValues->addToRdStorage(inputStorage, startTime, endTime);
			}
			// Create target
			SimmInverseKinematicsTarget *target = new SimmInverseKinematicsTarget(*model, inputStorage);
			// Create solver
			SimmIKSolverImpl *ikSolver = new SimmIKSolverImpl(*target, subject->getIKParams());
			// Solve
			Storage	outputStorage;
			ikSolver->solveFrames(trialParams, inputStorage, outputStorage);
			outputStorage.setWriteSIMMHeader(true);
			outputStorage.print(trialParams.getOutputMotionFilename().c_str());

			delete coordinateValues;
			delete ikSolver;
			delete target;
		}

	}
	else {
			cout << "Inverse kinematics parameters not set. IK was not solved." << endl;
	}
	delete subject;

}
//_____________________________________________________________________________
/**
 * Print the usage for this application
 */
void PrintUsage(ostream &aOStream)
{
	aOStream<<"\n\nik.exe:\n\n";
	aOStream<<"Option             Argument         Action / Notes\n";
	aOStream<<"------             --------         --------------\n";
	aOStream<<"-Help, -H                           Print the command-line options for scale.exe.\n";
	aOStream<<"-PrintSetup, -PS                    Generates a template Setup file to customize the scaling\n";
	aOStream<<"-Setup, -S         SetupFile        Specify an xml setupfile that specifies an OpenSim model,\n";
	aOStream<<"                                    a marker file, and scaling parameters.\n";
}
	
