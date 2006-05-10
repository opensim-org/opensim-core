// scale.cpp
// Author: Ayman Habib
/* Copyright (c) 2005, Stanford University and Ayman Habib.
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
#include <OpenSim/Tools/IO.h>
#include <OpenSim/Tools/XMLDocument.h>
#include <OpenSim/Tools/VisibleProperties.h>
#include <OpenSim/Tools/ScaleSet.h>
#include <OpenSim/Simulation/SIMM/SimmModel.h>
#include <OpenSim/Simulation/SIMM/SimmMuscle.h>
#include <OpenSim/Actuators/LinearSetPoint.h>
#include <OpenSim/Simulation/SIMM/SimmKinematicsEngine.h>
#include <OpenSim/Simulation/SIMM/SimmMarkerSet.h>
#include <OpenSim/Simulation/SIMM/SimmSubject.h>
#include <OpenSim/Simulation/SIMM/SimmMarkerData.h>
#include <OpenSim/Simulation/SIMM/SimmMotionData.h>

#include <OpenSim/Simulation/Model/Analysis.h>
#include <OpenSim/Applications/IK/SimmIKSolverImpl.h>
#include "SimmScalerImpl.h"
#include <OpenSim/Applications/IK/SimmInverseKinematicsTarget.h>



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
				} else if((option=="-S")||(option=="-Setup")) {
					if (argv[i+1]==0){
						PrintUsage(cout);
						return(0);
					}
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
					cout << "Unrecognized option " << option << " on command line... Ignored" << endl;
					PrintUsage(cout);
					return(0);
				}
		}
	}
	try {
		// Construct model and read parameters file
		SimmSubject* subject = new SimmSubject(inName);
		SimmModel* model = subject->createModel();
		if (!subject->isDefaultScalingParams()){
			SimmScalingParams& params = subject->getScalingParams();
			ScalerInterface *scaler = new SimmScalerImpl(*model);
			if (!scaler->scaleModel(params.getScaleSet(*model), params.getPreserveMassDist(), subject->getMass()))
			{
				cout << "===ERROR===: Unable to scale generic model." << endl;
				return 0;
			}
			else {
				cout << "Scaled model "<< inName << "Successfully" << endl;
			}
			params.writeOutputFiles(model);
			delete scaler;
		}
		else {
			cout << "Scaling parameters not set. Model is not scaled." << endl;
		}
		if (!subject->isDefaultMarkerPlacementParams()){
			SimmMarkerPlacementParams& params = subject->getMarkerPlacementParams();
			// Update markers to correspond to those specified in IKParams block
			model->updateMarkers(params.getMarkerSet());

			/* Load the static pose marker file, and average all the
			* frames in the user-specified time range.
			*/
			SimmMarkerData staticPose(params.getStaticPoseFilename());
			// Convert read trc fil into "common" rdStroage format
			Storage inputStorage;
			staticPose.makeRdStorage(inputStorage);
			// Convert the marker data into the model's units.
			double startTime, endTime;
			params.getTimeRange(startTime, endTime);
			staticPose.averageFrames(0.01, startTime, endTime);
			staticPose.convertToUnits(model->getLengthUnits());

			/* Delete any markers from the model that are not in the static
			* pose marker file.
			*/
			model->deleteUnusedMarkers(staticPose.getMarkerNames());

			/* Now solve the static pose. */
			SimmIKTrialParams options;
			options.setStartTime(startTime);
			options.setEndTime(endTime);
			options.setIncludeMarkers(true);

			// Convert read trc fil into "common" rdStroage format
			staticPose.makeRdStorage(inputStorage);
			// Create target
			SimmInverseKinematicsTarget *target = new SimmInverseKinematicsTarget(*model, inputStorage);
			// Create solver
			SimmIKSolverImpl *ikSolver = new SimmIKSolverImpl(*target, subject->getIKParams());
			// Solve
			Storage	outputStorage;
			ikSolver->solveFrames(options, inputStorage, outputStorage);

			model->moveMarkersToCloud(staticPose);
	
			params.writeOutputFiles(model, outputStorage);

			delete ikSolver;
			delete target;
		}
		else {
			cout << "Marker placement parameters not set. No markers have been moved." << endl;
		}
		delete model;
		delete subject;
	}
	catch(Exception &x) {
		x.print(cout);
	}

}

//_____________________________________________________________________________
/**
* Print the usage for this application
*/
void PrintUsage(ostream &aOStream)
{
	aOStream<<"\n\nscale.exe:\n\n";
	aOStream<<"Option            Argument      Action / Notes\n";
	aOStream<<"------            --------      --------------\n";
	aOStream<<"-Help, -H                       Print the command-line options for scale.exe.\n";
	aOStream<<"-PrintSetup, -PS                Generates a template Setup file to customize scaling\n";
	aOStream<<"-Setup, -S        SetupFile     Specify an xml setup file that specifies an OpenSim model,\n";
	aOStream<<"                                a marker file, and scaling parameters.\n";
}
