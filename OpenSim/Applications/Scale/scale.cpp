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
#include <OpenSim/Simulation/SIMM/SimmCoordinate.h>
#include <OpenSim/Simulation/Model/Analysis.h>
#include <OpenSim/Applications/IK/SimmIKSolverImpl.h>
#include "SimmScalerImpl.h"
#include <OpenSim/Applications/IK/SimmInverseKinematicsTarget.h>


using namespace std;
using namespace OpenSim;


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
	// SET OUTPUT FORMATTING
	IO::SetDigitsPad(4);

	// PARSE COMMAND LINE
	string inName;
	string option = "";
	if (argc < 2) {
		PrintUsage(cout);
		exit(-1);
	} else {
		int i;
		for(i=1;i<=(argc-1);i++) {
			option = argv[i];

			// PRINT THE USAGE OPTIONS
			if((option=="-help")||(option=="-h")||(option=="-Help")||(option=="-H")||
				(option=="-usage")||(option=="-u")||(option=="-Usage")||(option=="-U")) {
					PrintUsage(cout);
					return(0);

				// Identify the setup file
				} else if((option=="-S")||(option=="-Setup")) {
					if (argv[i+1]==0){
						PrintUsage(cout);
						return(0);
					}
					inName = argv[i+1];
					break;

				// Print a default setup file
				} else if((option=="-PrintSetup")||(option=="-PS")) {
					SimmSubject *subject = new SimmSubject();
					subject->setName("default");
					// Add in useful objects that may need to be instantiated
					Object::setSerializeAllDefaults(true);
					// Add instances of objects that matter as examples
					SimmGenericModelParams& params  = subject->getGenericModelParams();
					params.addMarker(new SimmMarker());
					// Add a measurement
					SimmScalingParams& params2 = subject->getScalingParams();
					SimmMeasurement* meas = new SimmMeasurement();
					meas->addBodyScale(new BodyScale());
					meas->addMarkerPair(new SimmMarkerPair());
					meas->setApply(false);
					params2.addMeasurement(meas);

					params2.addScale(new Scale());

					// Now do SimmMarkerPlacementParams 
					SimmMarkerPlacementParams& params3 = subject->getMarkerPlacementParams();
					params3.getMarkerSet().append(new SimmMarker());
					params3.addCoordinate(new SimmCoordinate());

					// do SimmIKParams
					SimmIKParams& param4 = subject->getIKParams();
					param4.getMarkerSet().append(new SimmMarker());

					SimmCoordinate* aCoordinate = new SimmCoordinate();
					aCoordinate->setRestraintFunction(new NatCubicSpline());
					param4.getCoordinateSet().append(aCoordinate);

					subject->print("default_subject.xml");
					Object::setSerializeAllDefaults(false);
					cout << "Created file default_subject.xml with default setup" << endl;
					return(0);

				// Unrecognized
				} else {
					cout << "Unrecognized option " << option << " on command line... Ignored" << endl;
					PrintUsage(cout);
					return(0);
				}
		}
	}


	try {

		// CONSTRUCT SUBJECT INSTANCE
		SimmSubject *subject = new SimmSubject(inName);
		Object *subjectCopy = subject->copy();
		subjectCopy->print("test_subject.xml");

		// CONSTRUCT THE MODEL
		SimmModel *model = subject->createModel();

		// WRITE MODEL TO FILE
		Object *modelCopy = model->copy();
		modelCopy->print("gait_test.osim");

		// SCALE THE MODEL BASE ON PARAMETERS SPECIFIED IN THE SUBJECT FILE
		if (!subject->isDefaultScalingParams()){
			SimmScalingParams& params = subject->getScalingParams();
			ScalerInterface *scaler = new SimmScalerImpl(*model);
			const ScaleSet &scaleSet = params.getScaleSet(*model);
			bool preserveMassDistribution = params.getPreserveMassDist();
			double mass = subject->getMass();
			bool success = scaler->scaleModel(scaleSet,preserveMassDistribution, mass);
			if (!success) {
				cout << "===ERROR===: Unable to scale generic model." << endl;
				return -1;
			} else {
				cout << "Scaled model "<< inName << "Successfully" << endl;
			}
			params.writeOutputFiles(model);
			delete scaler;

		// NO SCALING PARAMETERS SET
		} else {
			cout << "Scaling parameters not set. Model is not scaled." << endl;
		}


		// ADJUST MARKERS TO AGREE WITH THE STATIC TRIAL
		if (!subject->isDefaultMarkerPlacementParams()){

			SimmMarkerPlacementParams& params = subject->getMarkerPlacementParams();
			// Update markers to correspond to those specified in IKParams block
			model->updateMarkers(params.getMarkerSet());

			// begin code restore  (What is a code restore?)
			SimmMotionData coordinateValues(params.getCoordinateFileName());

			// For each coordinate whose "value" field the user specified
			// as "fromFile", read the value from the first frame in the
			// coordinate file (a SIMM motion file) and use it to overwrite
			// the "fromFile" specification.
			ArrayPtrs<SimmCoordinate> &coordinateSet = params.getCoordinateSet();
			if (coordinateValues.getNumColumns() > 0) {
				for (int i = 0; i < coordinateSet.getSize(); i++) {
					if (coordinateSet[i]->getValueStr() == "fromFile"){
						double newValue = coordinateValues.getValue(coordinateSet[i]->getName(), 0);
						coordinateSet[i]->setValue(newValue);
					}
				}
				// Update the model with the coordinates specified
				// by the user in the params section.
				model->updateCoordinates(coordinateSet);
			}
			// end code restore

			// Load the static pose marker file, and average all the
			// frames in the user-specified time range.
			SimmMarkerData staticPose(params.getStaticPoseFilename());

			// Convert read trc fil into "common" rdStroage format
			Storage inputStorage;
			staticPose.makeRdStorage(inputStorage);

			// Convert the marker data into the model's units.
			double startTime, endTime;
			params.getTimeRange(startTime, endTime);
			staticPose.averageFrames(0.01, startTime, endTime);
			staticPose.convertToUnits(model->getLengthUnits());

			// Delete any markers from the model that are not in the static
			// pose marker file.
			model->deleteUnusedMarkers(staticPose.getMarkerNames());

			// SOLVE THE IK PROBLEM FOR THE STATIC POSE
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

			// MOVE THE MARKERS TO CORRESPOND TO EXPERIMENTAL LOCATIONS
			model->moveMarkersToCloud(outputStorage);
	
			// WRITE THE FILES
			params.writeOutputFiles(model, outputStorage);

			delete ikSolver;
			delete target;

		// DO NOT MOVE MARKERS
		} else {
			cout << "Marker placement parameters not set. No markers have been moved." << endl;
		}

		// CLEAN UP
		//delete model;
		delete subject;

	// HANDLE ANY EXCEPTIONS
	} catch(Exception &x) {
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
