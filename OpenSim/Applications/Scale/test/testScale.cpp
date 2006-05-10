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
	SimmSubject* subject = new SimmSubject("CrouchGait.xml");
	SimmModel* model = subject->createModel();
	SimmScalingParams& params = subject->getScalingParams();
	ScalerInterface *scaler = new SimmScalerImpl(*model);

	if (!scaler->scaleModel(params.getScaleSet(*model), params.getPreserveMassDist(), subject->getMass()))
	{
		cout << "===ERROR===: Unable to scale generic model." << endl;
		return 0;
	}
	params.writeOutputFiles(model);

	if (!subject->isDefaultMarkerPlacementParams()){
		SimmMarkerPlacementParams& markerPlacementParams = subject->getMarkerPlacementParams();
		// Update markers to correspond to those specified in IKParams block
		model->updateMarkers(markerPlacementParams.getMarkerSet());


		/**
		* Load the static pose marker file, and average all the
		* frames in the user-specified time range.
		*/
		SimmMarkerData staticPose(markerPlacementParams.getStaticPoseFilename());
		// Convert read trc fil into "common" rdStroage format
		Storage inputStorage;
		// Convert read trc fil into "common" rdStroage format
		staticPose.makeRdStorage(inputStorage);
		// Convert the marker data into the model's units.
		double startTime, endTime;
		markerPlacementParams.getTimeRange(startTime, endTime);
		staticPose.averageFrames(0.01, startTime, endTime);
		staticPose.convertToUnits(model->getLengthUnits());

		/* Delete any markers from the model that are not in the static
		* pose marker file.
		*/
		model->deleteUnusedMarkers(staticPose.getMarkerNames());

		/* Now solve the static pose, by faking it as an IKTrial */
		SimmIKTrialParams options;
		options.setStartTime(startTime);
		options.setEndTime(startTime);
		options.setIncludeMarkers(true);

		// Create target
		SimmInverseKinematicsTarget *target = new SimmInverseKinematicsTarget(*model, inputStorage);
		// Create solver
		SimmIKSolverImpl *ikSolver = new SimmIKSolverImpl(*target, subject->getIKParams());
		// Solve
		Storage	outputStorage;
		ikSolver->solveFrames(options, inputStorage, outputStorage);

		delete ikSolver;
		delete target;
	}
	else {
		cout << "Marker placement parameters not set. No markers have been moved." << endl;
	}
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
	
