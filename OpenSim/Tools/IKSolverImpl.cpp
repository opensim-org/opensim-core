#include <OpenSim/Common/rdMath.h>
#include <OpenSim/Common/Array.h>
#include <OpenSim/Common/Storage.h>
#include <OpenSim/SQP/rdFSQP.h>
#include "IKTrial.h"
#include <OpenSim/Simulation/Model/AbstractDynamicsEngine.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/IntegCallbackSet.h>
#include <OpenSim/Simulation/Model/AnalysisSet.h>
#include "IKSolverImpl.h"
#include "IKTarget.h"



using namespace OpenSim;
using namespace std;
//______________________________________________________________________________
/**
 * An implementation of the IKSolverInterface specific to simm classes/dynamicsEngine.
 *
 * @param aOptimizationTarget The target that IK will minimize
 * @param aIKTrial Parameters specified in input file to control IK.
 */
IKSolverImpl::
IKSolverImpl(IKTarget& aOptimizationTarget):
IKSolverInterface(aOptimizationTarget)
{
}
//______________________________________________________________________________
/**
 * This is the heart of the IK solver, this method solves a specific motion trial
 * given an input storage object for input, one for output.

 * @param aIKOptions Pass along to the method attributes specified by end-user in input file.
 * @param inputData Set of frames to solve packaged as a storage fle.
 * @param outputData the frames solved by the solver represented as a storage onbject.
 */
void IKSolverImpl::solveFrames(const IKTrial& aIKOptions, Storage& inputData, Storage& outputData)
{
	int i;

	// Instantiate the optimizer
	rdFSQP *optimizer = new rdFSQP(&_ikTarget);

	// Set optimization convergence criteria/tolerance
	// Enable some Debugging here if needed
	optimizer->setPrintLevel(0);
	optimizer->setConvergenceCriterion(1.0E-4);	// Error in markers of .01
	optimizer->setMaxIterations(1000);

	/* Get names for unprescribed Qs (ones that will be solved). */
	Array<string> unprescribedCoordinateNames;
   _ikTarget.getUnprescribedCoordinateNames(unprescribedCoordinateNames);

	/* Get names for prescribed Qs (specified in input file). */
	Array<string> prescribedCoordinateNames;
   _ikTarget.getPrescribedCoordinateNames(prescribedCoordinateNames);

	/* Get names for markers used for solving. */
	Array<string> markerNames;
	_ikTarget.getOutputMarkerNames(markerNames);

	string resultsHeader = "time\t";
	for (i = 0; i < unprescribedCoordinateNames.getSize(); i++)
		resultsHeader += unprescribedCoordinateNames[i] + "\t";

	for (i = 0; i < prescribedCoordinateNames.getSize(); i++)
		resultsHeader += prescribedCoordinateNames[i] + "\t";

	string markerComponentNames[] = {"_px", "_py", "_pz"};
	// Include markers for visual verification in SIMM
	if (aIKOptions.getIncludeMarkers())
	{
		for (i = 0; i < markerNames.getSize(); i++)
		{
			for (int j = 0; j < 3; j++)
			{
				resultsHeader += markerNames[i] + markerComponentNames[j] + "\t";
			}
		}
	}
	// User data (colummns that are in input storage file but not used by IK,
	// to be passed along for later processing.
	const Array<string> &inputColumnNames	 = inputData.getColumnLabelsArray();
	Array<int> userDataColumnIndices(0);
	string userHeaders;
	collectUserData(inputColumnNames, resultsHeader, userHeaders, userDataColumnIndices);
	resultsHeader += userHeaders;

	outputData.setColumnLabels(resultsHeader.c_str());

	// Set the lower and upper bounds on the unprescribed Q array
	// TODO: shouldn't have to search for coordinates by name
	AbstractDynamicsEngine &eng = _ikTarget.getModel().getDynamicsEngine();
	for (int i = 0; i < unprescribedCoordinateNames.getSize(); i++)
	{
		AbstractCoordinate* coord = eng.getCoordinateSet()->get(unprescribedCoordinateNames[i]);
		optimizer->setLowerBound(i, coord->getRangeMin());
		optimizer->setUpperBound(i, coord->getRangeMax());
	}

	// Main loop to set initial conditions and solve snapshots
	// At every step we use experimental data as a starting guess 
	Array<double> unprescribedQGuess(0.0, unprescribedCoordinateNames.getSize());	// Initial guess and work array
	Array<double> unprescribedQSol(0.0, unprescribedCoordinateNames.getSize());	// Solution array
	Array<double> experimentalMarkerLocations(0.0, markerNames.getSize() * 3);

	int startFrame = 0, endFrame = 1;

	/* Get the indices of the starting frame and the ending frame,
	 * based on the user-defined start/end times stored in
	 * the simmIKTrialOptions.
	 */
	aIKOptions.findFrameRange(inputData, startFrame, endFrame);

	if (endFrame - startFrame > 1)
		cout << "Solving frames " << startFrame + 1 << " to " << endFrame + 1 << " (time = " <<
		aIKOptions.getStartTime() << " to " << aIKOptions.getEndTime() << ")" << endl;

	for (int index = startFrame; index <= endFrame; index++)
	{
		// Get time associated with index
		double timeT = inputData.getStateVector(index)->getTime();

		// Set value for prescribed coordinates and get initial guess for unprescribed coordinates
		_ikTarget.prepareToSolve(index, &unprescribedQGuess[0]);

		// Invoke optimization mechanism to solve for Qs
		int optimizerReturn = optimizer->computeOptimalControls(&unprescribedQGuess[0], &unprescribedQSol[0]);

		/* Output variables include unprescribed (solved) Qs... */
		Array<double> qsAndMarkersArray = unprescribedQSol;

		/* ... then prescribed Qs... */
		Array<double> prescribedCoordinateValues(0.0);
		_ikTarget.getPrescribedCoordinateValues(prescribedCoordinateValues);
		qsAndMarkersArray.append(prescribedCoordinateValues);

		/* ... then, optionally, computed marker locations. */
		if (aIKOptions.getIncludeMarkers())
		{
			_ikTarget.getExperimentalMarkerLocations(experimentalMarkerLocations);
			qsAndMarkersArray.append(experimentalMarkerLocations);
		}

		double currentTime;
		inputData.getTime(index, currentTime);
		cout << "Frame " << index + 1 << " (t=" << currentTime << "):\t";
		if(optimizerReturn != 0) cout << " Optimizer returned = " << optimizerReturn << endl;
		else _ikTarget.printPerformance();

		// INTEGRATION CALLBACKS
		// TODO: pass callback a reasonable "dt" value
		double emptyX, emptyY;
		IntegCallbackSet *callbackSet = _ikTarget.getModel().getIntegCallbackSet();
		if(callbackSet!=NULL)
			callbackSet->step(&emptyX,&emptyY,index-startFrame-1,0,currentTime,&emptyX,&emptyY);

		// ANALYSES
		// TODO: pass callback a reasonable "dt" value
		AnalysisSet *analysisSet = _ikTarget.getModel().getAnalysisSet();
		if(analysisSet!=NULL)
			analysisSet->step(&emptyX,&emptyY,index-startFrame-1,0,currentTime,&emptyX,&emptyY);

		// Append user data to qsAndMarkersArray
		Array<double> dataRow(qsAndMarkersArray);
		appendUserData(dataRow, userDataColumnIndices, inputData.getStateVector(index));

		// Allocate new row (StateVector) and add it to outputData
		StateVector *nextDataRow = new StateVector();
		nextDataRow->setStates(timeT, dataRow.getSize(), &dataRow[0]);
		outputData.append(*nextDataRow);
	}

	delete optimizer;
}
//______________________________________________________________________________
/**
 * UserData is the set of columns that are not used directly by the IK problem (for example
 * ground reaction forces). This UserData needs to be carried along in IK.
 */
void IKSolverImpl::collectUserData(const Array<string> &inputColumnLabels,
									   string& resultsHeader, 
									   string& userHeaders, 
									   Array<int>& userDataColumnIndices)
{
	// Find columns that are none of the above to append them to the end of the list
	int i;
	for(i=0; i< inputColumnLabels.getSize(); i++){
		string nextLabel = inputColumnLabels[i];
		// We'll find out if the column is already there by doing string search for 
		// either \t$column or $column\t to account for substrings
		string searchString1(nextLabel+"\t");
		string searchString2("\t"+nextLabel);
		if (strstr(resultsHeader.c_str(), searchString1.c_str())==0 &&
			strstr(resultsHeader.c_str(), searchString2.c_str())==0 ){
			// Append to userHeaders
			userHeaders += nextLabel+"\t";
			// Keep track of indices to match data with labels
			userDataColumnIndices.append(i);

			// cout << "Column:" << nextLabel << " index" << i << endl;
		}
	}
}
//______________________________________________________________________________
/**
 * Companion helper function to (collectUserData) responsible for appending user columns 
 * to outputRow.
 */
void IKSolverImpl::appendUserData(Array<double>& outputRow, Array<int>& indices, StateVector* inputRow)
{
	int i;
	for(i=0; i< indices.getSize(); i++){
		double userValue=rdMath::NAN;
		int index = indices[i]-1;
		inputRow->getDataValue(index, userValue);
		outputRow.append(userValue);
	}
}
