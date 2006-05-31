#include <OpenSim/Tools/Array.h>
#include <OpenSim/Tools/Storage.h>
#include <OpenSim/SQP/rdFSQP.h>
#include <OpenSim/Simulation/SIMM/SimmIKTrialParams.h>
#include <OpenSim/Simulation/SIMM/SimmKinematicsEngine.h>
#include <OpenSim/Simulation/SIMM/SimmModel.h>
#include "SimmIKSolverImpl.h"
#include "SimmInverseKinematicsTarget.h"



using namespace OpenSim;
using namespace std;
//______________________________________________________________________________
/**
 * An implementation of the IKSolverInterface specific to simm classes/dynamicsEngine.
 *
 * @param aOptimizationTarget The target that IK will minimize
 * @param aIKParams Parameters specified in input file to control IK.
 */
SimmIKSolverImpl::
SimmIKSolverImpl(SimmInverseKinematicsTarget&	aOptimizationTarget,
				 const SimmIKParams&	aIKParams):
IKSolverInterface(aOptimizationTarget, aIKParams)
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
void SimmIKSolverImpl::solveFrames(const SimmIKTrialParams& aIKOptions, Storage& inputData, Storage& outputData)
{
	// Instantiate the optimizer
	rdFSQP *optimizer = new rdFSQP(&_ikTarget);

	// Set optimization convergence criteria/tolerance
	// Enable some Debugging here if needed
	optimizer->setPrintLevel(0);
	optimizer->setConvergenceCriterion(1.0E-4);	// Error in markers of .01
	optimizer->setMaxIterations(1000);

	/* Get names for unconstrained Qs (ones that will be solved). */
	Array<const string*> unconstrainedCoordinateNames(NULL);
   _ikTarget.getUnconstrainedCoordinateNames(unconstrainedCoordinateNames);

	/* Get names for prescribed Qs (specified in input file). */
	Array<const string*> prescribedCoordinateNames(NULL);
   _ikTarget.getPrescribedCoordinateNames(prescribedCoordinateNames);

	/* Get names for markers used for solving. */
	Array<const string*> markerNames(NULL);
	_ikTarget.getOutputMarkerNames(markerNames);

	string resultsHeader = "time\t";
	for (int i = 0; i < unconstrainedCoordinateNames.getSize(); i++)
	{
		resultsHeader += *(unconstrainedCoordinateNames[i]);
		resultsHeader += "\t";
	}

	for (int i = 0; i < prescribedCoordinateNames.getSize(); i++)
	{
		resultsHeader += *(prescribedCoordinateNames[i]);
		resultsHeader += "\t";
	}

	string markerComponentNames[] = {"_px", "_py", "_pz"};
	// Include markers for visual verification in SIMM
	if (aIKOptions.getIncludeMarkers())
	{
		for (int i = 0; i < markerNames.getSize(); i++)
		{
			for (int j = 0; j < 3; j++)
			{
				resultsHeader += *(markerNames[i]);
				resultsHeader += markerComponentNames[j];
				resultsHeader += "\t";
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

	// Set the lower and upper bounds on the unconstrained Q array
	// TODO: shouldn't have to search for coordinates by name
	SimmKinematicsEngine &eng = (SimmKinematicsEngine&)_ikTarget.getModel().getSimmKinematicsEngine();
	for (int i = 0; i < unconstrainedCoordinateNames.getSize(); i++)
	{
		Coordinate* coord = eng.getCoordinate(*(unconstrainedCoordinateNames[i]));
		optimizer->setLowerBound(i, coord->getRangeMin());
		optimizer->setUpperBound(i, coord->getRangeMax());
	}

	// Main loop to set initial conditions and solve snapshots
	// At every step we use experimental data as a starting guess 
	Array<double> unconstrainedQGuess(0.0, unconstrainedCoordinateNames.getSize());	// Initial guess and work array
	Array<double> unconstrainedQSol(0.0, unconstrainedCoordinateNames.getSize());	// Solution array
	Array<double> experimentalMarkerLocations(0.0, markerNames.getSize() * 3);

	int startFrame = 0, endFrame = 1;
	double currentTime;

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

		/* This sets the values of the prescribed coordinates, which
		 * are coordinates that are:
		 *   (a) specified in the input data, and
		 *   (b) locked at that specified value.
		 * These coordinates are not variables in the IK solving.
		 * If a coordinate is specified in the file but not
		 * locked, it is an unconstrained coordinate and it is
		 * variable in the IK solving.
		 */
		_ikTarget.setPrescribedCoordinates(index);

		// This sets the guess of unconstrained generalized coordinates 
		// and marker data from recordedDataStorage
		_ikTarget.setIndexToSolve(index, &unconstrainedQGuess[0]);

		// Invoke optimization mechanism to solve for Qs
		int optimizerReturn = optimizer->computeOptimalControls(&unconstrainedQGuess[0], &unconstrainedQSol[0]);

		/* Output variables include unconstrained (solved) Qs... */
		Array<double> qsAndMarkersArray = unconstrainedQSol;

		/* ... then prescribed Qs... */
		Array<double> prescribedQValues(0.0);
		_ikTarget.getPrescribedQValues(prescribedQValues);
		qsAndMarkersArray.append(prescribedQValues);

		/* ... then, optionally, computed marker locations. */
		if (aIKOptions.getIncludeMarkers())
		{
			_ikTarget.getExperimentalMarkerLocations(experimentalMarkerLocations);
			qsAndMarkersArray.append(experimentalMarkerLocations);
		}

		inputData.getTime(index, currentTime);
		cout << "Solved frame " << index + 1 << " at time " << currentTime << ", Optimizer returned = " << optimizerReturn << endl;
		// Allocate new row (StateVector) and add it to ikStorage
		StateVector *nextDataRow = new StateVector();

		// Append user data to qsAndMarkersArray
		Array<double> dataRow(qsAndMarkersArray);
		appendUserData(dataRow, userDataColumnIndices, inputData.getStateVector(index));

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
void SimmIKSolverImpl::collectUserData(const Array<string> &inputColumnLabels,
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
void SimmIKSolverImpl::appendUserData(Array<double>& outputRow, Array<int>& indices, StateVector* inputRow)
{
	int i;
	for(i=0; i< indices.getSize(); i++){
		double userValue=rdMath::NAN;
		int index = indices[i]-1;
		inputRow->getDataValue(index, userValue);
		outputRow.append(userValue);
	}
}