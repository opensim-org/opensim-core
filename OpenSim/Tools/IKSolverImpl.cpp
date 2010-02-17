/* Copyright (c)  2006 Stanford University
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
#include <OpenSim/Common/Array.h>
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Common/IO.h>
#include "IKTrial.h"
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/AnalysisSet.h>
#include <SimTKmath.h>
#include <simmath/Optimizer.h>
#include "IKSolverImpl.h"
#include "IKTarget.h"



using namespace OpenSim;
using namespace std;
using SimTK::Optimizer;
using SimTK::Vector;
//______________________________________________________________________________
/**
 * An implementation of the IKSolverInterface 
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
 * Initializes the columns of the output storage
 */
void IKSolverImpl::initializeSolver(const IKTrial& aIKOptions, Storage& inputData, Storage& outputData)
{
	/* Get names for unprescribed Qs (ones that will be solved). */
	Array<string> unprescribedCoordinateNames;
   _ikTarget.getUnprescribedCoordinateNames(unprescribedCoordinateNames);

	/* Get names for prescribed Qs (specified in input file). */
	Array<string> prescribedCoordinateNames;
   _ikTarget.getPrescribedCoordinateNames(prescribedCoordinateNames);

	/* Get names for markers used for solving. */
	Array<string> markerNames;
	_ikTarget.getOutputMarkerNames(markerNames);

	Array<string> resultColumnLabels;
	resultColumnLabels.append("time");
	for (int i = 0; i < unprescribedCoordinateNames.getSize(); i++)
		resultColumnLabels.append(unprescribedCoordinateNames[i]);

	for (int i = 0; i < prescribedCoordinateNames.getSize(); i++)
		resultColumnLabels.append(prescribedCoordinateNames[i]);

	// Include markers for visual verification in SIMM
	if (aIKOptions.getIncludeMarkers()) {
		for (int i = 0; i < markerNames.getSize(); i++) {
			resultColumnLabels.append(markerNames[i] + "_px");
			resultColumnLabels.append(markerNames[i] + "_py");
			resultColumnLabels.append(markerNames[i] + "_pz");
		}
	}
	// User data (colummns that are in input storage file but not used by IK,
	// to be passed along for later processing.
	_userDataColumnIndices.setSize(0);
	Array<string> userColumnLabels;
	collectUserData(inputData.getColumnLabels(), resultColumnLabels, userColumnLabels, _userDataColumnIndices);
	resultColumnLabels.append(userColumnLabels);

	outputData.setColumnLabels(resultColumnLabels);
}
//______________________________________________________________________________
/**
 * This is the heart of the IK solver, this method solves a specific motion trial
 * given an input storage object for input, one for output.

 * @param aIKOptions Pass along to the method attributes specified by end-user in input file.
 * @param inputData Set of frames to solve packaged as a storage fle.
 * @param outputData the frames solved by the solver represented as a storage onbject.
 */
void IKSolverImpl::solveFrames(SimTK::State& s, const IKTrial& aIKOptions, Storage& inputData, Storage& outputData)
{
	// Main loop to set initial conditions and solve snapshots
	// At every step we use experimental data as a starting guess 
	int numParameters = _ikTarget.getNumUnprescribedCoordinates();
	Array<double> unprescribedQGuess(0.0, numParameters);	// Initial guess and work array
	Array<double> unprescribedQSol(0.0, numParameters);	// Solution array
	Array<double> computedMarkerLocations(0.0, _ikTarget.getNumOutputMarkers() * 3);

	int startFrame = aIKOptions.getStartFrame(), endFrame = aIKOptions.getEndFrame();

	if (endFrame - startFrame > 1)
		cout << "Solving frames " << startFrame + 1 << " to " << endFrame + 1 << " (time = " <<
		aIKOptions.getStartTime() << " to " << aIKOptions.getEndTime() << ")" << endl;

	// Gather lower and upper bounds
	Vector lowerBounds(numParameters), upperBounds(numParameters);
	// Set the lower and upper bounds on the unprescribed Q array
	for (int i = 0; i < numParameters; i++)
	{
		const Coordinate* coord = _ikTarget.getUnprescribedCoordinate(i);
		lowerBounds[i] = coord->getRangeMin();
		upperBounds[i] = coord->getRangeMax();
	}
	_ikTarget.setParameterLimits(lowerBounds,upperBounds);

	Optimizer *optimizer = createOptimizer(aIKOptions, _ikTarget);

	try {

	// Invoke optimization mechanism to solve for Qs
	//Vector results(numParameters); // initialize with initial guess
	SimTK::Vector results(numParameters); // initialize with initial guess

	//LARGE_INTEGER start;
	//LARGE_INTEGER stop;
	//LARGE_INTEGER frequency;

	//QueryPerformanceFrequency(&frequency);
	//QueryPerformanceCounter(&start);

	for (int index = startFrame; index <= endFrame; index++)
	{
		// Get time associated with index
		double timeT = inputData.getStateVector(index)->getTime();

		s.setTime(timeT);
		_ikTarget.getModel().getSystem().realize( s, SimTK::Stage::Time );

		// Set value for prescribed coordinates and get initial guess for unprescribed coordinates
		_ikTarget.prepareToSolve(s, index, &unprescribedQGuess[0]);

		// Invoke optimization mechanism to solve for Qs
		if(index == startFrame) { 
			for(int i=0;i<numParameters;i++) {
				results[i]=unprescribedQGuess[i]; 
			}
		}
		else { 
			for(int i=0;i<numParameters;i++) {
				results[i]=unprescribedQSol[i]; 
			}
		}

		if(optimizer){
			try {
				_ikTarget.getModel().getSystem().realize( s, SimTK::Stage::Velocity );
				_ikTarget.setCurrentState( &s );
				optimizer->optimize(results);
			}
			catch (const SimTK::Exception::Base &ex) {
				cout << ex.getMessage() << endl;
				cout << "OPTIMIZATION FAILED..." << endl;
			}
		}
		else {
		   _ikTarget.setCurrentState( &s );
           _ikTarget.iterativeOptimization(s, results);
		}

		for(int i=0;i<numParameters;i++) unprescribedQSol[i]=results[i];

		/* Output variables include unprescribed (solved) Qs... */
		Array<double> qsAndMarkersArray = unprescribedQSol;

		/* ... then prescribed Qs... */
		Array<double> prescribedCoordinateValues(0.0);
		_ikTarget.getPrescribedCoordinateValues(s,prescribedCoordinateValues);
		qsAndMarkersArray.append(prescribedCoordinateValues);

		/* ... then, optionally, computed marker locations. */
		if (aIKOptions.getIncludeMarkers())
		{
			_ikTarget.getComputedMarkerLocations(computedMarkerLocations);
			qsAndMarkersArray.append(computedMarkerLocations);
		}

		double currentTime;
		inputData.getTime(index, currentTime);
		cout << "Frame " << index + 1 << " (t=" << currentTime << "):\t";
		_ikTarget.printPerformance( &unprescribedQSol[0]);

		// Append user data to qsAndMarkersArray
		Array<double> dataRow(qsAndMarkersArray);
		appendUserData(dataRow, _userDataColumnIndices, inputData.getStateVector(index));

		// Allocate new row (StateVector) and add it to outputData
		StateVector *nextDataRow = new StateVector();
		nextDataRow->setStates(timeT, dataRow.getSize(), &dataRow[0]);
		outputData.append(*nextDataRow);

		// ANALYSES
		// TODO: pass callback a reasonable "dt" value
		AnalysisSet& analysisSet = _ikTarget.getModel().updAnalysisSet();
		
		analysisSet.step(_ikTarget.getModel().getSystem().getDefaultState(), index);
	    

	}

	//QueryPerformanceCounter(&stop);
	//double duration = (double)(stop.QuadPart-start.QuadPart)/(double)frequency.QuadPart;
	//cout << "IK solution time = " << (duration*1.0e3) << " milliseconds" << endl;

	} catch (...) { // e.g. may get InterruptedException from the callbackSet
		delete optimizer;
		throw;
	}
	delete optimizer;
}

void IKSolverImpl::interrupt()
{
	_ikTarget.interrupt();
}
//______________________________________________________________________________
/**
 * UserData is the set of columns that are not used directly by the IK problem (for example
 * ground reaction forces). This UserData needs to be carried along in IK.
 */
void IKSolverImpl::collectUserData(const OpenSim::Array<string> &inputColumnLabels,
											  const OpenSim::Array<string> &resultColumnLabels,
											  OpenSim::Array<string> &userColumnLabels,
									   OpenSim::Array<int>& userDataColumnIndices)
{
	// Find columns that are none of the above to append them to the end of the list
	for(int i=0; i< inputColumnLabels.getSize(); i++){
		string nextLabel = inputColumnLabels[i];
		if(resultColumnLabels.findIndex(nextLabel)==-1) {
			// This input column is not already part of our results, so we'll add it as user data
			userColumnLabels.append(nextLabel);
			// Keep track of indices to match data with labels
			userDataColumnIndices.append(i);
		}
	}
}
//______________________________________________________________________________
/**
 * Companion helper function to (collectUserData) responsible for appending user columns 
 * to outputRow.
 */
void IKSolverImpl::appendUserData(OpenSim::Array<double>& outputRow, OpenSim::Array<int>& indices, StateVector* inputRow)
{
	int i;
	for(i=0; i< indices.getSize(); i++){
		double userValue=SimTK::NaN;
		int index = indices[i]-1;
		inputRow->getDataValue(index, userValue);
		outputRow.append(userValue);
	}
}
//______________________________________________________________________________
/**
 */
SimTK::Optimizer *IKSolverImpl::createOptimizer(const IKTrial &aIKOptions, SimTK::OptimizerSystem &aSystem) const
{
	// Pick optimizer algorithm
	SimTK::OptimizerAlgorithm algorithm = SimTK::InteriorPoint;
	if(IO::Uppercase(aIKOptions.getOptimizerAlgorithm()) == "CFSQP") {
		if(!SimTK::Optimizer::isAlgorithmAvailable(SimTK::CFSQP)) {
			std::cout << "CFSQP optimizer algorithm unavailable.  Will try to use IPOPT instead." << std::endl;
			algorithm = SimTK::InteriorPoint;
		} else {
			std::cout << "Using CFSQP optimizer algorithm." << std::endl;
			algorithm = SimTK::CFSQP;
		}
	} else if(IO::Uppercase(aIKOptions.getOptimizerAlgorithm()) == "IPOPT") {
		std::cout << "Using Ipopt optimizer algorithm." << std::endl;
		algorithm = SimTK::InteriorPoint;
	} else if(IO::Uppercase(aIKOptions.getOptimizerAlgorithm()) == "JACOBIAN") {
		std::cout << "Using Jacobian with Linear Least Squares Solver." << std::endl;
		algorithm = SimTK::BestAvailiable;
	} else {
		throw Exception("CMCTool: ERROR- Unrecognized optimizer algorithm: '"+aIKOptions.getOptimizerAlgorithm()+"'",__FILE__,__LINE__);
	}

	SimTK::Optimizer *optimizer = NULL;

	if(algorithm != SimTK::BestAvailiable) {
	
		optimizer = new SimTK::Optimizer(aSystem, algorithm);

		optimizer->setDiagnosticsLevel(0);
		optimizer->setConvergenceTolerance(1e-4);
		optimizer->setMaxIterations(1000);
		optimizer->useNumericalGradient(false); // Use our own central difference approximations
		optimizer->useNumericalJacobian(false);

		if(algorithm == SimTK::InteriorPoint) {
			// Some IPOPT-specific settings
			optimizer->setLimitedMemoryHistory(500); // works well for our small systems
			optimizer->setAdvancedBoolOption("warm_start",true);
			optimizer->setAdvancedRealOption("obj_scaling_factor",1);
			optimizer->setAdvancedRealOption("nlp_scaling_max_gradient",1);
		}
	}

	return optimizer;
}
