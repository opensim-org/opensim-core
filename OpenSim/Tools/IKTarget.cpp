// IKTarget.cpp
// Authors: Ayman Habib, Peter Loan, Eran Guendelman
/* Copyright (c) 2005, Stanford University, Ayman Habib, Peter Loan, and Eran Guendelman.
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
#include <OpenSim/SQP/rdFSQP.h>
#include <OpenSim/Common/rdMath.h>
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Common/SimmMacros.h>
#include <OpenSim/Simulation/Model/AbstractDynamicsEngine.h>
#include <OpenSim/Simulation/Model/AbstractCoordinate.h>
#include <OpenSim/Simulation/Model/MarkerSet.h>
#include "IKTaskSet.h"
#include "IKCoordinateTask.h"
#include "IKMarkerTask.h"
#include "IKTarget.h"

using namespace std;
using namespace OpenSim;

const double IKTarget::_perturbation=1e-3; 

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Constructor
 */
static bool debug = false; // used for debugging
/*
 * used for debugging to tell if the code is in computePerformanceGradient context
*/
static bool calcDerivs = true; 

IKTarget::IKTarget(Model &aModel, IKTaskSet &aIKTaskSet, Storage& aExperimentalDataStorage):
_model(aModel),
_ikTaskSet(aIKTaskSet),
_experimentalDataStorage(aExperimentalDataStorage),
_markers(NULL)
{
	buildMarkerMap(aExperimentalDataStorage.getColumnLabels());
	buildCoordinateMap(aExperimentalDataStorage.getColumnLabels());

	/** Number of controls -- also allocates _dx. */
	setNumControls(_unprescribedQs.getSize());
	/** Number of performance criteria. */
	_np=1;
	/** Number of nonlinear inequality constraints. */
	_nineqn=0;
	/** Number of inequality constraints. */
	// Every Q has min and max
	_nineq=0; // Set min & max on optiimizer 2*model->getNumCoordinates();
	/** Number of nonlinear equality constraints. */
	_neqn=0;
	/** Number of equality constraints. */
	_neq=0;

	for (int i = 0; i < _nx; i++)
		_dx[i] = _perturbation;

	_printPerformanceValues = false;
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
IKTarget::
~IKTarget(void)
{
	for(int i=0; i<_markers.getSize(); i++) delete _markers[i];
	for(int i=0; i<_prescribedQs.getSize(); i++) delete _prescribedQs[i];
	for(int i=0; i<_unprescribedQs.getSize(); i++) delete _unprescribedQs[i];
	// don't delete contents of _unprescribedWeightedQs since those are a subset of _unprescribedQs
}

//=============================================================================
// Optimization framework methods
//=============================================================================
//_____________________________________________________________________________
/**
 * Compute objective function (sum of squared errors in marker positions) using
 * current values (x) for controls. Value of error is returned in p
 */
int IKTarget::computePerformance(double *x, double *p)
{
	// Assemble model in new configuration
	// x contains values only for unprescribed coordinates
	for (int i = 0; i < _nx; i++)
	{
		_unprescribedQs[i]->coord->setValue(x[i]);
		if (debug)
			cout << _unprescribedQs[i]->coord->getName() << " = " << _unprescribedQs[i]->coord->getValue() << endl;
	}

	// Tally the square of the errors from markers
	double totalErrorSquared = 0.0;
	double maxMarkerError = 0.0, maxCoordinateError = 0.0; // these are the max weighted errors
	int worstMarker = -1, worstCoordinate = -1;

	AbstractDynamicsEngine& de = _model.getDynamicsEngine();

	// We keep track of worst marker for debugging/tuning purposes
	for (int i = 0; i < _markers.getSize(); i++)
	{
		if(!_markers[i]->validExperimentalPosition) continue;
		double markerError = 0.0;
		double globalPos[3];

		// Get marker offset in local frame
		_markers[i]->marker->getOffset(_markers[i]->computedPosition);

		// transform local marker to world frame
		de.transformPosition(*_markers[i]->body, _markers[i]->computedPosition, globalPos);

		double err = 0.0;
		for (int j = 0; j < 3; j++)
		{
			err = _markers[i]->experimentalPosition[j] - globalPos[j];
			markerError += (err * err);
		}
		markerError *= _markers[i]->weight;
		if (markerError > maxMarkerError)
		{
			maxMarkerError = markerError;
			worstMarker = i;
		}
		totalErrorSquared += markerError;

		if (debug)
			cout << _markers[i]->marker->getName() << " w = " << _markers[i]->weight 
				  << " exp = " << _markers[i]->experimentalPosition[0] << " " << _markers[i]->experimentalPosition[1] << " " << _markers[i]->experimentalPosition[2]
				  << " comp + " << globalPos[0] << " " << globalPos[1] << " " << globalPos[2] << endl;
	}

	for (int i = 0; i < _unprescribedWeightedQs.getSize(); i++)
	{
		double experimentalValue = _unprescribedWeightedQs[i]->experimentalValue;
		double computedValue = _unprescribedWeightedQs[i]->coord->getValue();
		double err = experimentalValue - computedValue;
		double coordinateError = _unprescribedWeightedQs[i]->weight * err * err;
		if (coordinateError > maxCoordinateError)
		{
			maxCoordinateError = coordinateError;
			worstCoordinate = i;
		}
		totalErrorSquared += coordinateError;

		if (debug)
			cout << _unprescribedWeightedQs[i]->coord->getName() << " w = " << _unprescribedWeightedQs[i]->weight << " exp = " << experimentalValue << " comp + " << computedValue << endl;
	}

	if (_printPerformanceValues || (!calcDerivs && debug))
	{
		cout << "total error = " << totalErrorSquared;
		if (worstMarker >= 0)
			cout << ", worst marker " << _markers[worstMarker]->marker->getName() << " (" << maxMarkerError << ")";
		if (worstCoordinate >= 0)
			cout << ", worst coordinate " << _unprescribedWeightedQs[worstCoordinate]->coord->getName() << " (" << maxCoordinateError << ")";
		cout << endl;
		setErrorReportingQuantities(
			maxMarkerError, 
			(worstMarker<0)?"":_markers[worstMarker]->marker->getName(),
			maxCoordinateError, 
			(worstCoordinate<0)?"":_unprescribedWeightedQs[worstCoordinate]->coord->getName());
	}

	*p = totalErrorSquared;

	return 0;
}

//_____________________________________________________________________________
/**
 * Compute derivative of objective function using finite differences
 */
int IKTarget::
computePerformanceGradient(double *x,double *dpdx)
{
	calcDerivs=true;
	int status = rdFSQP::CentralDifferences(this,_dx,x,dpdx);
	calcDerivs=false;

	return (status);
}

//_____________________________________________________________________________
/**
 * Default implementation for methods unused by the optimizer for this problem
 */
int IKTarget::compute(double *x,double *p,double *c){ return (0);};
int IKTarget::computeGradients(double *dx,double *x,double *dpdx,double *dcdx){return (0);};
int IKTarget::computeConstraint(double *x,int i,double *c){return (0);};
int IKTarget::computeConstraintGradient(double *x,int i,double *dcdx){return (0);};

//=============================================================================
// Helper methods for book keeping
//=============================================================================
//_____________________________________________________________________________
/**
 * prepareToSolve specifies the row of the Storage instance _experimentalDataStorage
 * that the optimizer is trying to solve.
 *
 * It also sets the values of the prescribed coordinates and returns the
 * initial guess for the unprescribed coordinates.
 */
void IKTarget::prepareToSolve(int aIndex, double* qGuess)
{
	double time;
	_experimentalDataStorage.getTime(aIndex,time);
	StateVector *dataRow = _experimentalDataStorage.getStateVector(aIndex);

	//--------------------------------------------------------------------
	// PRESCRIBED COORDINATES
	//--------------------------------------------------------------------
	// Set prescribed coordinates to their file value or to the constant experimental value
	for(int i=0; i<_prescribedQs.getSize(); i++)
	{
		double value;
		coordinateInfo *info = _prescribedQs[i];
		// Either get value from file or use the constantExperimentalValue
		if(_prescribedQs[i]->experimentalColumn >= 0)
			dataRow->getDataValue(info->experimentalColumn, value);
		else
			value = info->constantExperimentalValue;

		AbstractCoordinate *coord = info->coord;
		bool lockedState = coord->getLocked(); // presumebly this should return true since it's a prescribed Q!
		coord->setLocked(false);
		coord->setValue(value);
		coord->setLocked(lockedState);
	}

	//--------------------------------------------------------------------
	// UNPRESCRIBED COORDINATES
	//--------------------------------------------------------------------
	// Get initial guess and set the target experimental value for unprescribed coordinates
	for(int i=0; i<_unprescribedQs.getSize(); i++)
	{
		coordinateInfo *info = _unprescribedQs[i];

		// Set the initial guess
		if(info->experimentalColumn >= 0) {
			// Use the value from file as the initial guess
			dataRow->getDataValue(info->experimentalColumn, qGuess[i]);
		} else {
			// Use its current value as its initial guess
			qGuess[i] = info->coord->getValue();
		}

		// If this unprescribed coordinate has a nonzero weight, we need an experimental target value for it.
		// Get it either from file or based on the constantExperimentalValue
		if(info->weight)
			info->experimentalValue = (info->experimentalColumn >= 0) ? qGuess[i] : info->constantExperimentalValue;
	}

	//--------------------------------------------------------------------
	// MARKERS
	//--------------------------------------------------------------------
	// Get the experimental marker positions for all markers that will be solved (i.e. have non-zero weight)
	for(int i=0; i<_markers.getSize(); i++) {
		// get the location of the marker in the experimental data
		int dataColumnNumber = _markers[i]->experimentalColumn;

		dataRow->getDataValue(dataColumnNumber, _markers[i]->experimentalPosition[0]);
		dataRow->getDataValue(dataColumnNumber + 1, _markers[i]->experimentalPosition[1]);
		dataRow->getDataValue(dataColumnNumber + 2, _markers[i]->experimentalPosition[2]);

		/* If the marker is missing from this frame, its coordinates will
		 * all be NAN. In that case, do not compute an error for the marker.
		 * THIS IS COMPLETELY WRONG!! WHAT IF A MARKER POSITION IS SUPPOSED TO BE (0,0,0)??
		 * IT WOULD BE TRIGGERED AS INVALID!
		 */
		_markers[i]->validExperimentalPosition =
		   (NOT_EQUAL_WITHIN_ERROR(_markers[i]->experimentalPosition[0], rdMath::NAN) &&
			 NOT_EQUAL_WITHIN_ERROR(_markers[i]->experimentalPosition[1], rdMath::NAN) &&
			 NOT_EQUAL_WITHIN_ERROR(_markers[i]->experimentalPosition[2], rdMath::NAN));
	}
}

//_____________________________________________________________________________
/**
 * buildMarkerNameMap is a utility used to construct an array of references to
 * markers that are in the model and also in the experimental data. Stored with
 * the reference is the corresponding index into the experimental data columns.
 */
void IKTarget::buildMarkerMap(const Array<string>& aNameArray)
{
	_markers.setSize(0);

	MarkerSet* markerSet = _model.getDynamicsEngine().getMarkerSet();

	for(int i=0; i<_ikTaskSet.getSize(); i++) {
		IKMarkerTask *markerTask = dynamic_cast<IKMarkerTask*>(_ikTaskSet.get(i));

		if(!markerTask) continue; // not a marker task

		string markerName=markerTask->getName();
		AbstractMarker *modelMarker = markerSet->get(markerName);
		if(!modelMarker)
			throw Exception("IKTarget.buildMarkerMap: ERROR- marker '"+markerName+
								 "' named in IKMarkerTask not found in model",__FILE__,__LINE__);

		if(markerTask->getWeight() == 0) continue; // we don't care about marker tasks with zero weight

		// Marker will have a _tx (and _ty, _tz) suffix in the storage file
		int j=aNameArray.findIndex(markerName+"_tx");
		if(j<0) 
			throw Exception("IKTarget.buildMarkerMap: ERROR- experimental data for marker '"+markerName+
								 "' not found in trc file",__FILE__,__LINE__);

		markerToSolve *newMarker = new markerToSolve;
		newMarker->marker = modelMarker;
		newMarker->body = modelMarker->getBody();
		newMarker->experimentalColumn = j - 1;		// make it j-1 to account for time column
		newMarker->weight = markerTask->getWeight();
		_markers.append(newMarker);
	}
}

//_____________________________________________________________________________
/**
 * buildCoordinateNameMap is a utility used to construct a map between an array of
 * coordinate names (aNameArray) and the coordinates in a SimmKinematicsEngine. The
 * size of the map is the number of coordinates in the SimmKinematicsEngine. It is
 * important that coordinates are not added or deleted after this map is made because
 * the indexing into the map depends on a fixed set of coordinates.
 */
void IKTarget::buildCoordinateMap(const Array<string>& aNameArray)
{
	CoordinateSet* coordinateSet = _model.getDynamicsEngine().getCoordinateSet();

	// Initialize info structures for all coordinates
	Array<coordinateInfo*> allCoordinates;
	for(int i=0; i<coordinateSet->getSize(); i++) {
		AbstractCoordinate *coord = coordinateSet->get(i);

		coordinateInfo *info = new coordinateInfo;
		info->coord = coord;
		info->prescribed = coord->getLocked();

		// Initialize as if it has no task
		info->experimentalColumn = -1;
		// initialize the constant experimental value (used if from_file is false) to the current value of the SimmCoordinate
		// (comes from <value> in the SimmCoordinate, or <default_value> if that's not defined).  If the IKCoordinateTask
		// specifies its own value, constantExperimentalValue will be overwritten with that value below.
		info->constantExperimentalValue = coord->getValue();
		info->weight = 0;

		allCoordinates.append(info);
	}

	// Update info structures based on user-specified IKCoordinateTasks
	for(int i=0; i<_ikTaskSet.getSize(); i++) {
		IKCoordinateTask *coordTask = dynamic_cast<IKCoordinateTask*>(_ikTaskSet.get(i));

		if(!coordTask) continue; // not a coordinate task

		string coordName = coordTask->getName();
		int coordIndex = coordinateSet->getIndex(coordName);
		if(coordIndex<0)
			throw Exception("IKTarget.buildCoordinateMap: ERROR- coordinate '"+coordName+
								 "' named in IKCoordinateTask not found in model",__FILE__,__LINE__);

		coordinateInfo *info = allCoordinates[coordIndex];

		// Potential issue here if marker has same name as coordinate...  We'll search in reverse
		// because coordinates should appear after markers in the storage.
		// NOTE: If we're not getting the experimental value from file, we'll use constantExperimentalValue
		if(coordTask->getFromFile()) {
			int j = aNameArray.rfindIndex(coordName);
			if(j < 0)
				throw Exception("IKTarget.buildCoordinateMap: ERROR- coordinate task '"+coordName+
									 "' specifies from_file but no column found for this coordinate in coordinates file",__FILE__,__LINE__);
			info->experimentalColumn = j - 1; // account for time column
		} else if(!coordTask->getValueUseDefault()) {
			info->constantExperimentalValue = coordTask->getValue();
		}

		info->weight = coordTask->getWeight();
	}

	// Now we filter the coordinate infos into the three sets (not a partitioning since the second set is a subset of the first)
	_unprescribedQs.setSize(0);
	_unprescribedWeightedQs.setSize(0);
	_prescribedQs.setSize(0);

	for(int i=0; i<allCoordinates.getSize(); i++) {
		if(allCoordinates[i]->prescribed) _prescribedQs.append(allCoordinates[i]);
		else {
			_unprescribedQs.append(allCoordinates[i]);
			if(allCoordinates[i]->weight) _unprescribedWeightedQs.append(allCoordinates[i]);
		}
	}
}
//_____________________________________________________________________________
/**
 */
void IKTarget::printTasks() const
{
	if(_markers.getSize())
		cout << "Marker Tasks:" << endl;
	for(int i=0; i<_markers.getSize(); i++) {
		cout << "\t" << _markers[i]->marker->getName() << ": weight " << _markers[i]->weight;
		cout << " from file (columns " << _markers[i]->experimentalColumn << "-" << _markers[i]->experimentalColumn+2 << ")" << endl;
	}

	if(_unprescribedWeightedQs.getSize())
		cout << "Unprescribed Coordinate Tasks (with nonzero weight):" << endl;
	for(int i=0; i<_unprescribedWeightedQs.getSize(); i++) {
		cout << "\t" << _unprescribedWeightedQs[i]->coord->getName() << ": weight " << _unprescribedWeightedQs[i]->weight;
		if(_unprescribedWeightedQs[i]->experimentalColumn >= 0)
			cout << " from file (column " << _unprescribedWeightedQs[i]->experimentalColumn << ")" << endl;
		else
			cout << " constant target value of " << _unprescribedWeightedQs[i]->constantExperimentalValue << endl;
	}

	if(_prescribedQs.getSize())
		cout << "Prescribed Coordinate Tasks:" << endl;
	for(int i=0; i<_prescribedQs.getSize(); i++) {
		std::cout << "\t" << _prescribedQs[i]->coord->getName() << ": ";
		if(_prescribedQs[i]->experimentalColumn >= 0)
			cout << "from file (column " << _prescribedQs[i]->experimentalColumn << ")" << endl;
		else
			cout << "constant target value of " << _prescribedQs[i]->constantExperimentalValue << endl;
	}
}
//_____________________________________________________________________________
/**
 */
void IKTarget::printPerformance()
{
	_printPerformanceValues = true;
	double *qs=new double[_nx];
	for(int i=0;i<_nx;i++) qs[i]=_unprescribedQs[i]->coord->getValue();
	double p;
	computePerformance(qs,&p);
	_printPerformanceValues=false;
}

//_____________________________________________________________________________
/**
 * getComputedMarkerLocations returns current commputed (model) marker locations for debugging and display purposes
 */
void IKTarget::getComputedMarkerLocations(Array<double> &aMarkerLocations) const
{
	aMarkerLocations.setSize(0);
	for (int i = 0; i < _markers.getSize(); i++) 
		aMarkerLocations.append(3, _markers[i]->computedPosition);
}

//_____________________________________________________________________________
/**
 * getExperimentalMarkerLocations returns current experimental marker locations for debugging and display purposes
 */
void IKTarget::getExperimentalMarkerLocations(Array<double> &aMarkerLocations) const
{
	aMarkerLocations.setSize(0);
	for (int i = 0; i < _markers.getSize(); i++) 
		aMarkerLocations.append(3, _markers[i]->experimentalPosition);
}

//_____________________________________________________________________________
/**
 * getComputedMarkerLocations returns current marker locations for debugging and display purposes
 */
void IKTarget::getPrescribedCoordinateValues(Array<double>& aQValues) const
{
	aQValues.setSize(_prescribedQs.getSize());
	for (int i = 0; i < _prescribedQs.getSize(); i++)
		aQValues[i] = _prescribedQs.get(i)->coord->getValue();
}

void IKTarget::getUnprescribedCoordinateNames(Array<string>& aNameArray)
{
	aNameArray.setSize(_unprescribedQs.getSize());
	for (int i = 0; i < _unprescribedQs.getSize(); i++)
		aNameArray[i] = _unprescribedQs.get(i)->coord->getName();
}

void IKTarget::getPrescribedCoordinateNames(Array<string>& aNameArray)
{
	aNameArray.setSize(_prescribedQs.getSize());
	for (int i = 0; i < _prescribedQs.getSize(); i++)
		aNameArray[i] = _prescribedQs.get(i)->coord->getName();
}

void IKTarget::getOutputMarkerNames(Array<string>& aNameArray)
{
	aNameArray.setSize(_markers.getSize());
	for (int i = 0; i < _markers.getSize(); i++)
		aNameArray[i] = _markers[i]->marker->getName();
}

void IKTarget::setErrorReportingQuantities(const double& aMarkerError, const std::string& aMarkerName,
									const double& aCoordinateError, const std::string& aCoordinateName)
{
	_worstMarkerError=aMarkerError;
	_nameOfWorstMarker=aMarkerName;
	_worstCoordinateError=aCoordinateError;
	_nameOfWorstCoordinate=aCoordinateName;
}