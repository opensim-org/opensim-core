// SimmInverseKinematicsTarget.cpp
// Authors: Ayman Habib, Peter Loan
/* Copyright (c) 2005, Stanford University, Ayman Habib, and Peter Loan.
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
#include <OpenSim/Tools/rdMath.h>
#include <OpenSim/Tools/Storage.h>
#include <OpenSim/Simulation/SIMM/SimmMacros.h>
#include <OpenSim/Simulation/SIMM/SimmKinematicsEngine.h>
#include <OpenSim/Simulation/SIMM/AbstractCoordinate.h>
#include <OpenSim/Simulation/SIMM/MarkerSet.h>
#include "SimmInverseKinematicsTarget.h"

using namespace std;
using namespace OpenSim;

const double SimmInverseKinematicsTarget::_perturbation=1e-3; 

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

SimmInverseKinematicsTarget::SimmInverseKinematicsTarget(AbstractModel &aModel, Storage& aExperimentalDataStorage):
_model(aModel),
_experimentalDataStorage(aExperimentalDataStorage),
_markers(NULL),
_unconstrainedQs(),
_prescribedQs()
{
	// Mark these arrays as not owned so that we don't free the model's Qs 
	_unconstrainedQs.setMemoryOwner(false);
	_prescribedQs.setMemoryOwner(false);

	buildMarkerMap(aExperimentalDataStorage.getColumnLabelsArray());
	buildCoordinateMap(aExperimentalDataStorage.getColumnLabelsArray());

	/** Number of controls. */
	setNumControls(_numUnconstrainedQs);
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
	/** Perturbation size for computing numerical derivatives. */
	_dx=new double[_numUnconstrainedQs];

	for (int i = 0; i < _numUnconstrainedQs; i++)
		_dx[i] = _perturbation;
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
SimmInverseKinematicsTarget::
~SimmInverseKinematicsTarget(void)
{
	for (int i = 0; i < _markers.getSize(); i++)
		delete _markers[i];

	delete [] _unconstrainedQsIndices;
	delete [] _prescribedQsIndices;
}

//=============================================================================
// Optimization framework methods
//=============================================================================
//_____________________________________________________________________________
/**
 * Compute objective function (sum of squared errors in marker positions) using
 * current values (x) for controls. Value of error is returned in p
 */
int SimmInverseKinematicsTarget::computePerformance(double *x, double *p)
{
	int i;
	AbstractDynamicsEngine& de = _model.getDynamicsEngine();
	// Assemble model in new configuration
	// x contains values only for independent/unconstrained states
	for (i = 0; i < _numUnconstrainedQs; i++)
	{
		_unconstrainedQs.get(i)->setValue(x[i]);
		if (debug)
			cout << _unconstrainedQs.get(i)->getName() << " = " << _unconstrainedQs.get(i)->getValue() << endl;
	}

	double time;

	_experimentalDataStorage.getTime(_indexToSolve, time);
	StateVector *dataRow = _experimentalDataStorage.getStateVector(_indexToSolve);

	// Tally the square of the errors from markers
	double err, weight, totalErrorSquared = 0.0;
	double maxMarkerError = 0.0, maxCoordinateError = 0.0;
	int worstMarker = -1, worstCoordinate = -1;

	// We keep track of worst marker for debugging/tuning purposes
	for (i = 0; i < _markers.getSize(); i++)
	{
		double markerError = 0.0;

		// get the location of the marker in the experimental data
		int dataColumnNumber = _markers[i]->experimentalColumn;
		dataRow->getDataValue(dataColumnNumber, _markers[i]->experimentalPosition[0]);
		dataRow->getDataValue(dataColumnNumber + 1, _markers[i]->experimentalPosition[1]);
		dataRow->getDataValue(dataColumnNumber + 2, _markers[i]->experimentalPosition[2]);

		/* If the marker is missing from this frame, its coordinates will
		 * all be NAN. In that case, do not compute an error for the marker.
		 */
		if (NOT_EQUAL_WITHIN_ERROR(_markers[i]->experimentalPosition[0], rdMath::NAN) &&
			 NOT_EQUAL_WITHIN_ERROR(_markers[i]->experimentalPosition[1], rdMath::NAN) &&
			 NOT_EQUAL_WITHIN_ERROR(_markers[i]->experimentalPosition[2], rdMath::NAN))
		{
			double globalPos[3];

			// Get marker offset in local frame
			_markers[i]->marker->getOffset(_markers[i]->computedPosition);

			// transform local marker to world frame
			de.transformPosition(*_markers[i]->body, _markers[i]->computedPosition, globalPos);

			err = 0.0;
			for (int j = 0; j < 3; j++)
			{
				err = _markers[i]->experimentalPosition[j] - globalPos[j];
				markerError += (err * err);
			}
			if (markerError > maxMarkerError)
			{
				maxMarkerError = markerError;
				worstMarker = i;
			}
			weight = _markers[i]->marker->getWeight();
			if (debug)
			{
				cout << _markers[i]->marker->getName() << " w = " << weight << " exp = " << _markers[i]->experimentalPosition[0] << " " << _markers[i]->experimentalPosition[1] << " " << _markers[i]->experimentalPosition[2] <<
					" comp + " << globalPos[0] << " " << globalPos[1] << " " << globalPos[2] << endl;
			}
			totalErrorSquared += (markerError * weight);
		}
	}

	for (i = 0; i < _numUnconstrainedQs; i++)
	{
		if (_unconstrainedQsIndices[i] >= 0)
		{
			double coordinateError = 0.0;
			double experimentalValue;
			dataRow->getDataValue(_unconstrainedQsIndices[i], experimentalValue);
			double computedValue = _unconstrainedQs.get(i)->getValue();
			err = experimentalValue - computedValue;
			coordinateError += (err * err);
			if (coordinateError > maxCoordinateError)
			{
				maxCoordinateError = coordinateError;
				worstCoordinate = i;
			}
			weight = _unconstrainedQs.get(i)->getWeight();
			if (debug)
			{
				cout << _unconstrainedQs.get(i)->getName() << " w = " << weight << " exp = " << experimentalValue << " comp + " << computedValue << endl;
			}
			totalErrorSquared += (coordinateError * weight);
		}
	}

	if (!calcDerivs && debug)
	{
		cout << "Total Error Squared = " << totalErrorSquared << endl;
		if (worstMarker >= 0)
			cout << "Largest Marker Err Squared = " << maxMarkerError << " at marker " << _markers[worstMarker]->marker->getName() << endl;
		if (worstCoordinate >= 0)
			cout << "Largest Coordinate Err Squared = " << maxCoordinateError << " at coordinate " << _unconstrainedQs[worstCoordinate]->getName() << endl;
	}

	*p = totalErrorSquared;

	return 0;
}

//_____________________________________________________________________________
/**
 * Compute derivative of objective function using finite differences
 */
int SimmInverseKinematicsTarget::
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
int SimmInverseKinematicsTarget::compute(double *x,double *p,double *c){ return (0);};
int SimmInverseKinematicsTarget::computeGradients(double *dx,double *x,double *dpdx,double *dcdx){return (0);};
int SimmInverseKinematicsTarget::computeConstraint(double *x,int i,double *c){return (0);};
int SimmInverseKinematicsTarget::computeConstraintGradient(double *x,int i,double *dcdx){return (0);};

//=============================================================================
// Helper methods for book keeping
//=============================================================================
//_____________________________________________________________________________
/**
 * setIndexToSolve specifies the row of the Storage instance _experimentalDataStorage
 * that the optimizer is trying to solve. It has the side effect of returning in qGuess 
 * the readings for the columns corresponding to optimization controls
 */
void SimmInverseKinematicsTarget::setIndexToSolve(int aIndex, double* qGuess)
{
	_indexToSolve = aIndex;

	StateVector *dataRow = _experimentalDataStorage.getStateVector(aIndex);

	for (int i = 0; i < _numUnconstrainedQs; i++)
	{
		int dataColumnNumber = _unconstrainedQsIndices[i];

		/* If there are values for this coordinate in the experimental data,
		 * copy the appropriate one to qGuess. If not, use the coordinate's
		 * current value as its initial guess.
		 */
		if (dataColumnNumber >= 0)
			dataRow->getDataValue(dataColumnNumber, qGuess[i]);
		else
			qGuess[i] = _unconstrainedQs.get(i)->getValue();
	}
}

//_____________________________________________________________________________
/**
 * setPrescribedCoordinates sets the values of the prescribed coordinates
 * according to their values in the _experimentalDataStorage.
 */
void SimmInverseKinematicsTarget::setPrescribedCoordinates(int aIndex)
{
	double value;
	StateVector *dataRow = _experimentalDataStorage.getStateVector(aIndex);

	for (int i = 0; i < _numPrescribedQs; i++)
	{
		if (_prescribedQsIndices[i] >= 0)
		{
			dataRow->getDataValue(_prescribedQsIndices[i], value);
			bool lockedState = _prescribedQs.get(i)->getLocked();
			_prescribedQs.get(i)->setLocked(false);
			_prescribedQs.get(i)->setValue(value);
			_prescribedQs.get(i)->setLocked(lockedState);
		}
	}
}

//_____________________________________________________________________________
/**
 * getComputedMarkerLocations returns current commputed (model) marker locations for debugging and display purposes
 */
void SimmInverseKinematicsTarget::getComputedMarkerLocations(Array<double> &aMarkerLocations) const
{
	aMarkerLocations.setSize(0);

	for (int i = 0; i < _markers.getSize(); i++)
	{
		aMarkerLocations.append(_markers[i]->computedPosition[0]);
		aMarkerLocations.append(_markers[i]->computedPosition[1]);
		aMarkerLocations.append(_markers[i]->computedPosition[2]);
	}
}

//_____________________________________________________________________________
/**
 * getExperimentalMarkerLocations returns current experimental marker locations for debugging and display purposes
 */
void SimmInverseKinematicsTarget::getExperimentalMarkerLocations(Array<double> &aMarkerLocations) const
{
	aMarkerLocations.setSize(0);

	for (int i = 0; i < _markers.getSize(); i++)
	{
		aMarkerLocations.append(_markers[i]->experimentalPosition[0]);
		aMarkerLocations.append(_markers[i]->experimentalPosition[1]);
		aMarkerLocations.append(_markers[i]->experimentalPosition[2]);
	}
}

//_____________________________________________________________________________
/**
 * getComputedMarkerLocations returns current marker locations for debugging and display purposes
 */
void SimmInverseKinematicsTarget::getPrescribedQValues(Array<double>& aQValues) const
{
	aQValues.setSize(0);

	for (int i = 0; i < _prescribedQs.getSize(); i++)
		aQValues.append(_prescribedQs.get(i)->getValue());
}

//_____________________________________________________________________________
/**
 * buildMarkerNameMap is a utility used to construct an array of references to
 * markers that are in the model and also in the experimental data. Stored with
 * the reference is the corresponding index into the experimental data columns.
 */
void SimmInverseKinematicsTarget::buildMarkerMap(const Array<string>& aNameArray)
{
	_markers.setSize(0);

	MarkerSet* markerSet = _model.getDynamicsEngine().getMarkerSet();

	int i;
	for (i = 0; i < aNameArray.getSize(); i++)
	{
		// Marker names should show up in the experimental data three times, with
		// the suffixes _tx, _ty, and _tz. You only want to look for the marker
		// once in the model, so deal with only the _tx name. Also, you have to
		// strip off the _tx suffix before comparing it to the names of the
		// markers in the model.
		string mName = aNameArray[i];
		if (mName.length() > 3 && (mName.rfind("_tx") == mName.length() - 3))
		{
			mName.erase(mName.end() - 3, mName.end());

			AbstractMarker* modelMarker = markerSet->get(mName);
			if (modelMarker)
			{
				markerToSolve* newMarker = new markerToSolve;
				newMarker->marker = modelMarker;
				newMarker->body = modelMarker->getBody();
				newMarker->experimentalColumn = i - 1;		// make it i-1 to account for time column
				_markers.append(newMarker);
			}
			else
			{
				cout << "___WARNING___: marker " << mName << " not found in model. It will not be used for IK." << endl;
			}
		}
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
void SimmInverseKinematicsTarget::buildCoordinateMap(const Array<string>& aNameArray)
{
	int i, j;

	// The unconstrained Qs are the unlocked coordinates in the kinematics engine.
	_model.getDynamicsEngine().getUnlockedCoordinates(_unconstrainedQs);
	_numUnconstrainedQs = _unconstrainedQs.getSize();

	_unconstrainedQsIndices = new int[_numUnconstrainedQs];

	for (i = 0; i < _numUnconstrainedQs; i++)
	{
		_unconstrainedQsIndices[i] = -1;

		for (j = 0; j < aNameArray.getSize(); j++)
		{
			if (_unconstrainedQs.get(i)->getName() == aNameArray[j])
			{
				_unconstrainedQsIndices[i] = j-1;		// Account for time column
				break;
			}
		}
	}

#if 0
	cout << "Unconstrained Qs:" << endl;
	for (i = 0; i < _unconstrainedQs.getSize(); i++)
		cout << _unconstrainedQs.get(i)->getName() << " index = " << _unconstrainedQsIndices[i] << endl;
#endif

	_numPrescribedQs = 0;
	_prescribedQsIndices = new int[_model.getDynamicsEngine().getNumCoordinates()];
	_prescribedQs.setMemoryOwner(false);

	const CoordinateSet* coordinateSet = _model.getDynamicsEngine().getCoordinateSet();

	for (i = 0; i < coordinateSet->getSize(); i++)
	{
		AbstractCoordinate* coord = coordinateSet->get(i);
		if (coord->getLocked())
		{
			for (j = 0; j < aNameArray.getSize(); j++)
			{
				if (coord->getName() == aNameArray[j])
				{
					_prescribedQsIndices[_numPrescribedQs++] = j-1;		// Account for time column
					_prescribedQs.append(coord);
					break;
				}
			}
		}
	}

#if 0
	cout << "Prescribed Qs:" << endl;
	for (i = 0; i < _prescribedQs.getSize(); i++)
		cout << _prescribedQs.get(i)->getName() << " index = " << _prescribedQsIndices[i] << endl;
#endif
}

void SimmInverseKinematicsTarget::getUnconstrainedCoordinateNames(Array<const string*>& aNameArray)
{
	aNameArray.setSize(0);

	for (int i = 0; i < _numUnconstrainedQs; i++)
		aNameArray.append(&_unconstrainedQs.get(i)->getName());
}

void SimmInverseKinematicsTarget::getPrescribedCoordinateNames(Array<const string*>& aNameArray)
{
	aNameArray.setSize(0);

	for (int i = 0; i < _numPrescribedQs; i++)
		aNameArray.append(&_prescribedQs.get(i)->getName());
}

void SimmInverseKinematicsTarget::getOutputMarkerNames(Array<const string*>& aNameArray)
{
	aNameArray.setSize(0);

	for (int i = 0; i < _markers.getSize(); i++)
		aNameArray.append(&_markers[i]->marker->getName());
}
