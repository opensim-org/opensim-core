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
#include <OpenSim/Simulation/SIMM/simmMacros.h>
#include <OpenSim/Simulation/SIMM/SimmKinematicsEngine.h>
#include <OpenSim/Simulation/SIMM/SimmMarker.h>
#include <OpenSim/Simulation/SIMM/SimmBody.h>
#include <OpenSim/Simulation/SIMM/SimmModel.h>
#include "SimmInverseKinematicsTarget.h"



using namespace OpenSim;
const double SimmInverseKinematicsTarget::_perturbation=1e-3; 

using namespace std;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Constructor
 */
static bool debug = false; // used for debugging
/*
/* used for debugging to tell if the code is in computePerformanceGradient context
*/
static bool calcDerivs = true; 

SimmInverseKinematicsTarget::SimmInverseKinematicsTarget(SimmModel &aModel, Storage& aExperimentalDataStorage):
_model(aModel),
_experimentalDataStorage(aExperimentalDataStorage),
_markers(NULL)
{
	// Mark these arrays as not owned so that we don't free the model's Qs 
	_unconstrainedQs.setMemoryOwner(false);
	_prescribedQs.setMemoryOwner(false);

	buildMarkerMap(aExperimentalDataStorage.getColumnLabelsArray());
	buildCoordinateMap(aExperimentalDataStorage.getColumnLabelsArray());

	/** Number of controls. */
	setNX(_numUnconstrainedQs);
	/** Number of performance criteria. */
	_np=1;
	/** Number of nonlinear inequality constraints. */
	_nineqn=0;
	/** Number of inequality constraints. */
	// Every Q has min and max
	_nineq=0; // Set min & max on optiimizer 2*model->getNQ();
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
	SimmKinematicsEngine& ke = _model.getSimmKinematicsEngine();
	// Assemble model in new configuration
	// x contains values only for independent/unconstrained states
	for (i = 0; i < _numUnconstrainedQs; i++)
	{
		_unconstrainedQs[i]->setValue(x[i]);
	}

	int numCalls = 0;
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
			// Get marker offset in local frame
			_markers[i]->marker->getOffset(_markers[i]->computedPosition);

			// transform local marker to world frame
			ke.convertPoint(_markers[i]->computedPosition, _markers[i]->body, ke.getGroundBodyPtr());

			err = 0.0;
			for (int j = 0; j < 3; j++)
			{
				err = _markers[i]->experimentalPosition[j] - _markers[i]->computedPosition[j];
				markerError += (err * err);
			}
			if (markerError > maxMarkerError)
			{
				maxMarkerError = markerError;
				worstMarker = i;
			}
			weight = _markers[i]->marker->getWeight();
			if (0)
			{
				cout << _markers[i]->marker->getName() << " exp = " << _markers[i]->experimentalPosition[0] << " " << _markers[i]->experimentalPosition[1] << " " << _markers[i]->experimentalPosition[2] <<
					" comp + " << _markers[i]->computedPosition[0] << " " << _markers[i]->computedPosition[1] << " " << _markers[i]->computedPosition[2] << endl;
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
			double computedValue = _unconstrainedQs[i]->getValue();
			err = experimentalValue - computedValue;
			coordinateError += (err * err);
			if (coordinateError > maxCoordinateError)
			{
				maxCoordinateError = coordinateError;
				worstCoordinate = i;
			}
			weight = _unconstrainedQs[i]->getIKweight();
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
			qGuess[i] = _unconstrainedQs[i]->getValue();
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
			bool lockedState = _prescribedQs[i]->isLocked();
			_prescribedQs[i]->setLocked(false);
			_prescribedQs[i]->setValue(value);
			_prescribedQs[i]->setLocked(lockedState);
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
		aQValues.append(_prescribedQs[i]->getValue());
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

	SimmBodyArray& bodies = _model.getSimmKinematicsEngine().getBodyArray();

	for (int i = 0; i < aNameArray.getSize(); i++)
	{
		/* Marker names should show up in the experimental data three times, with
		 * the suffixes _tx, _ty, and _tz. You only want to look for the marker
		 * once in the model, so deal with only the _tx name. Also, you have to
		 * strip off the _tx suffix before comparing it to the names of the
		 * markers in the model.
		 */
		bool foundIt = false;
		string mName = aNameArray[i];
		if (mName.length() > 3 && mName.rfind("_tx") == mName.length() - 3)
		{
			mName.erase(mName.end() - 3, mName.end());

			for (int j = 0; j < bodies.getSize(); j++)
			{
				for (int k = 0; k < bodies[j]->getNumMarkers(); k++)
				{
					if (bodies[j]->getMarker(k)->getName() == mName)
					{
						markerToSolve *newMarker = new markerToSolve;
						newMarker->marker = bodies[j]->getMarker(k);
						newMarker->body = bodies[j];
						newMarker->experimentalColumn = i-1;	// make it i-1 to account for time column
						_markers.append(newMarker);
						foundIt = true;
						j = bodies.getSize(); // to break out of outer loop
						break;
					}
				}
			}
			if (!foundIt)
				cout << "___WARNING___: marker " << mName << " not found in model. It will not be used for IK." << endl;
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
	int i;
	// The unconstrained Qs are the unlocked coordinates in the kinematics engine.
	_model.getSimmKinematicsEngine().getUnlockedCoordinates(_unconstrainedQs);
	_numUnconstrainedQs = _unconstrainedQs.getSize();

	_unconstrainedQsIndices = new int[_numUnconstrainedQs];

	for (i = 0; i < _numUnconstrainedQs; i++)
	{
		_unconstrainedQsIndices[i] = -1;

		for (int j = 0; j < aNameArray.getSize(); j++)
		{
			if (_unconstrainedQs[i]->getName() == aNameArray[j])
			{
				_unconstrainedQsIndices[i] = j-1;		// Account for time column
				break;
			}
		}
	}

#if 0
	cout << "Unconstrained Qs:" << endl;
	for (i = 0; i < _unconstrainedQs.getSize(); i++)
		cout << _unconstrainedQs[i]->getName() << " index = " << _unconstrainedQsIndices[i] << endl;
#endif

	_numPrescribedQs = 0;
	_prescribedQsIndices = new int[_model.getKinematicsEngine().getNumCoordinates()];
	ArrayPtrs<SimmCoordinate>& coords = _model.getSimmKinematicsEngine().getCoordinates();
	for (i = 0; i < coords.getSize(); i++)
	{
		if (coords[i]->isLocked())
		{
			for (int j = 0; j < aNameArray.getSize(); j++)
			{
				if (coords[i]->getName() == aNameArray[j])
				{
					_prescribedQsIndices[_numPrescribedQs++] = j-1;		// Account for time column
					_prescribedQs.append(coords[i]);
					break;
				}
			}
		}
	}

#if 0
	cout << "Prescribed Qs:" << endl;
	for (i = 0; i < _prescribedQs.getSize(); i++)
		cout << _prescribedQs[i]->getName() << " index = " << _prescribedQsIndices[i] << endl;
#endif
}

void SimmInverseKinematicsTarget::getUnconstrainedCoordinateNames(Array<const string*>& aNameArray)
{
	aNameArray.setSize(0);

	for (int i = 0; i < _numUnconstrainedQs; i++)
		aNameArray.append(&_unconstrainedQs[i]->getName());
}

void SimmInverseKinematicsTarget::getPrescribedCoordinateNames(Array<const string*>& aNameArray)
{
	aNameArray.setSize(0);

	for (int i = 0; i < _numPrescribedQs; i++)
		aNameArray.append(&_prescribedQs[i]->getName());
}

void SimmInverseKinematicsTarget::getOutputMarkerNames(Array<const string*>& aNameArray)
{
	aNameArray.setSize(0);

	for (int i = 0; i < _markers.getSize(); i++)
		aNameArray.append(&_markers[i]->marker->getName());
}
