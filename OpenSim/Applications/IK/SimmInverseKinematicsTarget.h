#ifndef _SimmInverseKinematicsTarget_h_
#define _SimmInverseKinematicsTarget_h_
// SimmInverseKinematicsTarget.h
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

#include <OpenSim/SQP/rdOptimizationTarget.h>
#include <OpenSim/Tools/Array.h>
#include <OpenSim/Simulation/SIMM/SimmCoordinate.h>
namespace OpenSim { 

class SimmKinematicsEngine;
class SimmMarker;
class SimmBody;
class Storage;
class SimmModel;

//=============================================================================
/**
 * A Class that represents an inverse kinematics optimization target. This solves the problem
 * of computing Qs that minimize the sum of the squared differences between a set of markers on a 
 * on the nominal model and passed in marker positions.
 *
 * @author Ayman Habib, Peter Loan
 * @version 1.0
 */

class SimmInverseKinematicsTarget : public rdOptimizationTarget
{
//==============================================================================
// DATA
//==============================================================================
private:
	SimmModel& _model;

	// Amount of perturbation used for derivative computation this should be an array
	static const double _perturbation;

	int _indexToSolve;

	// internal datastructure used to map data columns to states
	Storage&	_experimentalDataStorage;

	// Marker Map information
	typedef struct
	{
		const SimmMarker* marker;
		const SimmBody* body;
		int experimentalColumn;
		double experimentalPosition[3];
		double computedPosition[3];
	} markerToSolve;
	Array<markerToSolve*> _markers;

	// Coordinate Map information
	int _numUnconstrainedQs; // number of unconstrained (unlocked) coordinates in model
	SimmCoordinateArray _unconstrainedQs; // array of unconstrained coordinates in model
	int* _unconstrainedQsIndices; // map from unconstrained coordinates into experimental data

	int _numPrescribedQs; // number of coordinates that are locked AND that are specified in _experimentalDataStorage
	SimmCoordinateArray _prescribedQs; // array of locked+specified coordinates in model
	int* _prescribedQsIndices; // map from locked+specified coordinates into experimental data

//==============================================================================
// METHODS
//==============================================================================
private:
	void buildMarkerMap(const Array<std::string>& aNameArray);
	void buildCoordinateMap(const Array<std::string>& aNameArray);

public:
	//---------------------------------------------------------------------------
	// CONSTRUCTION
	//---------------------------------------------------------------------------
	SimmInverseKinematicsTarget(SimmModel &aModel, Storage& aExperimentalDataStorage);

	virtual ~SimmInverseKinematicsTarget(void);

	//---------------------------------------------------------------------------
	// SET AND GET
	//---------------------------------------------------------------------------
	void setIndexToSolve(int aIndex, double* qGuess);
	void setPrescribedCoordinates(int aIndex);
	void getComputedMarkerLocations(Array<double> &aMarkerLocations) const;
	void getExperimentalMarkerLocations(Array<double> &aMarkerLocations) const;
	void getPrescribedQValues(Array<double>& aQValues) const;
	void getUnconstrainedCoordinateNames(Array<const std::string*>& aNameArray);
	void getPrescribedCoordinateNames(Array<const std::string*>& aNameArray);
	void getOutputMarkerNames(Array<const std::string*>& aNameArray);
	const SimmModel& getModel() { return _model; };
	//--------------------------------------------------------------------------
	// REQUIRED OPTIMIZATION TARGET METHODS
	//--------------------------------------------------------------------------
	// PERFORMANCE AND CONSTRAINTS
	int compute(double *x,double *p,double *c);
	int computeGradients(double *dx,double *x,double *dpdx,double *dcdx);
	// PERFORMANCE
	int computePerformance(double *x,double *p);
	int computePerformanceGradient(double *x,double *dpdx);
	// CONSTRAINTS
	int computeConstraint(double *x,int i,double *c);
	int computeConstraintGradient(double *x,int i,double *dcdx);
};

}; //namespace

#endif