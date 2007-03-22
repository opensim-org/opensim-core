#ifndef __IKTarget_h__
#define __IKTarget_h__
// IKTarget.h
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
#include "osimToolsDLL.h"
#include <OpenSim/SQP/rdOptimizationTarget.h>
#include <OpenSim/Common/Array.h>
#include <OpenSim/Simulation/Model/CoordinateSet.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/AbstractBody.h>
#include <OpenSim/Simulation/Model/AbstractMarker.h>

namespace OpenSim {

class Storage;
class IKTaskSet;

#ifdef SWIG
	#ifdef OSIMTOOLS_API
		#undef OSIMTOOLS_API
		#define OSIMTOOLS_API
	#endif
#endif

//=============================================================================
/**
 * A Class that represents an inverse kinematics optimization target. This solves the problem
 * of computing Qs that minimize the sum of the squared differences between a set of markers on a 
 * on the nominal model and passed in marker positions.
 *
 * @author Ayman Habib, Peter Loan
 * @version 1.0
 */

class OSIMTOOLS_API IKTarget : public rdOptimizationTarget
{
//==============================================================================
// DATA
//==============================================================================
private:
	Model& _model;

	// Task set contains weights and other properties affecting performance criterion
	IKTaskSet& _ikTaskSet;

	// Amount of perturbation used for derivative computation this should be an array
	static const double _perturbation;

	// internal datastructure used to map data columns to states
	Storage&	_experimentalDataStorage;

	// internal flag used for printing weighted errors at end of optimization
	bool _printPerformanceValues;

	// Marker Map information
	typedef struct
	{
		const AbstractMarker* marker;
		const AbstractBody* body;
		int experimentalColumn; // always >= 0
		double weight; // always nonzero
		// set each frame before optimization
		bool validExperimentalPosition;
		double experimentalPosition[3];
		// set each frame during optimization
		double computedPosition[3];
	} markerToSolve;

	// Coordinate Map information
	typedef struct
	{
		AbstractCoordinate* coord;
		bool prescribed; // if prescribed, we don't solve for it
		int experimentalColumn; // if task was from _file then this will be >=0
		double constantExperimentalValue; // if task was not from_file then this fixed value will be set
		double weight; // only matters for unprescribed coordinates
		// set each frame before optimization (for _unprescribedWeightedQs only)
		double experimentalValue;
	} coordinateInfo;

	// Only markers with nonzero weights are stored in this array
	Array<markerToSolve*> _markers;
	// The coordinates that are unprescribed (i.e. the ones computed by IK)
	Array<coordinateInfo*> _unprescribedQs;
	// Unprescribed coordinates with nonzero weight (the ones that appear in the performance criterion)
	Array<coordinateInfo*> _unprescribedWeightedQs;
	// The coordinates that are prescribed
	Array<coordinateInfo*> _prescribedQs;

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
	IKTarget(Model &aModel, IKTaskSet &aIKTaskSet, Storage& aExperimentalDataStorage);

	virtual ~IKTarget(void);

	//---------------------------------------------------------------------------
	// UTILITIES
	//---------------------------------------------------------------------------
	void prepareToSolve(int aIndex, double* qGuess);
	void printTasks() const;
	void printPerformance();
	//---------------------------------------------------------------------------
	// SET AND GET
	//---------------------------------------------------------------------------
	void getComputedMarkerLocations(Array<double> &aMarkerLocations) const;
	void getExperimentalMarkerLocations(Array<double> &aMarkerLocations) const;
	void getPrescribedCoordinateValues(Array<double>& aQValues) const;
	void getUnprescribedCoordinateNames(Array<std::string>& aNameArray);
	void getPrescribedCoordinateNames(Array<std::string>& aNameArray);
	void getOutputMarkerNames(Array<std::string>& aNameArray);
	Model& getModel() { return _model; };
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

} // namespace OpenSim

#endif
