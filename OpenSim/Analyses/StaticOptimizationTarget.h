#ifndef _StaticOptimizationTarget_h_
#define _StaticOptimizationTarget_h_
/* -------------------------------------------------------------------------- *
 *                    OpenSim:  StaticOptimizationTarget.h                    *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Jeffrey A. Reinbolt                                             *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

//=============================================================================
// INCLUDES
//=============================================================================
#include "osimAnalysesDLL.h"
#include "OpenSim/Common/Array.h"
#include <OpenSim/Common/GCVSplineSet.h>
#include "SimTKsimbody.h"
#include <simmath/Optimizer.h>

//=============================================================================
//=============================================================================
namespace OpenSim { 

/**
 * This class provides an interface specification for static optimization Objective Function.
 *
 * @author Jeff Reinbolt
 */
class OSIMANALYSES_API StaticOptimizationTarget : public SimTK::OptimizerSystem
{
//=============================================================================
// DATA
//=============================================================================
public:
	/** Smallest allowable perturbation size for computing derivatives. */
	static const double SMALLDX;
	/** . */
private:

	/** Work model. */
	Model *_model;
	/** Current state */
	const SimTK::State* _currentState;
	/** Reciprocal of actuator area squared. */
	Array<double> _recipAreaSquared;
	/** Reciprocal of optimal force squared accounting for force-length curve if actuator is a muscle. */
	Array<double> _recipOptForceSquared;
	/** Optimal force accounting for force-length curve if desired and if actuator is a muscle. */
	Array<double> _optimalForce;
	
	SimTK::Matrix _constraintMatrix;
	SimTK::Vector _constraintVector;

	const Storage *_statesStore;
	GCVSplineSet _statesSplineSet;

protected:
	double _activationExponent;
	bool   _useMusclePhysiology;
	/** Perturbation size for computing numerical derivatives. */
	Array<double> _dx;
	Array<int> _accelerationIndices;

//=============================================================================
// METHODS
//=============================================================================
public:
	StaticOptimizationTarget(const SimTK::State& s, Model *aModel,int aNX,int aNC, const bool useMusclePhysiology=true);

	// SET AND GET
	void setModel(Model& aModel);
	void setStatesStore(const Storage *aStatesStore);
	void setStatesSplineSet(GCVSplineSet aStatesSplineSet);
	void setNumParams(const int aNP);
	void setNumConstraints(const int aNC);
	void setDX(double aVal);
	void setDX(int aIndex,double aVal);
	double getDX(int aIndex);
	double* getDXArray();
	void getActuation(SimTK::State& s, const SimTK::Vector &parameters, SimTK::Vector &forces);
	void setActivationExponent(double aActivationExponent) { _activationExponent=aActivationExponent; }
	double getActivationExponent() const { return _activationExponent; }
	void setCurrentState( const SimTK::State* state) { _currentState = state; }
	const SimTK::State* getCurrentState() const { return _currentState; }

	// UTILITY
	void validatePerturbationSize(double &aSize);

	virtual void printPerformance(const SimTK::State& s, double *x);

	void computeActuatorAreas(const SimTK::State& s);

	static int
		CentralDifferencesConstraint(const StaticOptimizationTarget *aTarget,
		double *dx,const SimTK::Vector &x,SimTK::Matrix &jacobian);
	static int
		CentralDifferences(const StaticOptimizationTarget *aTarget,
		double *dx,const SimTK::Vector &x,SimTK::Vector &dpdx);

	bool prepareToOptimize(SimTK::State& s, double *x);

	//--------------------------------------------------------------------------
	// REQUIRED OPTIMIZATION TARGET METHODS
	//--------------------------------------------------------------------------
	int objectiveFunc(const SimTK::Vector &x, bool new_coefficients, SimTK::Real& rP) const;
	int gradientFunc(const SimTK::Vector &x, bool new_coefficients, SimTK::Vector &gradient) const;
	int constraintFunc(const SimTK::Vector &x, bool new_coefficients, SimTK::Vector &constraints) const;
	int constraintJacobian(const SimTK::Vector &x, bool new_coefficients, SimTK::Matrix &jac) const;

private:
	void computeConstraintVector(SimTK::State& s, const SimTK::Vector &x, SimTK::Vector &c) const;
	void computeAcceleration(SimTK::State& s, const SimTK::Vector &aF,SimTK::Vector &rAccel) const;
	void cumulativeTime(double &aTime, double aIncrement);
};

}; //namespace

#endif // _StaticOptimizationTarget_h_
