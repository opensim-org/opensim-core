// rdActuatorForceTargetFast.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Copyright (c) 2006 Stanford University and Realistic Dynamics, Inc.
// Contributors: Frank C. Anderson
//
// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation files (the
// "Software"), to deal in the Software without restriction, including
// without limitation the rights to use, copy, modify, merge, publish,
// distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so, subject
// to the following conditions:
// 
// The above copyright notice and this permission notice shall be included
// in all copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
// EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
// PURPOSE AND NON-INFRINGEMENT. IN NO EVENT SHALL THE AUTHORS,
// CONTRIBUTORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
// TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH
// THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//
// This software, originally developed by Realistic Dynamics, Inc., was
// transferred to Stanford University on November 1, 2006.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#ifndef rdActuatorForceTargetFast_h__
#define rdActuatorForceTargetFast_h__


//==============================================================================
// INCLUDES
//==============================================================================
#include "osimToolsDLL.h"
#include <OpenSim/SQP/rdOptimizationTarget.h>
#include <SimTKcommon.h>

namespace OpenSim {

class rdCMC;

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/**
 * A Computed Muscle Control (CMC) optimization target for controlling 
 * dynamic systems whose actuators may be themselves governed by differential
 * equations, meaning there may be non-linear behavior and delays in force
 * production.
 *
 * The performance criterion is the sum of actutor stresses squared. The
 * desired accelerations are achieved by imposing a set of constraints.
 * The desired accelerations are computed according to the Proportional
 * Derivative (PD) control in order to drive the dynamic model toward a set
 * of target kinematic trajectories.
 *
 * Because the desired accelerations are achieved by imposing a set of
 * linear hard constraints, this optimiztion target can fail if the desired
 * accelreations cannot be achieved.  To insure that the constraints can
 * be achieved, a number of sufficiently strong actuators can be added
 * to the model.  Althernatively, one can use a different optimization
 * target @wee rdActuatorForceTarget.  The benefits of using the fast
 * target are both speed and tracking accuracy.
 * 
 * @version 1.0
 * @author Frank C. Anderson
 */
class OSIMTOOLS_API rdActuatorForceTargetFast : public rdOptimizationTarget
{
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//==============================================================================
// DATA
//==============================================================================
private:
	/** Model. */
	rdCMC *_controller;
	/** Work array of the model controls. */
	Array<double> _modelControls;
	/** Work array of the states. */
	Array<double> _y;
	/** Work array of the derivatives of the derivatives of the states. */
	Array<double> _dydt;
	/** Array of the derivatives of the generalized coordinates. */
	Array<double> _dqdt;
	/** Array of the derivatives of the generalized speeds. */
	Array<double> _dudt;
	/** Reciprocal of actuator area squared. */
	Array<double> _recipAreaSquared;

	SimTK::Matrix _constraintMatrix;
	SimTK::Vector _constraintVector;

//==============================================================================
// METHODS
//==============================================================================
public:
	//---------------------------------------------------------------------------
	// CONSTRUCTION
	//---------------------------------------------------------------------------
	virtual ~rdActuatorForceTargetFast();
	rdActuatorForceTargetFast(int aNX,rdCMC *aController);

private:
	void setNull();

public:

	bool prepareToOptimize(double *x);

	//--------------------------------------------------------------------------
	// REQUIRED OPTIMIZATION TARGET METHODS
	//--------------------------------------------------------------------------
	int objectiveFunc(const SimTK::Vector &aF, const bool new_coefficients, SimTK::Real& rP) const;
	int gradientFunc(const SimTK::Vector &x, const bool new_coefficients, SimTK::Vector &gradient) const;
	int constraintFunc(const SimTK::Vector &x, const bool new_coefficients, SimTK::Vector &constraints) const;
	int constraintJacobian(const SimTK::Vector &x, const bool new_coefficients, SimTK::Matrix &jac) const;

private:
	void computeConstraintVector(const SimTK::Vector &x, SimTK::Vector &c) const;

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
};	// END class rdActuatorForceTargetFast
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

}; // end namespace

#endif // rdActuatorForceTargetFast_h__
