// rdActuatorForceTarget.h
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
#ifndef rdActuatorForceTarget_h__
#define rdActuatorForceTarget_h__

#include "osimToolsDLL.h"
#include <OpenSim/Common/Array.h>
class OpenSim::rdCMC;

namespace OpenSim {

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/**
 * A Computed Muscle Control (CMC) optimization target for controlling 
 * dynamic systems whose actuators may be themselves governed by differential
 * equations, meaning there may be non-linear behavior and delays in force
 * production.
 *
 * The performance criterion is a sum of two terms.  The first term
 * is the sum of actutor stresses squared.  The second term is a weighted
 * sum of terms designed to achieve a set of desired accelerations that
 * will drive the dynamic model toward a set of target kinematic trajectories.
 * The desired accelerations are according to the Proportional Derivative (PD)
 * control law.
 *
 * Because the performance criterion is simply a long sum of things,
 * achieving the desired accelerations can be compromised in order to
 * reduce the forces (or moments) applied by the actutors.  This feature
 * is what is exploited by the Residual Reduction Algorithm.
 *
 * Although this target is fairly robust (meaning the optimizer should not
 * fail to find a solution), it is a bit slower and less accurate than
 * the "fast" target @see rdActuatorForceTargetFast.
 * 
 * @version 1.0
 * @author Frank C. Anderson
 */
class OSIMTOOLS_API rdActuatorForceTarget : public rdOptimizationTarget
{
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//==============================================================================
// DATA
//==============================================================================
private:
	/** Parent controller. */
	rdCMC *_controller;
	/** Work array of the controls. */
	Array<double> _x;
	/** Work array of the states. */
	Array<double> _y;
	/** Work array of the derivatives of the states. */
	Array<double> _dydt;
	/** Array of the derivatives of the generalized coordinates. */
	Array<double> _dqdt;
	/** Array of the derivatives of the generalized speeds. */
	Array<double> _dudt;
	/** Weight used to scale the stress term of objective function. **/
	double _stressTermWeight;

//==============================================================================
// METHODS
//==============================================================================
public:
	//---------------------------------------------------------------------------
	// CONSTRUCTION
	//---------------------------------------------------------------------------
	virtual ~rdActuatorForceTarget();
	rdActuatorForceTarget(int aNX,rdCMC *aController);
private:
	void setNull();

	//---------------------------------------------------------------------------
	// SET AND GET
	//---------------------------------------------------------------------------
public:
	void setStressTermWeight(double aWeight);

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

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
};	// END class rdActuatorForceTarget
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

}; // end namespace

#endif // rdActuatorForceTarget_h__
