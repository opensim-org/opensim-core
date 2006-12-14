// rdInitialStatesTarget.h
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
#ifndef rdInitialStatesTarget_h__
#define rdInitialStatesTarget_h__

#include "rdCMCDLL.h"
#include <OpenSim/Tools/rdTools.h>
#include <OpenSim/Tools/rdMath.h>
#include <OpenSim/SQP/rdFSQP.h>
#include <OpenSim/SQP/rdOptimizationTarget.h>
#include <OpenSim/Simulation/Model/ModelIntegrandForActuators.h>
#include "rdCMC.h"

namespace OpenSim {

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/**
 * An optimization target for establishing initial states appropriate for
 * a starting point for Computed Muscle Control (CMC).
 * 
 * @version 1.0
 * @author Frank C. Anderson
 */
class RDCMC_API rdInitialStatesTarget : public rdOptimizationTarget
{
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//==============================================================================
// DATA
//==============================================================================
private:
	/** Initial time for the simulation. */
	double _ti;
	/** Final time for the forward integration. */
	double _tf;
	/** Final time for the equilibrium integration. */
	double _tfEqui;
	/** Controller. */
	rdCMC *_controller;
	/* Integrand for establishing equilibrium values for the initial states. */
	ModelIntegrandForActuators *_equi;
	/** Integrator for producing equilibrium values for the initial states. */
	IntegRKF *_integEqui;
	/** Integrand for integrating forward to the next target time. */
	ModelIntegrand *_forw;
	/** Integrator for integrating forward to the next target time. */
	IntegRKF *_integForw;

	/** Work array of the initial states. */
	Array<double> _yi;
	/** Work array of the states. */
	Array<double> _y;
	/** Array of the model state derivatives. */
	Array<double> _dydt;

//==============================================================================
// METHODS
//==============================================================================
public:
	//---------------------------------------------------------------------------
	// CONSTRUCTION
	//---------------------------------------------------------------------------
	virtual ~rdInitialStatesTarget();
	rdInitialStatesTarget(int aNX,double aTI,const double *aY,
		rdCMC *aController,ModelIntegrandForActuators *aEquilib);
private:
	void setNull();

	//---------------------------------------------------------------------------
	// UTILITY
	//---------------------------------------------------------------------------
public:
	void generateEquilibriumStates(const double *aX,double *rY);

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
};	// END class rdInitialStatesTarget
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

}; // end namespace

#endif // rdInitialStatesTarget_h__
