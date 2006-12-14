// rdInitialStatesTarget.cpp
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


//==============================================================================
// INCLUDES
//==============================================================================
#include "rdInitialStatesTarget.h"
#include <iostream>
#include <OpenSim/Tools/Exception.h>
#include <OpenSim/Simulation/Control/ControlConstant.h>

using namespace std;
using namespace OpenSim;


//==============================================================================
// DESTRUCTOR & CONSTRUCTIOR(S)
//==============================================================================
//______________________________________________________________________________
/**
 * Destructor.
 */
rdInitialStatesTarget::~rdInitialStatesTarget()
{
}
//______________________________________________________________________________
/**
 * Constructor.
 *
 * @param aNX Number of controls.
 * @param aTI Initial time for the simulation.
 * @param aYI Initial states for the simulation.
 * @param aController Controller for the simulation.
 * @param aEquilib Integrand for the state equlibrium integration.
 */
rdInitialStatesTarget::
rdInitialStatesTarget(int aNX,double aTI,const double *aYI,
							 rdCMC *aController,
							 ModelIntegrandForActuators *aEquilib) :
	rdOptimizationTarget(aNX),
	_yi(0.0), _y(0.0), _dydt(0.0)
{
	setNull();

	// CONTROLLER
	_controller = aController;
	AbstractModel *model = _controller->getModel();
	if(model==NULL) {
		throw(Exception("rdInitialStatesTarget: ERROR- no model.\n"));
	}

	// TIME
	_ti = aTI;
	_tf = _ti + _controller->getTargetDT();
	_tfEqui = _ti + 0.100;

	// CONSTRUCT EQUILIBIRUM INTEGRATOR
	_equi = aEquilib;
	_integEqui = new IntegRKF(_equi,1.0e-5,5.0e-7);
	_integEqui->setMaxDT(0.001);

	// CONSTRUCT FORWARD INTEGRAND
	_forw = new ModelIntegrand(model);
	_integForw = new IntegRKF(_forw,1.0e-5,5.0e-7);
	_integForw->setMaxDT(0.001);

	// SET UP CONTROL SETS
	// Using constant controls.
	int i;
	int nxModel = model->getNumControls();
	ControlSet xSet;
	for(i=0;i<nxModel;i++) {
		ControlConstant *x = new ControlConstant();
		x->setIsModelControl(true);
		xSet.append(x);
	}
	cout<<"Constant control set constructed (size="<<xSet.getSize()<<").\n";
	_equi->setControlSet(xSet);
	_forw->setControlSet(xSet);

	// SIZE WORK ARRAYS
	int ny = model->getNumStates();
	_yi.setSize(ny);
	_y.setSize(ny);
	_dydt.setSize(ny);

	// SET THE INITIAL STATES
	// This array should not change.
	for(i=0;i<ny;i++) {
		_yi[i] = aYI[i];
	}

	// NUMBERS OF CONSTRAINTS
	_nineqn = 0;
	_nineq = 0;
	_neqn = 0;
	_neq = 0;

	// DERIVATIVE PERTURBATION SIZES;
	setDX(1.0e-6);
}


//==============================================================================
// CONSTRUCTION AND DESTRUCTION
//==============================================================================
//______________________________________________________________________________
/**
 * Set all member variables to their NULL values.
 */
void rdInitialStatesTarget::
setNull()
{
	_ti = 0.0;
	_tf = 0.0;
	_tfEqui = 0.0;
	_controller = NULL;
	_equi = NULL;
	_forw = NULL;
	_integEqui = NULL;
	_integForw = NULL;
}


//==============================================================================
// UTILITY
//==============================================================================
//______________________________________________________________________________
/**
 * Generate equilibrium actuator states given a set of controls.
 * @param aX Controls.
 * @param rY Actuator states (&y[nq+nu] should be sent in).
 */
void rdInitialStatesTarget::
generateEquilibriumStates(const double *aX,double *rY)
{
	// SET CONTROLS
	ControlSet *xEqui = _equi->getControlSet();
	xEqui->setControlValues(_ti,aX);

	// INTEGRATE SEVERAL TIMES
	int i;
	for(i=0;i<2;i++) {
		_integEqui->integrate(_ti,_tfEqui,rY,0.00001);
	}
}


//==============================================================================
// PERFORMANCE AND CONSTRAINTS
//==============================================================================
//------------------------------------------------------------------------------
// PERFORMANCE AND CONSTRAINTS
//------------------------------------------------------------------------------
//______________________________________________________________________________
/**
 * Compute performance and the constraints given x.
 * Note - used by paramopt.
 */
int rdInitialStatesTarget::
compute(double *x,double *p,double *c)
{
	int status = 0;
	return(status);
}
//______________________________________________________________________________
/**
 * Compute the gradients of the performance and the constraints given x.
 * The array dx is an array of perturbation sizes which can be used to
 * compute the gradients numerically.
 *
 * Note- used by paramopt.
 */
int rdInitialStatesTarget::
computeGradients(double *dx,double *x,double *dpdx,double *dcdx)
{
	int status = 0;
	return(status);
}

//------------------------------------------------------------------------------
// PERFORMANCE
//------------------------------------------------------------------------------
//______________________________________________________________________________
/**
 * Compute performance given x.
 */
int rdInitialStatesTarget::
computePerformance(double *aX,double *rP)
{
	int i;

	// MODEL
	AbstractModel *model = _controller->getModel();
	rdCMC_TaskSet *taskSet = _controller->getTaskSet();

	// TIME STUFF
	double window = 0.100;
	double timeNorm = model->getTimeNormConstant();
	double tfReal = _tf * timeNorm;

	// GET EQUILIBRIUM ACTUATOR STATES
	// Always begin from the starting initial states.
	_y = _yi;
	int nqnu = model->getNumCoordinates() + model->getNumSpeeds();
	generateEquilibriumStates(aX,&_y[nqnu]);

	// INTEGRATE FORWARD
	int nxModel = model->getNumControls();
	ControlSet *xForw = _forw->getControlSet();
	xForw->setControlValues(_ti,&aX[nxModel]);
	_integForw->integrate(_ti,_tf,&_y[0],0.00001);

	// TASK ERRORS
	_forw->compute(_tf,&_y[0],&_dydt[0]);
	taskSet->computeErrors(tfReal);
	taskSet->computeAccelerations();
	Array<double> &w = taskSet->getWeights();
	Array<double> &kp = taskSet->getPositionGains();
	Array<double> &kv = taskSet->getVelocityGains();
	Array<double> &pErr = taskSet->getPositionErrors();
	Array<double> &vErr = taskSet->getVelocityErrors();
	Array<double> &aTask = taskSet->getTaskAccelerations(tfReal);
	Array<double> &a = taskSet->getAccelerations();

	// PERFORMANCE
	int n = a.getSize();
	double perf_x,perf_task,perf_p,perf_v,perf_a,perf_kin;
	for(perf_x=0.0,i=0;i<_nx;i++) {
		perf_x += aX[i] * aX[i];
	}
	for(perf_task=0.0,i=0;i<n;i++) {
		perf_p = kp[i]*pErr[i]*pErr[i];
		perf_v = kv[i]*vErr[i]*vErr[i];
		perf_a = (aTask[i]-a[i])*(aTask[i]-a[i]);
		perf_kin = perf_p + perf_v + perf_a;
		perf_task += w[i]*perf_kin;
	}
	*rP = perf_x + perf_task;

	cout<<"perf = "<<*rP<<endl;
	return(0);
}
//______________________________________________________________________________
/**
 * Compute the gradient of performance given x.
 */
int rdInitialStatesTarget::
computePerformanceGradient(double *x,double *dpdx)
{
	int status = rdFSQP::CentralDifferences(this,_dx,x,dpdx);
	return(status);
}

//------------------------------------------------------------------------------
// CONSTRAINT
//------------------------------------------------------------------------------
//______________________________________________________________________________
/**
 * Compute constraint i given x.
 * Note that the indexing starts at 1;
 */
int rdInitialStatesTarget::
computeConstraint(double *x,int ic,double *c)
{
	int status = 0;
	return(status);
}
//______________________________________________________________________________
/**
 * Compute the gradient of constraint i given x.
 */
int rdInitialStatesTarget::
computeConstraintGradient(double *x,int ic,double *dcdx)
{
	// COMPUTE GRADIENT
	int status = rdFSQP::CentralDifferencesConstraint(this,_dx,x,ic,dcdx);
	return(0);
}

