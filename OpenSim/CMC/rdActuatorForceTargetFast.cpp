// rdActuatorForceTargetFast.cpp
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
#include <iostream>
#include <OpenSim/Tools/Exception.h>
#include <OpenSim/Simulation/Model/DerivCallbackSet.h>
#include "rdActuatorForceTargetFast.h"

using namespace std;
using namespace OpenSim;


//==============================================================================
// DESTRUCTOR & CONSTRUCTIOR(S)
//==============================================================================
//______________________________________________________________________________
/**
 * Destructor.
 */
rdActuatorForceTargetFast::~rdActuatorForceTargetFast()
{
}
//______________________________________________________________________________
/**
 * Constructor.
 *
 * @param aNX Number of controls.
 * @param aController Parent controller.
 */
rdActuatorForceTargetFast::
rdActuatorForceTargetFast(int aNX,rdCMC *aController):
	rdOptimizationTarget(aNX),
	_modelControls(0.0), _y(0.0), _dydt(0.0), _dqdt(0.0), _dudt(0.0),
	_recipAreaSquared(0.0)
{
	setNull();

	// CONTROLLER
	_controller = aController;

	// NUMBER OF CONTROLS
	_nx = aNX;
	if(_nx<=0) {
		throw(Exception("rdActuatorForceTargetFast: ERROR- no controls.\n"));
	}

	// MODEL
	AbstractModel *model = _controller->getModel();
	if(model==NULL) {
		throw(Exception("rdActuatorForceTargetFast: ERROR- no model.\n"));
	}

	// ALLOCATE STATE ARRAYS
	int nx = model->getNumControls();
	int ny = model->getNumStates();
	int nq = model->getNumCoordinates();
	int nu = model->getNumSpeeds();
	int na = model->getNumActuators();
	_modelControls.setSize(nx);
	_y.setSize(ny);
	_dydt.setSize(ny);
	_dqdt.setSize(nq);
	_dudt.setSize(nu);
	_recipAreaSquared.setSize(na);

	// PARSE TASK SET FOR NUMBER OF CONSTRAINTS
	int i,j,n;
	rdCMC_Task *task;
	rdCMC_TaskSet *taskSet = _controller->getTaskSet();
	int size = taskSet->getSize();
	int nConstraints;
	for(nConstraints=i=0;i<size;i++) {
		task = taskSet->get(i);
		if(task==NULL) continue;

		n = task->getNumTaskFunctions();
		for(j=0;j<n;j++) {
			if(task->getActive(j)) nConstraints++;
		}
	}

	// NUMBERS OF CONSTRAINTS
	// There are only linear equality constraints.
	_nineqn = 0;
	_nineq = 0;
	_neqn = 0;
	_neq = nConstraints;

	// DERIVATIVE PERTURBATION SIZES;
	setDX(1.0e-6);

	// COMPUTE ACTUATOR AREAS
	Array<double> f(1.0,_nx);
	ActuatorSet *actuatorSet = model->getActuatorSet();
	for(i=0;i<na;i++) {
		actuatorSet->get(i)->setForce(f[i]);
		_recipAreaSquared[i] = actuatorSet->get(i)->getStress();
		_recipAreaSquared[i] *= _recipAreaSquared[i];
	}
}


//==============================================================================
// CONSTRUCTION AND DESTRUCTION
//==============================================================================
//______________________________________________________________________________
/**
 * Set all member variables to their NULL values.
 */
void rdActuatorForceTargetFast::
setNull()
{
	_controller = NULL;
}


//==============================================================================
// SET AND GET
//==============================================================================


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
 *
 * @param x Array of controls.
 * @param p Value of the performance criterion.
 * @param c Array of constraint values.
 * @return Status (normal termination = 0, error < 0).
 */
int rdActuatorForceTargetFast::
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
 *
 * @param dx Array of perturbations for numerical derivatives.
 * @param x Array of controls.
 * @param dpdx Derivative of the performance criterion with respect to
 * the controls.
 * @param dcdx Matrix of derivatives of the constraints with respect
 * to the controls.
 * @return Status (normal termination = 0, error < 0).
 */
int rdActuatorForceTargetFast::
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
 *
 * @param aF Array of controls.
 * @param rP Value of the performance criterion.
 * @return Status (normal termination = 0, error < 0).
 */
int rdActuatorForceTargetFast::
computePerformance(double *aX,double *rP)
{
	int i;
	int nx = getNumControls();
	double p;
	for(p=0.0,i=0;i<nx;i++) {
		p += _recipAreaSquared[i] * aX[i] * aX[i];
	}
	*rP = p;

	return(0);
}
//______________________________________________________________________________
/**
 * Compute the gradient of performance given x.
 *
 * @param x Array of controls.
 * @param dpdx Derivatives of performance with respect to the controls.
 * @return Status (normal termination = 0, error < 0).
 */
int rdActuatorForceTargetFast::
computePerformanceGradient(double *x,double *dpdx)
{
	int i;
	int nx = getNumControls();
	for(i=0;i<nx;i++) {
		dpdx[i] = 2.0 * _recipAreaSquared[i] * x[i];
	}

	return(0);
}

//------------------------------------------------------------------------------
// CONSTRAINT
//------------------------------------------------------------------------------
//______________________________________________________________________________
/**
 * Compute constraint ic given x.
 * Note that the indexing starts at 1;
 *
 * @param x Array of controls.
 * @param ic Index of the constraint (indexing starts at 1, not 0).
 * @param c Value of constraint ic.
 * @return Status (normal termination = 0, error < 0).
 */
int rdActuatorForceTargetFast::
computeConstraint(double *x,int ic,double *c)
{
	int i;

	AbstractModel *model = _controller->getModel();
	rdCMC_TaskSet *taskSet = _controller->getTaskSet();

	// TIME STUFF
	//double timeNorm = model->getTimeNormConstant();
	double t = model->getTime();
	//double tReal = t * timeNorm;

	// SET
	model->getStates(&_y[0]);
	model->setStates(&_y[0]);
	model->getDerivCallbackSet()->set(t,&_modelControls[0],&_y[0]);

	// ACTUATION
	model->getActuatorSet()->computeActuation();
	int nf = model->getNumActuators();
	model->getDerivCallbackSet()->computeActuation(t,&_modelControls[0],&_y[0]);
	ActuatorSet *actuatorSet = model->getActuatorSet();
	for(i=0;i<nf;i++) {
		actuatorSet->get(i)->setForce(x[i]);
	}
	model->getActuatorSet()->apply();
	model->getDerivCallbackSet()->applyActuation(t,&_modelControls[0],&_y[0]);

	// CONTACT
	model->getContactSet()->computeContact();
	model->getDerivCallbackSet()->computeContact(t,&_modelControls[0],&_y[0]);
	model->getContactSet()->apply();
	model->getDerivCallbackSet()->applyContact(t,&_modelControls[0],&_y[0]);

	// ACCELERATIONS
	model->getDynamicsEngine().computeDerivatives(&_dqdt[0],&_dudt[0]);
	model->getDerivCallbackSet()->computeDerivatives(t,&_modelControls[0],&_y[0],&_dydt[0]);
	taskSet->computeAccelerations();
	Array<double> &w = taskSet->getWeights();
	Array<double> &aDes = taskSet->getDesiredAccelerations();
	Array<double> &a = taskSet->getAccelerations();

	// CONSTRAINTS
	i = ic - 1;
	*c = w[i]*(aDes[i]-a[i]);
	//cout<<"Constraint: ic="<<ic<<" i="<<i<<" *c="<<*c<<endl;

/*
	// SET
	_model->setConfiguration(&_q[0],&_u[0]);

	// ACTUATION
	_model->computeActuation();
	int i,nx=getNX();
	for(i=0;i<nx;i++) {
		_model->setActuatorForce(i,x[i]);
	}
	_model->applyActuatorForces();

	// CONTACT
	_model->computeContact();
	_model->applyContactForces();

	// COMPUTE ACCELERATIONS
	_model->computeAccelerations(&_dqdt[0],&_dudt[0]);

	// EVALUATE CONSTRAINTS
	int j,nu = _model->getNU();
	for(i=j=0;i<nu;i++) {
		if(_constrained[i]) {
			c[j] = _dudt[i] - _dudtDesired[i];
		}
	}
*/
	return(0);
}
//______________________________________________________________________________
/**
 * Compute the gradient of constraint i given x.
 *
 * @param x Array of controls.
 * @param ic Index of the constraint (indexing starts at 1, not 0).
 * @param dcdx Derivative of constraint ic with respect to the controls.
 * @return Status (normal termination = 0, error < 0).
 */
int rdActuatorForceTargetFast::
computeConstraintGradient(double *x,int ic,double *dcdx)
{
	// COMPUTE GRADIENT
	rdFSQP::CentralDifferencesConstraint(this,_dx,x,ic,dcdx);
	return(0);
}

