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
#include <OpenSim/Common/Exception.h>
#include <OpenSim/Simulation/Model/DerivCallbackSet.h>

#include <OpenSim/Common/rdMath.h>
#include <OpenSim/Simulation/Model/AbstractActuator.h>
#include <OpenSim/Simulation/Model/AbstractMuscle.h>

#include "rdActuatorForceTargetFast.h"
#include "rdCMC_TaskSet.h"
#include "rdCMC.h"

#include <OpenSim/Common/Storage.h>

using namespace std;
using namespace OpenSim;
using SimTK::Real;
using SimTK::Vector;
using SimTK::Matrix;

#define USE_LINEAR_CONSTRAINT_MATRIX

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
	rdOptimizationTarget(aNX), _controller(aController)
{
	// NUMBER OF CONTROLS
	if(getNumParameters()<=0) {
		throw(Exception("rdActuatorForceTargetFast: ERROR- no controls.\n"));
	}

	// MODEL
	Model *model = _controller->getModel();
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
	_recipOptForceSquared.setSize(na);
	_recipAvgActForceRangeSquared.setSize(na);

	
	int nConstraints = _controller->getTaskSet()->getNumActiveTaskFunctions();

	// NUMBERS OF CONSTRAINTS
	// There are only linear equality constraints.
	setNumEqualityConstraints(nConstraints);
	setNumLinearEqualityConstraints(nConstraints);

	// DERIVATIVE PERTURBATION SIZES;
	setDX(1.0e-6);

	// COMPUTE ACTUATOR AREAS
	Array<double> f(1.0,na);
	ActuatorSet *actuatorSet = model->getActuatorSet();
	for(int i=0;i<na;i++) {
		actuatorSet->get(i)->setForce(f[i]);
		_recipAreaSquared[i] = actuatorSet->get(i)->getStress();
		_recipAreaSquared[i] *= _recipAreaSquared[i];
	}
}


//==============================================================================
// CONSTRUCTION AND DESTRUCTION
//==============================================================================
bool rdActuatorForceTargetFast::
prepareToOptimize(double *x)
{
#ifdef USE_LINEAR_CONSTRAINT_MATRIX
	//cout<<"Computing linear constraint matrix..."<<endl;
	Model *model = _controller->getModel();
	int nf = model->getNumActuators();
	int nc = getNumConstraints();

	_constraintMatrix.resize(nc,nf);
	_constraintVector.resize(nc);

	Vector f(nf), c(nc);

	// Build linear constraint matrix and constant constraint vector
	f = 0;
	computeConstraintVector(f, _constraintVector);

	for(int j=0; j<nf; j++) {
		f[j] = 1;
		computeConstraintVector(f, c);
		for(int i=0; i<nc; i++) _constraintMatrix(i,j) = (c[i] - _constraintVector[i]);
		f[j] = 0;
	}
#endif

	// COMPUTE MAX ISOMETRIC FORCE
	ActuatorSet *actSet = model->getActuatorSet();
	Array<double> y(0.0,model->getNumStates());
	model->getStates(&y[0]);
	AbstractActuator *act;
	AbstractMuscle *mus;
	double fOpt;
	int na = model->getNumActuators();
	for(int i=0;i<na;i++) {
		act = actSet->get(i);
		mus = dynamic_cast<AbstractMuscle*>(act);
		if(mus==NULL) {
			fOpt = 1.0e-4;
		} else {
			double activation = 1.0;
			fOpt = mus->computeIsokineticForceAssumingInfinitelyStiffTendon(activation);
			if(rdMath::IsZero(fOpt)) fOpt = 1.0e-4;
		}
		_recipOptForceSquared[i] = 1.0 / (fOpt*fOpt);
	}
	model->setStates(&y[0]);


	// return false to indicate that we still need to proceed with optimization (did not do a lapack direct solve)
	return false;
}

//==============================================================================
// SET AND GET
//==============================================================================


//==============================================================================
// PERFORMANCE AND CONSTRAINTS
//==============================================================================
//------------------------------------------------------------------------------
// PERFORMANCE
//------------------------------------------------------------------------------
//______________________________________________________________________________
/**
 * Compute performance given x.
 *
 * @param aF Vector of controls.
 * @param rP Value of the performance criterion.
 * @return Status (normal termination = 0, error < 0).
 */
int rdActuatorForceTargetFast::
objectiveFunc(const Vector &aF, const bool new_coefficients, Real& rP) const
{
	Model *model = _controller->getModel();
	ActuatorSet *actSet = model->getActuatorSet();
	AbstractActuator *act;
	AbstractMuscle *mus;
	int na = model->getNumActuators();
	double p = 0.0;
	for(int i=0;i<na;i++) {
		act = actSet->get(i);
		mus = dynamic_cast<AbstractMuscle*>(act);
		if(mus==NULL) {
			p +=  aF[i] * aF[i] *  _recipAreaSquared[i];
		} else {
			p +=  aF[i] * aF[i] * _recipOptForceSquared[i];
		}
	}
	rP = p;

	return(0);
}
//______________________________________________________________________________
/**
 * Compute the gradient of performance given x.
 *
 * @param x Vector of controls.
 * @param dpdx Derivatives of performance with respect to the controls.
 * @return Status (normal termination = 0, error < 0).
 */
int rdActuatorForceTargetFast::
gradientFunc(const Vector &x, const bool new_coefficients, Vector &gradient) const
{
	Model *model = _controller->getModel();
	ActuatorSet *actSet = model->getActuatorSet();
	AbstractActuator *act;
	AbstractMuscle *mus;
	int na = model->getNumActuators();
	for(int i=0;i<na;i++) {
		act = actSet->get(i);
		mus = dynamic_cast<AbstractMuscle*>(act);
		if(mus==NULL) {
			gradient[i] =  2.0 * x[i] * _recipAreaSquared[i];
		} else {
			gradient[i] =  2.0 * x[i] * _recipOptForceSquared[i];
		}
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
constraintFunc(const SimTK::Vector &x, const bool new_coefficients, SimTK::Vector &constraints) const
{
#ifndef USE_LINEAR_CONSTRAINT_MATRIX

	// Evaluate constraint function for all constraints and pick the appropriate component
	computeConstraintVector(x,constraints);

#else

	// Use precomputed constraint matrix
	//cout<<"Computing constraints assuming linear dependence..."<<endl;
	constraints = _constraintMatrix * x + _constraintVector;

#endif

	return(0);
}
//______________________________________________________________________________
/**
 * Compute all constraints given x.
 */
void rdActuatorForceTargetFast::
computeConstraintVector(const Vector &x,Vector &c) const
{
	Model *model = _controller->getModel();
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
	for(int i=0;i<nf;i++) {
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
	for(int i=0; i<getNumConstraints(); i++)
		c[i]=w[i]*(aDes[i]-a[i]);
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
constraintJacobian(const SimTK::Vector &x, const bool new_coefficients, SimTK::Matrix &jac) const
{
#ifndef USE_LINEAR_CONSTRAINT_MATRIX

	// Compute gradient using callbacks to constraintFunc
	rdOptimizationTarget::CentralDifferencesConstraint(this,&_dx[0],x,jac);

#else

	// Use precomputed constraint matrix (works if constraint is linear)
	//cout<<"Computing constraint gradient assuming linear dependence..."<<endl;
	jac = _constraintMatrix;

#endif

	return 0;
}
