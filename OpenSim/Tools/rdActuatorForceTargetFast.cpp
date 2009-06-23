// rdActuatorForceTargetFast.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c)  2005, Stanford University. All rights reserved. 
* Use of the OpenSim software in source form is permitted provided that the following
* conditions are met:
* 	1. The software is used only for non-commercial research and education. It may not
*     be used in relation to any commercial activity.
* 	2. The software is not distributed or redistributed.  Software distribution is allowed 
*     only through https://simtk.org/home/opensim.
* 	3. Use of the OpenSim software or derivatives must be acknowledged in all publications,
*      presentations, or documents describing work in which OpenSim or derivatives are used.
* 	4. Credits to developers may not be removed from executables
*     created from modifications of the source.
* 	5. Modifications of source code must retain the above copyright notice, this list of
*     conditions and the following disclaimer. 
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
*  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
*  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
*  SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
*  TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
*  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR BUSINESS INTERRUPTION) OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
*  WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
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
	int nc = 0; // number of actuators with one or more controls
	ActuatorSet *actuatorSet = model->getActuatorSet();
	for (int i=0; i<actuatorSet->getSize(); i++)
		if (actuatorSet->get(i)->getNumControls() > 0)
			nc++;

	_modelControls.setSize(nx);
	_y.setSize(ny);
	_dydt.setSize(ny);
	_dqdt.setSize(nq);
	_dudt.setSize(nu);
	_recipAreaSquared.setSize(nc);
	_recipOptForceSquared.setSize(nc);
	_recipAvgActForceRangeSquared.setSize(nc);

	
	int nConstraints = _controller->getTaskSet()->getNumActiveTaskFunctions();

	// NUMBERS OF CONSTRAINTS
	// There are only linear equality constraints.
	setNumEqualityConstraints(nConstraints);
	setNumLinearEqualityConstraints(nConstraints);

	// DERIVATIVE PERTURBATION SIZES;
	setDX(1.0e-6);

	// COMPUTE ACTUATOR AREAS
	Array<double> f(1.0,nc);
	for(int i=0,index=0;i<na;i++) {
		if (actuatorSet->get(i)->getNumControls() > 0) {
			actuatorSet->get(i)->setForce(f[index]);
			_recipAreaSquared[index] = actuatorSet->get(i)->getStress();
			_recipAreaSquared[index] *= _recipAreaSquared[index];
			index++;
		}
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
	for(int i=0,index=0;i<na;i++) {
		act = actSet->get(i);
		if (act->getNumControls() > 0) {
			mus = dynamic_cast<AbstractMuscle*>(act);
			if(mus==NULL) {
				fOpt = 1.0e-4;
			} else {
				double activation = 1.0;
				fOpt = mus->computeIsokineticForceAssumingInfinitelyStiffTendon(activation);
				if(rdMath::IsZero(fOpt)) fOpt = 1.0e-4;
			}
			_recipOptForceSquared[index++] = 1.0 / (fOpt*fOpt);
		}
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
	for(int i=0,index=0;i<na;i++) {
		act = actSet->get(i);
		if (act->getNumControls() > 0) {
			mus = dynamic_cast<AbstractMuscle*>(act);
			if(mus==NULL) {
				p +=  aF[index] * aF[index] *  _recipAreaSquared[index];
			} else {
				p +=  aF[index] * aF[index] * _recipOptForceSquared[index];
			}
			index++;
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
	for(int i=0,index=0;i<na;i++) {
		act = actSet->get(i);
		if (act->getNumControls() > 0) {
			mus = dynamic_cast<AbstractMuscle*>(act);
			if(mus==NULL) {
				gradient[index] =  2.0 * x[index] * _recipAreaSquared[index];
			} else {
				gradient[index] =  2.0 * x[index] * _recipOptForceSquared[index];
			}
			index++;
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
