/* -------------------------------------------------------------------------- *
 *                   OpenSim:  ActuatorForceTargetFast.cpp                    *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
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
//
// This software, originally developed by Realistic Dynamics, Inc., was
// transferred to Stanford University on November 1, 2006.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//==============================================================================
// INCLUDES
//==============================================================================
#include "ActuatorForceTargetFast.h"
#include "CMC_TaskSet.h"
#include "CMC.h"
#include "StateTrackingTask.h"

using namespace std;
using namespace OpenSim;
using SimTK::Real;
using SimTK::Vector;
using SimTK::Matrix;

#define USE_LINEAR_CONSTRAINT_MATRIX

//==============================================================================
// DESTRUCTOR & CONSTRUCTOR(S)
//==============================================================================
//______________________________________________________________________________
/**
 * Destructor.
 */
ActuatorForceTargetFast::~ActuatorForceTargetFast()
{
}
//______________________________________________________________________________
/**
 * Constructor.
 *
 * @param aNX Number of controls.
 * @param aController Parent controller.
 */
ActuatorForceTargetFast::
ActuatorForceTargetFast(SimTK::State& s, int aNX,CMC *aController):
    OptimizationTarget(aNX), _controller(aController)
{
    // NUMBER OF CONTROLS
    if(getNumParameters()<=0) {
        throw(Exception("ActuatorForceTargetFast: ERROR- no controls.\n"));
    }

    // ALLOCATE STATE ARRAYS
    int ny = _controller->getModel().getNumStateVariables();
    int nq = _controller->getModel().getNumCoordinates();
    int nu = _controller->getModel().getNumSpeeds();
    int na = _controller->getActuatorSet().getSize();

    _y.setSize(ny);
    _dydt.setSize(ny);
    _dqdt.setSize(nq);
    _dudt.setSize(nu);
    _recipAreaSquared.setSize(na);
    _recipOptForceSquared.setSize(na);
    _recipAvgActForceRangeSquared.setSize(na);
    
    int nConstraints = _controller->getTaskSet().getNumActiveTaskFunctions();

    // NUMBERS OF CONSTRAINTS
    // There are only linear equality constraints.
    setNumEqualityConstraints(nConstraints);
    setNumLinearEqualityConstraints(nConstraints);

    // DERIVATIVE PERTURBATION SIZES;
    setDX(1.0e-6);

    // COMPUTE ACTUATOR AREAS
    Array<double> f(1.0,na);
    const Set<const Actuator>& fSet = _controller->getActuatorSet();
    for(int i=0,j=0;i<fSet.getSize();i++) {
        auto act = dynamic_cast<const ScalarActuator*>(&fSet[i]);
        auto musc = dynamic_cast<const Muscle *>(act);
        if(musc)
            _recipAreaSquared[j] = f[j]/musc->getMaxIsometricForce();
        else
            _recipAreaSquared[j] = f[j]/act->getOptimalForce();
        
        _recipAreaSquared[j] *= _recipAreaSquared[j];
        j++;
    }
}


//==============================================================================
// CONSTRUCTION AND DESTRUCTION
//==============================================================================
bool ActuatorForceTargetFast::
prepareToOptimize(SimTK::State& s, double *x)
{
    // Keep around a "copy" of the state so we can use it in objective function 
    // in cases where we're tracking states
    _saveState = s;
#ifdef USE_LINEAR_CONSTRAINT_MATRIX
    int nf = _controller->getActuatorSet().getSize();
    int nc = getNumConstraints();

    _constraintMatrix.resize(nc,nf);
    _constraintVector.resize(nc);

    Vector f(nf), c(nc);

    // Build linear constraint matrix and constant constraint vector
    f = 0;

    computeConstraintVector(s, f, _constraintVector);

    for(int j=0; j<nf; j++) {
        f[j] = 1;
        computeConstraintVector(s, f, c);
        _constraintMatrix(j) = (c - _constraintVector);
        f[j] = 0;
    }
#endif

    // use temporary copy of state because computeIsokineticForceAssumingInfinitelyStiffTendon
    // will change the muscle states. This is necessary ONLY in the case of deprecated muscles
    SimTK::State tempState = s;
    double activation = 1.0;
    getController()->getModel().getMultibodySystem().realize( tempState, SimTK::Stage::Dynamics );

    // COMPUTE MAX ISOMETRIC FORCE
    const Set<const Actuator>& fSet = _controller->getActuatorSet();
    
    double fOpt = SimTK::NaN;

    getController()->getModel().getMultibodySystem().realize(tempState, SimTK::Stage::Dynamics );
    for(int i=0 ; i<fSet.getSize(); ++i) {
        auto act = dynamic_cast<const ScalarActuator*>(&fSet[i]);
        auto mus = dynamic_cast<const Muscle*>(act);
        if(mus==NULL) {
            fOpt = act->getOptimalForce();
        }
        else{   
            fOpt = mus->calcInextensibleTendonActiveFiberForce(tempState,
                                                              activation);
        }
        
        if( std::fabs(fOpt) < SimTK::TinyReal )
            fOpt = SimTK::TinyReal;

        _recipOptForceSquared[i] = 1.0 / (fOpt*fOpt);   
    }
    
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
int ActuatorForceTargetFast::
objectiveFunc(const Vector &aF, const bool new_coefficients, Real& rP) const
{
    const Set<const Actuator>& fSet = _controller->getActuatorSet();
    double p = 0.0;
    const CMC_TaskSet& tset=_controller->getTaskSet();
    for(int i=0,j=0;i<fSet.getSize();i++) {
        auto act = dynamic_cast<const ScalarActuator*>(&fSet[i]);
        auto mus = dynamic_cast<const Muscle*>(act);
        if(mus) {
            p +=  aF[j] * aF[j] * _recipOptForceSquared[j];
        } else {
            p +=  aF[j] * aF[j] *  _recipAreaSquared[j];
        }
        j++;
    }
    // double pre = p;
    // If tracking states, add in errors from them squared
    for(int t=0; t<tset.getSize(); t++){
        TrackingTask& ttask = tset.get(t);
        StateTrackingTask* stateTask=NULL;
        if ((stateTask=dynamic_cast<StateTrackingTask*>(&ttask))!= NULL){
            double err = stateTask->getTaskError(_saveState);
            //cout << "task error " << err << endl;
            p+= (err * err * stateTask->getWeight(0));

        }
    }
    rP = p;
    //cout << "Objective function" << rP << " vs. without emg " << pre << endl;

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
int ActuatorForceTargetFast::
gradientFunc(const Vector &x, const bool new_coefficients, Vector &gradient) const
{
    const Set<const Actuator>& fSet = _controller->getActuatorSet();
    // double p = 0.0;
    for(int i=0,index=0;i<fSet.getSize();i++) {
        auto act = dynamic_cast<const ScalarActuator*>(&fSet[i]);
        auto mus = dynamic_cast<const Muscle*>(act);
        if(mus) {
            gradient[index] =  2.0 * x[index] * _recipOptForceSquared[index];
        } else {
            gradient[index] =  2.0 * x[index] * _recipAreaSquared[index];
        }
        index++;
    }
//std::cout << "rdActuatorForceTargetFast::gradentFuncgradient =" << gradient << std::endl;
    // Add in the terms for the stateTracking
    const CMC_TaskSet& tset=_controller->getTaskSet();
    for(int t=0; t<tset.getSize(); t++){
        TrackingTask& ttask = tset.get(t);
        StateTrackingTask* stateTask=NULL;
        if ((stateTask=dynamic_cast<StateTrackingTask*>(&ttask))!= NULL){
            Vector errGradient = stateTask->getTaskErrorGradient(_saveState);
            gradient += errGradient;
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
 *
 * @param x Array of active forces.
 * @return Status (normal termination = 0, error < 0).
 */
int ActuatorForceTargetFast::
constraintFunc(const SimTK::Vector &x, const bool new_coefficients, SimTK::Vector &constraints) const
{
#ifndef USE_LINEAR_CONSTRAINT_MATRIX

    // Evaluate constraint function for all constraints and pick the appropriate component
    computeConstraintVector(s, x,constraints);

#else

    // Use precomputed constraint matrix
    constraints = _constraintMatrix * x + _constraintVector;
    //cout <<"x = " << x[0] <<" constraintEqn = " << constraints[0] << endl;

#endif
    return(0);
}
//______________________________________________________________________________
/**
 * Compute all constraints given x.
 */
void ActuatorForceTargetFast::
computeConstraintVector(SimTK::State& s, const Vector &x,Vector &c) const
{
    CMC_TaskSet&  taskSet = _controller->updTaskSet();
    const Set<const Actuator>& fSet = _controller->getActuatorSet();

    int nf = fSet.getSize();

    // Now override the actuator forces with computed active force
    // (from static optimization) but also include the passive force
    // contribution of muscles when applying forces to the model
    for(int i=0;i<nf;i++) {
        auto act = dynamic_cast<const ScalarActuator*>(&fSet[i]);
        act->overrideActuation(s, true);
        act->setOverrideActuation(s, x[i]);
    }
    _controller->getModel().getMultibodySystem().realize(s, SimTK::Stage::Acceleration );

    taskSet.computeAccelerations(s);
    Array<double> &w = taskSet.getWeights();
    Array<double> &aDes = taskSet.getDesiredAccelerations();
    Array<double> &a = taskSet.getAccelerations();

    // CONSTRAINTS
    for(int i=0; i<getNumConstraints(); i++)
        c[i]=w[i]*(aDes[i]-a[i]);

    // reset the actuator control 
    for(int i=0;i<fSet.getSize();i++) {
        auto act = dynamic_cast<const ScalarActuator*>(&fSet[i]);
        act->overrideActuation(s, false);
    }

    _controller->getModel().getMultibodySystem().realizeModel(s);
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
int ActuatorForceTargetFast::
constraintJacobian(const SimTK::Vector &x, const bool new_coefficients, SimTK::Matrix &jac) const
{
#ifndef USE_LINEAR_CONSTRAINT_MATRIX

    // Compute gradient using callbacks to constraintFunc
    OptimizationTarget::CentralDifferencesConstraint(this,&_dx[0],x,jac);

#else

    // Use precomputed constraint matrix (works if constraint is linear)
    jac = _constraintMatrix;

#endif

    return 0;
}
