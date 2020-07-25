/* -------------------------------------------------------------------------- *
 *                   OpenSim:  StaticOptimizationTarget.cpp                   *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Frank C. Anderson                                               *
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

/* Note: This code was originally developed by Realistic Dynamics Inc. 
 * Author: Frank C. Anderson 
 */


//=============================================================================
// INCLUDES
//=============================================================================
#include <OpenSim/Simulation/Model/Model.h>
#include "StaticOptimizationTarget.h"

using namespace OpenSim;
using namespace std;
using SimTK::Vector;
using SimTK::Matrix;
using SimTK::Real;

#define USE_LINEAR_CONSTRAINT_MATRIX

const double StaticOptimizationTarget::SMALLDX = 1.0e-14;
//const double StaticOptimizationTarget::_activationExponent = 2.0;
 
//==============================================================================
// CONSTRUCTOR
//==============================================================================
//______________________________________________________________________________
/**
 * Construct an optimization target.
 *
 * @param aNP The number of parameters.
 * @param aNC The number of constraints.
 * @param aT Current time in the integration.
 * @param aX Current control values.
 * @param aY Current states.
 * @param aDYDT Current state derivatives.
 */
StaticOptimizationTarget::
StaticOptimizationTarget(const SimTK::State& s, Model *aModel,int aNP,int aNC, bool useMusclePhysiology)
{
    // ALLOCATE STATE ARRAYS
    _recipAreaSquared.setSize(aNP);
    _recipOptForceSquared.setSize(aNP);
    _optimalForce.setSize(aNP);
    _useMusclePhysiology=useMusclePhysiology;

    setModel(*aModel);
    setNumParams(aNP);
    setNumConstraints(aNC);
    setActivationExponent(2.0);
    computeActuatorAreas(s);

    // Gather indices into speed set corresponding to the unconstrained degrees of freedom (for which we will set acceleration constraints)
    _accelerationIndices.setSize(0);
    auto coordinates = aModel->getCoordinatesInMultibodyTreeOrder();
    for (size_t i = 0u; i < coordinates.size(); ++i) {
        const Coordinate& coord = *coordinates[i];
        if(!coord.isConstrained(s)) {
            _accelerationIndices.append(static_cast<int>(i));
        }
    }
}

//==============================================================================
// CONSTRUCTION
//==============================================================================
bool StaticOptimizationTarget::
prepareToOptimize(SimTK::State& s, double *x)
{
    // COMPUTE MAX ISOMETRIC FORCE
    const ForceSet& fSet = _model->getForceSet();
    
    for(int i=0, j=0;i<fSet.getSize();i++) {
        ScalarActuator* act = dynamic_cast<ScalarActuator*>(&fSet.get(i));
         if( act ) {
             double fOpt;
             Muscle *mus = dynamic_cast<Muscle*>(&fSet.get(i));
             if( mus ) {
                //ActivationFiberLengthMuscle *aflmus = dynamic_cast<ActivationFiberLengthMuscle*>(mus);
                if(mus && _useMusclePhysiology) {
                    _model->setAllControllersEnabled(true);
                    fOpt = mus->calcInextensibleTendonActiveFiberForce(s, 1.0);
                    _model->setAllControllersEnabled(false);
                } else {
                    fOpt = mus->getMaxIsometricForce();
                }
             } else {
                  fOpt = act->getOptimalForce();
             }
            _optimalForce[j++] = fOpt;
         }
    }

#ifdef USE_LINEAR_CONSTRAINT_MATRIX
    //cout<<"Computing linear constraint matrix..."<<endl;
    int np = getNumParameters();
    int nc = getNumConstraints();

    _constraintMatrix.resize(nc,np);
    _constraintVector.resize(nc);

    Vector pVector(np), cVector(nc);

    // Build linear constraint matrix and constant constraint vector
    pVector = 0;
    computeConstraintVector(s, pVector,_constraintVector);

    for(int p=0; p<np; p++) {
        pVector[p] = 1;
        computeConstraintVector(s, pVector, cVector);
        for(int c=0; c<nc; c++) _constraintMatrix(c,p) = (cVector[c] - _constraintVector[c]);
        pVector[p] = 0;
    }
#endif

    // return false to indicate that we still need to proceed with optimization
    return false;
}
//==============================================================================
// SET AND GET
//==============================================================================
//------------------------------------------------------------------------------
// MODEL
//------------------------------------------------------------------------------
///______________________________________________________________________________
/**
 * Set the model.
 *
 * @param aModel Model.
 */
void StaticOptimizationTarget::
setModel(Model& aModel)
{
    _model = &aModel;
}
//------------------------------------------------------------------------------
// STATES STORAGE
//------------------------------------------------------------------------------
///______________________________________________________________________________
/**
 * Set the states storage.
 *
 * @param aStatesStore States storage.
 */
void StaticOptimizationTarget::
setStatesStore(const Storage *aStatesStore)
{
    _statesStore = aStatesStore;
}
//------------------------------------------------------------------------------
// STATES SPLINE SET
//------------------------------------------------------------------------------
///______________________________________________________________________________
/**
 * Set the states spline set.
 *
 * @param aStatesSplineSet States spline set.
 */
void StaticOptimizationTarget::
setStatesSplineSet(GCVSplineSet aStatesSplineSet)
{
    _statesSplineSet = aStatesSplineSet;
}

//------------------------------------------------------------------------------
// CONTROLS
//------------------------------------------------------------------------------
///______________________________________________________________________________
/**
 * Set the number of parameters.
 *
 * The number of parameters can be set at any time.  However, the perturbation
 * sizes for the parameters (i.e., _dx) is destroyed.  Therefore, the
 * perturbation sizes must be reset.
 *
 * @param aNP Number of parameters.
 * @see setDX()
 */
void StaticOptimizationTarget::
setNumParams(const int aNP)
{
    setNumParameters(aNP);
    _dx.setSize(getNumParameters());
}

//------------------------------------------------------------------------------
// CONSTRAINTS
//------------------------------------------------------------------------------
///______________________________________________________________________________
/**
 * Set the number of constraints.
 *
 * @param aNC Number of constraints.
 */
void StaticOptimizationTarget::
setNumConstraints(const int aNC)
{
    // There are only linear equality constraints.
    setNumEqualityConstraints(aNC);
    setNumLinearEqualityConstraints(aNC);
}   

//------------------------------------------------------------------------------
// DERIVATIVE PERTURBATION SIZES
//------------------------------------------------------------------------------
//______________________________________________________________________________
/**
 * Set the derivative perturbation size.
 */
void StaticOptimizationTarget::
setDX(int aIndex,double aValue)
{
    // VALIDATE VALUE
    validatePerturbationSize(aValue);

    // SET VALUE (use get to do bounds checking)
    _dx.updElt(aIndex) = aValue;
}
//______________________________________________________________________________
/**
 * Set the derivative perturbation size for all controls.
 */
void StaticOptimizationTarget::
setDX(double aValue)
{
    // VALIDATE VALUE
    validatePerturbationSize(aValue);

    // SET VALUE
    for(int i=0;i<getNumParameters();i++) _dx.updElt(i) = aValue;
}
//______________________________________________________________________________
/**
 * Get the derivative perturbation size.
 */
double StaticOptimizationTarget::
getDX(int aIndex)
{
    return _dx.get(aIndex);
}
//______________________________________________________________________________
/**
 * Get a pointer to the vector of derivative perturbation sizes.
 */
double* StaticOptimizationTarget::
getDXArray()
{
    return &_dx[0];
}

//______________________________________________________________________________
/**
 * Get an optimal force.
 */
void StaticOptimizationTarget::
getActuation(SimTK::State& s, const SimTK::Vector &parameters, SimTK::Vector &forces)
{
    //return(_optimalForce[aIndex]);
    const ForceSet& fs = _model->getForceSet();
    SimTK::Vector tempAccel(getNumConstraints());
    computeAcceleration(s, parameters, tempAccel);
    for(int i=0,j=0;i<fs.getSize();i++) {
        ScalarActuator* act = dynamic_cast<ScalarActuator*>(&fs.get(i));
        if( act )forces(j++) = act->getActuation(s);
    }
}
//==============================================================================
// UTILITY
//==============================================================================
//______________________________________________________________________________
/**
 * Ensure that a derivative perturbation is a valid size
 */
void StaticOptimizationTarget::
validatePerturbationSize(double &aSize)
{
    if(aSize<SMALLDX) {
        log_warn("StaticOptimizationTarget.validatePerturbationSize: dx size "
                 "too small ({}). Resetting dx={}.", aSize, SMALLDX);
        aSize = SMALLDX;
    }
}
//______________________________________________________________________________
/**
 */
void StaticOptimizationTarget::
printPerformance(const SimTK::State& s, double *parameters)
{
    double p;
    setCurrentState( &s );
    objectiveFunc(SimTK::Vector(getNumParameters(),parameters,true),true,p);
    SimTK::Vector constraints(getNumConstraints());
    constraintFunc(SimTK::Vector(getNumParameters(),parameters,true),true,constraints);
    log_cout("time = {} Performance = {} Constraint violation = {}",
            s.getTime(), p, sqrt(~constraints*constraints));
}

//______________________________________________________________________________
/**
 */
void StaticOptimizationTarget::
computeActuatorAreas(const SimTK::State& s )
{
    // COMPUTE ACTUATOR AREAS
    ForceSet& forceSet = _model->updForceSet();
    for(int i=0, j=0;i<forceSet.getSize();i++) {
        ScalarActuator *act = dynamic_cast<ScalarActuator*>(&forceSet.get(i));
        if( act ) {
             act->setActuation(s, 1.0);
             _recipAreaSquared[j] = act->getStress(s);
             _recipAreaSquared[j] *= _recipAreaSquared[j];
             j++;
        }
    }
}

//=============================================================================
// STATIC DERIVATIVES
//=============================================================================
//_____________________________________________________________________________
/**
 * Compute derivatives of a constraint with respect to the
 * controls by central differences.
 *
 * @param dx An array of control perturbation values.
 * @param x Values of the controls at time t.
 * @param ic Index of the constraint.
 * @param dcdx The derivatives of the constraints.
 *
 * @return -1 if an error is encountered, 0 otherwise.
 */
int StaticOptimizationTarget::
CentralDifferencesConstraint(const StaticOptimizationTarget *aTarget,
    double *dx,const Vector &x,Matrix &jacobian)
{
    if(aTarget==NULL) return(-1);

    // INITIALIZE CONTROLS
    int nx = aTarget->getNumParameters(); if(nx<=0) return(-1);
    int nc = aTarget->getNumConstraints(); if(nc<=0) return(-1);
    Vector xp=x;
    Vector cf(nc),cb(nc);

    // INITIALIZE STATUS
    int status = -1;

    // LOOP OVER CONTROLS
    for(int i=0;i<nx;i++) {

        // PERTURB FORWARD
        xp[i] = x[i] + dx[i];
        status = aTarget->constraintFunc(xp,true,cf);
        if(status<0) return(status);

        // PERTURB BACKWARD
        xp[i] = x[i] - dx[i];
        status = aTarget->constraintFunc(xp,true,cb);
        if(status<0) return(status);

        // DERIVATIVES OF CONSTRAINTS
        double rdx = 0.5 / dx[i];
        for(int j=0;j<nc;j++) jacobian(j,i) = rdx*(cf[j]-cb[j]);

        // RESTORE CONTROLS
        xp[i] = x[i];
    }

    return(status);
}
//_____________________________________________________________________________
/**
 * Compute derivatives of performance with respect to the
 * controls by central differences.  Note that the gradient array should
 * be allocated as dpdx[nx].
 *
 * @param dx An array of control perturbation values.
 * @param x Values of the controls at time t.
 * @param dpdx The derivatives of the performance criterion.
 *
 * @return -1 if an error is encountered, 0 otherwise.
 */
int StaticOptimizationTarget::
CentralDifferences(const StaticOptimizationTarget *aTarget,
    double *dx,const Vector &x,Vector &dpdx)
{
    if(aTarget==NULL) return(-1);

    // CONTROLS
    int nx = aTarget->getNumParameters();  if(nx<=0) return(-1);
    Vector xp=x;

    // PERFORMANCE
    double pf,pb;

    // INITIALIZE STATUS
    int status = -1;

    // LOOP OVER CONTROLS
    for(int i=0;i<nx;i++) {

        // PERTURB FORWARD
        xp[i] = x[i] + dx[i];
        status = aTarget->objectiveFunc(xp,true,pf);
        if(status<0) return(status);

        // PERTURB BACKWARD
        xp[i] = x[i] - dx[i];
        status = aTarget->objectiveFunc(xp,true,pb);
        if(status<0) return(status);

        // DERIVATIVES OF PERFORMANCE
        double rdx = 0.5 / dx[i];
        dpdx[i] = rdx*(pf-pb);

        // RESTORE CONTROLS
        xp[i] = x[i];
    }

    return(status);
}

//==============================================================================
// PERFORMANCE AND CONSTRAINTS
//==============================================================================
//------------------------------------------------------------------------------
// PERFORMANCE
//------------------------------------------------------------------------------
//______________________________________________________________________________
/**
 * Compute performance given parameters.
 *
 * @param parameters Vector of optimization parameters.
 * @param performance Value of the performance criterion.
 * @return Status (normal termination = 0, error < 0).
 */
int StaticOptimizationTarget::
objectiveFunc(const Vector &parameters, const bool new_parameters, Real &performance) const
{
    //LARGE_INTEGER start;
    //LARGE_INTEGER stop;
    //LARGE_INTEGER frequency;

    //QueryPerformanceFrequency(&frequency);
    //QueryPerformanceCounter(&start);

    int na = _model->getActuators().getSize();
    double p = 0.0;
    for(int i=0;i<na;i++) {
            p +=  pow(fabs(parameters[i]),_activationExponent);
    }
    performance = p;

    // parameters.dump();
    // std::cout<<p<<std::endl;

    //QueryPerformanceCounter(&stop);
    //double duration = (double)(stop.QuadPart-start.QuadPart)/(double)frequency.QuadPart;
    //std::cout << "objectiveFunc time = " << (duration*1.0e3) << " milliseconds" << std::endl;

    // 0.03 ms

    return(0);
}
//______________________________________________________________________________
/**
 * Compute the gradient of performance given parameters.
 *
 * @param parameters Vector of optimization parameters.
 * @param gradient Derivatives of performance with respect to the parameters.
 * @return Status (normal termination = 0, error < 0).
 */
int StaticOptimizationTarget::
gradientFunc(const Vector &parameters, const bool new_parameters, Vector &gradient) const
{
    //LARGE_INTEGER start;
    //LARGE_INTEGER stop;
    //LARGE_INTEGER frequency;

    //QueryPerformanceFrequency(&frequency);
    //QueryPerformanceCounter(&start);

    int na = _model->getActuators().getSize();
    for(int i=0;i<na;i++) {
        if(parameters[i] < 0) {
            gradient[i] =  -1.0 * _activationExponent * pow(fabs(parameters[i]),_activationExponent-1.0);
        }
        else {
            gradient[i] = _activationExponent * pow(fabs(parameters[i]), _activationExponent - 1.0);
        }
    }

    //QueryPerformanceCounter(&stop);
    //double duration = (double)(stop.QuadPart-start.QuadPart)/(double)frequency.QuadPart;
    //std::cout << "gradientFunc time = " << (duration*1.0e3) << " milliseconds" << std::endl;

    // 0.02 ms

    return(0);
}

//------------------------------------------------------------------------------
// CONSTRAINT
//------------------------------------------------------------------------------
//______________________________________________________________________________
/**
 * Compute acceleration constraints given parameters.
 *
 * @param parameters Vector of optimization parameters.
 * @param constraints Vector of optimization constraints.
 * @return Status (normal termination = 0, error < 0).
 */
int StaticOptimizationTarget::
constraintFunc(const SimTK::Vector &parameters, const bool new_parameters, SimTK::Vector &constraints) const
{
    //LARGE_INTEGER start;
    //LARGE_INTEGER stop;
    //LARGE_INTEGER frequency;

    //QueryPerformanceFrequency(&frequency);
    //QueryPerformanceCounter(&start);

#ifndef USE_LINEAR_CONSTRAINT_MATRIX

    // Evaluate constraint function for all constraints and pick the appropriate component
    computeConstraintVector(parameters,constraints);

#else

    // Use precomputed constraint matrix
    //cout<<"Computing constraints assuming linear dependence..."<<endl;
    constraints = _constraintMatrix * parameters + _constraintVector;

#endif

    //QueryPerformanceCounter(&stop);
    //double duration = (double)(stop.QuadPart-start.QuadPart)/(double)frequency.QuadPart;
    //std::cout << "constraintFunc time = " << (duration*1.0e3) << " milliseconds" << std::endl;

    // 0.11 ms

    return(0);
}

//______________________________________________________________________________
/**
 * Compute all constraints given parameters.
 */
void StaticOptimizationTarget::
computeConstraintVector(SimTK::State& s, const Vector &parameters,Vector &constraints) const
{
    //LARGE_INTEGER start;
    //LARGE_INTEGER stop;
    //LARGE_INTEGER frequency;

    //QueryPerformanceFrequency(&frequency);
    //QueryPerformanceCounter(&start);

    // Compute actual accelerations
    Vector actualAcceleration(getNumConstraints());
    computeAcceleration(s, parameters, actualAcceleration);

    auto coordinates = _model->getCoordinatesInMultibodyTreeOrder();

    // CONSTRAINTS
    for(int i=0; i<getNumConstraints(); i++) {
        const Coordinate& coord = *coordinates[_accelerationIndices[i]];
        int ind = _statesStore->getStateIndex(coord.getSpeedName(), 0);
        if (ind < 0){
            // get the full coordinate speed state variable path name
            string fullname = coord.getStateVariableNames()[1];
            ind = _statesStore->getStateIndex(fullname, 0);
            if (ind < 0){
                string msg = "StaticOptimizationTarget::computeConstraintVector: \n";
                msg+= "target motion for coordinate '";
                msg += coord.getName() + "' not found.";
                throw Exception(msg);
            }
        }
        Function& targetFunc = _statesSplineSet.get(ind);
        std::vector<int> derivComponents(1,0); //take first derivative
        double targetAcceleration = targetFunc.calcDerivative(derivComponents, SimTK::Vector(1, s.getTime()));
        //std::cout << "computeConstraintVector:" << targetAcceleration << " - " <<  actualAcceleration[i] << endl;
        constraints[i] = targetAcceleration - actualAcceleration[i];
    }

    //QueryPerformanceCounter(&stop);
    //double duration = (double)(stop.QuadPart-start.QuadPart)/(double)frequency.QuadPart;
    //std::cout << "computeConstraintVector time = " << (duration*1.0e3) << " milliseconds" << std::endl;

    // 1.5 ms
}
//______________________________________________________________________________
/**
 * Compute the gradient of constraint given parameters.
 *
 * @param parameters Vector of parameters.
 * @param jac Derivative of constraint with respect to the parameters.
 * @return Status (normal termination = 0, error < 0).
 */
int StaticOptimizationTarget::
constraintJacobian(const SimTK::Vector &parameters, const bool new_parameters, SimTK::Matrix &jac) const
{
    //LARGE_INTEGER start;
    //LARGE_INTEGER stop;
    //LARGE_INTEGER frequency;

    //QueryPerformanceFrequency(&frequency);
    //QueryPerformanceCounter(&start);

#ifndef USE_LINEAR_CONSTRAINT_MATRIX

    // Compute gradient 
    StaticOptimizationTarget::CentralDifferencesConstraint(this,&_dx[0],parameters,jac);

#else

    // Use precomputed constraint matrix (works if constraint is linear)
    //cout<<"Computing constraint gradient assuming linear dependence..."<<endl;
    jac = _constraintMatrix;

#endif

    //QueryPerformanceCounter(&stop);
    //double duration = (double)(stop.QuadPart-start.QuadPart)/(double)frequency.QuadPart;
    //std::cout << "constraintJacobian time = " << (duration*1.0e3) << " milliseconds" << std::endl;

    // 0.01 ms

    return 0;
}
//=============================================================================
// ACCELERATION
//=============================================================================
//
void StaticOptimizationTarget::
computeAcceleration(SimTK::State& s, const SimTK::Vector &parameters,SimTK::Vector &rAccel) const
{
    // double time = s.getTime();
    

    const ForceSet& fs = _model->getForceSet();
    for(int i=0,j=0;i<fs.getSize();i++)  {
        ScalarActuator *act = dynamic_cast<ScalarActuator*>(&fs.get(i));
         if( act ) {
             act->setOverrideActuation(s, parameters[j] * _optimalForce[j]);
             j++;
         }
    }

    _model->getMultibodySystem().realize(s,SimTK::Stage::Acceleration);

    SimTK::Vector udot = _model->getMatterSubsystem().getUDot(s);

    for(int i=0; i<_accelerationIndices.getSize(); i++) 
        rAccel[i] = udot[_accelerationIndices[i]];

    //QueryPerformanceCounter(&stop);
    //double duration = (double)(stop.QuadPart-start.QuadPart)/(double)frequency.QuadPart;
    //std::cout << "computeAcceleration time = " << (duration*1.0e3) << " milliseconds" << std::endl;

    // 1.45 ms
}
