/* -------------------------------------------------------------------------- *
 *                      OpenSim:  OptimizationTarget.cpp                      *
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
#include <stdio.h>
#include "OptimizationTarget.h"

//=============================================================================
// EXPORTED STATIC CONSTANTS
//=============================================================================

using namespace OpenSim;
using SimTK::Vector;
using SimTK::Matrix;

const double OptimizationTarget::SMALLDX = 1.0e-14;

//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Construct an optimization target.
 *
 * @param aNX The number of controls.
 */
OptimizationTarget::
OptimizationTarget(int aNX)
{
    if(aNX>0) setNumParameters(aNX); // OptimizerSystem
}
//==============================================================================
// SET AND GET
//==============================================================================
//------------------------------------------------------------------------------
// CONTROLS
//------------------------------------------------------------------------------
///______________________________________________________________________________
/**
 * Set the number of controls.
 *
 * The number of controls can be set at any time.  However, the perturbation
 * sizes for the controls (i.e., _dx) is destroyed.  Therefore, the
 * perturbation sizes must be reset.
 *
 * @param aNX Number of controls.
 * @see setDX()
 */
void OptimizationTarget::
setNumParameters(const int aNX)
{
    OptimizerSystem::setNumParameters(aNX);
    _dx.setSize(getNumParameters());
}

//------------------------------------------------------------------------------
// DERIVATIVE PERTURBATION SIZES
//------------------------------------------------------------------------------
//______________________________________________________________________________
/**
 * Set the derivative perturbation size.
 */
void OptimizationTarget::
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
void OptimizationTarget::
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
double OptimizationTarget::
getDX(int aIndex)
{
    return _dx.get(aIndex);
}
//______________________________________________________________________________
/**
 * Get a pointer to the vector of derivative perturbation sizes.
 */
double* OptimizationTarget::
getDXArray()
{
    return &_dx[0];
}

//==============================================================================
// UTILITY
//==============================================================================
//______________________________________________________________________________
/**
 * Ensure that a derivative perturbation is a valid size
 */
void OptimizationTarget::
validatePerturbationSize(double &aSize)
{
    if(aSize<SMALLDX) {
        log_warn("OptimizationTarget.validatePerturbationSize: dx size too "
                 "small ({}). Resetting dx={}.",
                aSize, SMALLDX);
        aSize = SMALLDX;
    }
}
//______________________________________________________________________________
/**
 */
void OptimizationTarget::
printPerformance(double *x)
{
    double p;
    objectiveFunc(SimTK::Vector(getNumParameters(),x,true),true,p);
    log_cout("performance = {}", p);
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
int OptimizationTarget::
CentralDifferencesConstraint(const OptimizationTarget *aTarget,
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
int OptimizationTarget::
CentralDifferences(const OptimizationTarget *aTarget,
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

//_____________________________________________________________________________
/**
 * Compute derivatives of performance with respect to the
 * controls by forward differences.  Note that the gradient array should
 * be allocated as dpdx[nx].
 *
 * @param dx An array of control perturbation values.
 * @param x Values of the controls at time t.
 * @param dpdx The derivatives of the performance criterion.
 *
 * @return -1 if an error is encountered, 0 otherwise.
 */
int OptimizationTarget::
ForwardDifferences(const OptimizationTarget *aTarget,
    double *dx,const Vector &x,Vector &dpdx)
{
    if(aTarget==NULL) return(-1);

    // CONTROLS
    int nx = aTarget->getNumParameters();  if(nx<=0) return(-1);
    Vector xp=x;

    // INITIALIZE STATUS
    int status = -1;

    // PERFORMANCE
    double pf,pb;
    
    // current objective function value
    status = aTarget->objectiveFunc(xp,true,pb);
    if(status<0) return(status);

    // LOOP OVER CONTROLS
    for(int i=0;i<nx;i++) {
        // PERTURB FORWARD
        xp[i] = x[i] + dx[i];
        status = aTarget->objectiveFunc(xp,true,pf);
        if(status<0) return(status);

        // DERIVATIVES OF PERFORMANCE
        dpdx[i] = (pf-pb)/dx[i];

        // RESTORE CONTROLS
        xp[i] = x[i];
    }

    return(status);
}
