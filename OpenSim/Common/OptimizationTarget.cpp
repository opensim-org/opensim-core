// OptimizationTarget.cpp
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

/* Note: This code was originally developed by Realistic Dynamics Inc. 
 * Author: Frank C. Anderson 
 */


//=============================================================================
// INCLUDES
//=============================================================================
#include <stdlib.h>
#include <stdio.h>
#include "OptimizationTarget.h"
#include <iostream>

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
	_dx.get(aIndex) = aValue;
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
	for(int i=0;i<getNumParameters();i++) _dx.get(i) = aValue;
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
		printf("OptimizationTarget.validatePerturbationSize: WARNING- ");
		printf("dx size too small (%le).\n",aSize);
		printf("\tResetting dx=%le.\n",SMALLDX);
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
	std::cout << "performance = " << p << std::endl;
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
 * @return -1 if an error is encountered, 0 otherwize.
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
 * @return -1 if an error is encountered, 0 otherwize.
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
 * @return -1 if an error is encountered, 0 otherwize.
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