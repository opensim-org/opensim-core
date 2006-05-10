// DecompTargetNoCompPrescribed.cpp



//==============================================================================
// INCLUDES
//==============================================================================
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <OpenSim/Tools/rdMath.h>
#include <OpenSim/SQP/rdFSQP.h>
#include <OpenSim/Tools/PointConstraint.h>
#include "DecompHardUTWalkNoCompPrescribed.h"
#include "DecompTargetNoCompPrescribed.h"




//==============================================================================
// CONSTRUCTION
//==============================================================================
//______________________________________________________________________________


using namespace OpenSim;
/**
 * Destructor.
 */
DecompTargetNoCompPrescribed::~DecompTargetNoCompPrescribed()
{
}
//______________________________________________________________________________
/**
 * Constructor.
 */
DecompTargetNoCompPrescribed::DecompTargetNoCompPrescribed(int aNX,int aNC,
	DecompHardUTWalkNoCompPrescribed *aAnalysis) :
	rdOptimizationTarget(aNX)
{
	// MODEL
	_analysis = aAnalysis;

	// NUMBER OF CONTROLS
	_nx = aNX;
	if(_nx<=0) _nx=1;

	// NUMBERS OF CONSTRAINTS
	_nineqn = 0;
	_nineq = 0;
	_neqn = 0;
	_neq = aNC;

	// DERIVATIVE PERTURBATION SIZES;
	setDX(1.0e-6);
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
 */
int DecompTargetNoCompPrescribed::
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
int DecompTargetNoCompPrescribed::
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
int DecompTargetNoCompPrescribed::
computePerformance(double *x,double *p)
{
	int status = _analysis->suComputePerformance(x,p);
	return(status);
}
//______________________________________________________________________________
/**
 * Compute the gradient of performance given x.
 */
int DecompTargetNoCompPrescribed::
computePerformanceGradient(double *x,double *dpdx)
{
	int status = _analysis->suComputePerformanceGradient(x,dpdx);
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
int DecompTargetNoCompPrescribed::
computeConstraint(double *x,int ic,double *c)
{
	int status = _analysis->suComputeConstraint(x,ic,c);
	return(status);
}
//______________________________________________________________________________
/**
 * Compute the gradient of constraint i given x.
 */
int DecompTargetNoCompPrescribed::
computeConstraintGradient(double *x,int ic,double *dcdx)
{
	// COMPUTE GRADIENT
	int status = rdFSQP::CentralDifferencesConstraint(this,_dx,x,ic,dcdx);
	return(0);
}

