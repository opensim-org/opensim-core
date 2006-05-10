// rdStaticTarget.cpp



//==============================================================================
// INCLUDES
//==============================================================================
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <RD/Tools/Math.h>
#include <RD/Tools/Mtx.h>
#include <RD/SQP/rdFSQP.h>
#include "rdStaticTarget.h"



//==============================================================================
// CONSTRUCTION
//==============================================================================
//______________________________________________________________________________


using namespace OpenSim;
/**
 * Destructor.
 */
rdStaticTarget::~rdStaticTarget()
{
}
//______________________________________________________________________________
/**
 * Constructor.
 */
rdStaticTarget::rdStaticTarget(int aNX,Model *aModel) :
	rdOptimizationTarget(aNX)
{
	// MODEL
	_model = aModel;
	_q = NULL;
	_u = NULL;
	_udot = NULL;
	_ne = 0;
	_be = NULL;
	_pe = NULL;
	_fe = NULL;

	// NUMBER OF CONTROLS
	_nx = aNX;
	if(_nx<=0) _nx=1;

	// NUMBERS OF CONSTRAINTS
	_nineqn = 0;
	_nineq = 0;
	_neqn = _nx;
	_neq = _nx;

	// DERIVATIVE PERTURBATION SIZES;
	_dx = new double[_nx];
	setDX(1.0e-8);

	// WORK ARRAYS
	_dqdt = new double[_model->getNQ()];
	_dudt = new double[_model->getNU()];
	_dpdx = new double[getNX()];
	_dcdx = new double[getNC()*getNX()];

}


//==============================================================================
// SET AND GET
//==============================================================================
//------------------------------------------------------------------------------
// GENERALIZED COORDINATES
//------------------------------------------------------------------------------
//______________________________________________________________________________
/**
 * Set the generalized coordinates.
 */
void rdStaticTarget::
setQ(double *q)
{
	_q = q;
}
//______________________________________________________________________________
/**
 * Set the generalized speeds.
 */
void rdStaticTarget::
setU(double *u)
{
	_u = u;
}
//______________________________________________________________________________
/**
 * Set the generalized accelerations.
 */
void rdStaticTarget::
setUDOT(double *udot)
{
	_udot = udot;
}
//______________________________________________________________________________
/**
 * Set the external forces.
 */
void rdStaticTarget::
setFE(int aNE,int *aBE,double *aPE,double *aFE)
{
	_ne = aNE;
	_be = aBE;
	_pe = aPE;
	_fe = aFE;
}
//______________________________________________________________________________
/**
 * Set the mapping from constraints to generalized speeds.
 */
void rdStaticTarget::
setC2UMap(int *aC2U)
{
	_c2uMap = aC2U;
}
//______________________________________________________________________________
/**
 * Set the mapping from controls to generalized speeds.
 */
void rdStaticTarget::
setX2UMap(int *aX2U)
{
	_x2uMap = aX2U;
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
int rdStaticTarget::
compute(double *x,double *p,double *c)
{
	return(0);
}
//______________________________________________________________________________
/**
 * Compute the gradients of the performance and the constraints given x.
 * The array dx is an array of perturbation sizes which can be used to
 * compute the gradients numerically.
 *
 * Note- used by paramopt.
 */
int rdStaticTarget::
computeGradients(double *dx,double *x,double *dpdx,double *dcdx)
{
	int status = rdFSQP::CentralDifferences(this,dx,x,dpdx,dcdx);
	return(status);
}

//------------------------------------------------------------------------------
// PERFORMANCE
//------------------------------------------------------------------------------
//______________________________________________________________________________
/**
 * Compute performance given x.
 */
int rdStaticTarget::
computePerformance(double *x,double *p)
{
	int i;
	for(*p=0.0,i=0;i<_nx;i++) *p += 0.1*x[i]*x[i];
	return(0);
}
//______________________________________________________________________________
/**
 * Compute the gradient of performance given x.
 */
int rdStaticTarget::
computePerformanceGradient(double *x,double *dpdx)
{
	int i;
	for(i=0;i<_nx;i++) {
		dpdx[i] = 2.0*0.1*x[i];
	}
	return(0);
}

//------------------------------------------------------------------------------
// CONSTRAINT
//------------------------------------------------------------------------------
//______________________________________________________________________________
/**
 * Compute constraint i given x.
 * Note that the indexing starts at 1;
 */
int rdStaticTarget::
computeConstraint(double *x,int ic,double *c)
{
	int i,I;

	// SET CONFIGURATION
	_model->setConfiguration(_q,_u);

	// APPLY EXTERNAL FORCES
	for(i=0;i<_ne;i++) {
		I = Mtx::ComputeIndex(i,3,0);
		_model->applyForce(_be[i],&_pe[I],&_fe[I]);
	}

	// APPLY GENERALIZED FORCES
	for(i=0;i<_nx;i++) {
		_model->applyGeneralizedForce(_x2uMap[i],x[i]);
	}

	// COMPUTE DERIVATIVES
	_model->computeAccelerations(_dqdt,_dudt);

	// COMPUTE CONSTRAINT
	*c = _udot[ic] - _dudt[_c2uMap[ic]];

	return(0);
}
//______________________________________________________________________________
/**
 * Compute the gradient of constraint i given x.
 */
int rdStaticTarget::
computeConstraintGradient(double *x,int ic,double *dcdx)
{
	// COMPUTE GRADIENT
	int status = rdFSQP::CentralDifferencesConstraint(this,_dx,x,ic,dcdx);

	return(0);
}

