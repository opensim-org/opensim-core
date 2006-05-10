// rosenTarget.c

//==============================================================================
// INCLUDES
//==============================================================================
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <RD/Tools/rdMath.h>
#include "rosenTarget.h"



//==============================================================================
// CONSTRUCTION
//==============================================================================
//______________________________________________________________________________
/**
 * Destructor.
 */
rosenTarget::~rosenTarget()
{
}
//______________________________________________________________________________
/**
 * Constructor.
 */
rosenTarget::rosenTarget()
{
	// NUMBERS OF CONTROLS AND CONSTRAINTS
	_nx = 2;
	_nc = 2;
	_nceq = 1;
}


//==============================================================================
// PERFORMANCE AND CONSTRAINTS
//==============================================================================
//______________________________________________________________________________
/**
 * Compute performance and the constraints given x.
 */
int rosenTarget::
compute(double *x,double *p,double *c)
{
	// PERFORMANCE
	double a = x[0]*x[0] - x[1];
	double b = 1 - x[0];
	*p = 100.0*a*a + b*b;

	// CONSTRAINTS
	c[0] = x[0]*x[0] - x[0]*x[1] + x[1]*x[1] - 4.0;
	c[1] = 3.0 - x[0] - x[1];

	return(0);
}
//______________________________________________________________________________
/**
 * Compute the gradients of the performance criterion and constraints.
 */
int rosenTarget::
computeGradients(double *dx,double *x,double *dpdx,double *dcdx)
{
	// PERFORMANCE
	dpdx[0] = 400.0*x[0]*(x[0]*x[0]-x[1]) - 2.0*(1.0-x[0]);
	dpdx[1] = -200.0*(x[0]*x[0]-x[1]);

	// CONSTRAINTS
	int I;
	I = rdMtx::ComputeIndex(0,_nx+1,0);
   dcdx[I] = 2.0*x[0] - x[1];

	I = rdMtx::ComputeIndex(0,_nx+1,1);
	dcdx[I] = 2.0*x[1] - x[0];

	I = rdMtx::ComputeIndex(1,_nx+1,0);
	dcdx[I] = -1.0;

	I = rdMtx::ComputeIndex(1,_nx+1,1);
	dcdx[I] = -1.0;

	return(0);
}


//______________________________________________________________________________
/**
 * Compute performance given x.
 */
int rosenTarget::
computePerformance(double *x,double *p)
{
	double a = x[0]*x[0] - x[1];
	double b = 1.0 - x[0];
	*p = 100.0*a*a + b*b;
	printf("rosenTarget.computePerformance: p=%lf\n",*p);
	return(0);
}
//______________________________________________________________________________
/**
 * Compute the gradient of performance given x.
 */
int rosenTarget::
computePerformanceGradient(double *x,double *dpdx)
{
	dpdx[0] = 400.0*x[0]*(x[0]*x[0]-x[1]) - 2.0*(1.0-x[0]);
	dpdx[1] = -200.0*(x[0]*x[0]-x[1]);
	return(0);
}

//______________________________________________________________________________
/**
 * Compute constraint i given x.
 */
int rosenTarget::
computeConstraint(double *x,int i,double *c)
{

	return(0);
}
//______________________________________________________________________________
/**
 * Compute the gradient of constraint i given x.
 */
int rosenTarget::
computeConstraintGradient(double *x,int i,double *dcdx)
{

	return(0);
}

