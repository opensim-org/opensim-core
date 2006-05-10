// quadTarget.c



//==============================================================================
// INCLUDES
//==============================================================================
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "quadTarget.h"



//==============================================================================
// CONSTRUCTION
//==============================================================================
//______________________________________________________________________________
/**
 * Destructor.
 */
quadTarget::~quadTarget()
{
	if(_k!=NULL) { delete []_k;  _k=NULL; }
}
//______________________________________________________________________________
/**
 * Constructor.
 */
quadTarget::quadTarget(int aNX)
{
	// NUMBER OF CONTROLS
	_nx = aNX;
	if(_nx<=0) _nx=1;

	// NUMBERS OF CONSTRAINTS
	_nineqn = 0;
	_nineq = _nx;
	_neqn = 0;
	_neq = 1;

	// FUNCTION COEFFICENTS;
	_k = new double[_nx];
	for(int i=0;i<_nx;i++) {
		_k[i] = drand48() + 1.0;
	}
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
int quadTarget::
compute(double *x,double *p,double *c)
{
	// INCREMENT THE NUMBER OF EVALUATIONS
	_nEval++;

	// PERFORMANCE
	int i;
	for(*p=0.0,i=0;i<_nx;i++) {
		*p += _k[i]*x[i]*x[i];
	}

	// EQUALITY CONSTRAINTS
	for(c[0]=0.0,i=0;i<_nx;i++) c[0] += x[i];
	c[0] -= 2.1*_nx;

	// INEQUALITY CONSTRAINTS
	for(i=0;i<_nx;i++) {
		c[getNCEquality()] = x[i] - 1.0;
	}

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
int quadTarget::
computeGradients(double *dx,double *x,double *dpdx,double *dcdx)
{
	int status = rdSQP::CentralDifferences(this,dx,x,dpdx,dcdx);
	return(status);
}

//------------------------------------------------------------------------------
// PERFORMANCE
//------------------------------------------------------------------------------
//______________________________________________________________________________
/**
 * Compute performance given x.
 */
int quadTarget::
computePerformance(double *x,double *p)
{
	int i;
	for(*p=0.0,i=0;i<_nx;i++) {
		*p += _k[i]*x[i]*x[i];
	}
	return(0);
}
//______________________________________________________________________________
/**
 * Compute the gradient of performance given x.
 */
int quadTarget::
computePerformanceGradient(double *x,double *dpdx)
{
	int i;
	for(i=0;i<_nx;i++) {
		dpdx[i] = 2.0*_k[i]*x[i];
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
int quadTarget::
computeConstraint(double *x,int ic,double *c)
{
	int i;
	double value;

	// INEQUALITY CONSTRAINTS
	if(ic<=getNCInequality()) {
		int ix = ic-1;
		value = 1.0 - x[ix];

	// EQUALITY CONSTRAINT
	} else if(ic==getNCInequality()+1) {
		for(value=0.0,i=0;i<_nx;i++) value += x[i];
		value -= 2.1*_nx;
	}

	// ASSIGN VALUE
	*c = value;

	return(0);
}
//______________________________________________________________________________
/**
 * Compute the gradient of constraint i given x.
 */
int quadTarget::
computeConstraintGradient(double *x,int ic,double *dcdx)
{
	int i;

	// INEQUALITY CONSTRAINTS
	if(ic<=getNCInequality()) {
		int ico = ic-1;
		for(i=0;i<getNX();i++) {
			if(i==ico) {
				dcdx[i] = -1.0;
			} else {
				dcdx[i] = 0.0;
			}
		}

	// EQUALITY CONSTRAINT
	} else if(ic==getNCInequality()+1) {
		for(i=0;i<getNX();i++) {
			dcdx[i] = 1.0;
		}
	}

	return(0);
}

