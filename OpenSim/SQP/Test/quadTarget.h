#ifndef _quadTarget_h_
#define _quadTarget_h_
// quadTarget.h


//==============================================================================
// INCLUDES
//==============================================================================
#include <RD/SQP/rdSQP.h>


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/**
 * A simple example class for testing class rdSQP and rdOptimizationTarget.
 *
 * This simple class represents a simple n-dimensional quadratic with
 * constraints on the independent variables and nonlinear constraints
 * on functions of the independent variables.
 */
namespace OpenSim { 

class quadTarget : public rdOptimizationTarget
{
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//==============================================================================
// DATA
//==============================================================================
private:
	double *_k;

//==============================================================================
// METHODS
//==============================================================================
public:
	//---------------------------------------------------------------------------
	// CONSTRUCTION
	//---------------------------------------------------------------------------
	virtual ~quadTarget();
	quadTarget(int aNX);

	//---------------------------------------------------------------------------
	// PERFORMANCE AND CONSTRAINTS
	//---------------------------------------------------------------------------
	int compute(double *x,double *p,double *c);
	int computePerformance(double *x,double *p);
	int computeConstraint(double *x,int i,double *c);
	int computeGradients(double *dx,double *x,double *dpdx,double *dcdx);
	int computePerformanceGradient(double *x,double *dpdx);
	int computeConstraintGradient(double *x,int i,double *dcdx);

	//---------------------------------------------------------------------------
	// GET
	//---------------------------------------------------------------------------

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
};	// END class quadTarget

}; //namespace
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
