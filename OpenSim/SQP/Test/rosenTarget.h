#ifndef _rosenTarget_h_
#define _rosenTarget_h_
// rosenTarget.h


//==============================================================================
// INCLUDES
//==============================================================================
#include <RD/SQP/rdSQP.h>


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/**
 * This simple class represents the Rosenbroch Banana function which is
 * typically very difficult to optimize.
 */
namespace OpenSim { 

class rosenTarget : public rdOptimizationTarget
{
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//==============================================================================
// DATA
//==============================================================================
private:

//==============================================================================
// METHODS
//==============================================================================
public:
	//---------------------------------------------------------------------------
	// CONSTRUCTION
	//---------------------------------------------------------------------------
	virtual ~rosenTarget();
	rosenTarget();

	//---------------------------------------------------------------------------
	// PERFORMANCE AND CONSTRAINTS
	//---------------------------------------------------------------------------
	int compute(double *x,double *p,double *c);
	int computeGradients(double *dx,double *x,double *dpdx,double *dcdx);
	int computePerformance(double *x,double *p);
	int computePerformanceGradient(double *x,double *dpdx);
	int computeConstraint(double *x,int i,double *c);
	int computeConstraintGradient(double *x,int i,double *dcdx);

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
};	// END class rosenTarget

}; //namespace
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
