#ifndef _rdStaticTarget_h_
#define _rdStaticTarget_h_
// rdStaticTarget.h


//==============================================================================
// INCLUDES
//==============================================================================
#include <math.h>
#include <RD/Tools/Math.h>
#include <RD/SQP/rdFSQP.h>
#include <RD/Simulation/Model/Model.h>
#include "rdStaticTargetDLL.h"


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/**
 * An optimization target for finding valid static solutions for human
 * running.
 */
namespace OpenSim { 

class RDSTATICTARGET_API rdStaticTarget
	: public rdOptimizationTarget
{
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//==============================================================================
// DATA
//==============================================================================
private:
	Model *_model;
	double *_dpdx,*_dcdx;
	double *_q,*_dqdt;
	double *_u,*_dudt;
	double *_udot;
	int *_u2jointMap,*_u2axisMap;
	int *_x2uMap,*_c2uMap;
	int _ne;
	int *_be;
	double *_pe,*_fe;

//==============================================================================
// METHODS
//==============================================================================
public:
	//---------------------------------------------------------------------------
	// CONSTRUCTION
	//---------------------------------------------------------------------------
	virtual ~rdStaticTarget();
	rdStaticTarget(int aNX,Model *aModel);

	//---------------------------------------------------------------------------
	// SET AND GET
	//---------------------------------------------------------------------------
	void setQ(double *q);
	void setU(double *u);
	void setUDOT(double *udot);
	void setX2UMap(int *aX2U);
	void setC2UMap(int *aC2U);
	void setFE(int aNE,int *aBE,double *aPE,double *aFE);

	//--------------------------------------------------------------------------
	// REQUIRED OPTIMIZATION TARGET METHODS
	//--------------------------------------------------------------------------
	// PERFORMANCE AND CONSTRAINTS
	int compute(double *x,double *p,double *c);
	int computeGradients(double *dx,double *x,double *dpdx,double *dcdx);
	// PERFORMANCE
	int computePerformance(double *x,double *p);
	int computePerformanceGradient(double *x,double *dpdx);
	// CONSTRAINTS
	int computeConstraint(double *x,int i,double *c);
	int computeConstraintGradient(double *x,int i,double *dcdx);

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
};	// END class rdStaticTarget

}; //namespace
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
