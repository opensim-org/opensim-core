#ifndef _rdOptimizationTarget_h_
#define _rdOptimizationTarget_h_
// rdOptimizationTarget.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c) 2005, Stanford University. All rights reserved. 
* Redistribution and use in source and binary forms, with or without 
* modification, are permitted provided that the following conditions
* are met: 
*  - Redistributions of source code must retain the above copyright 
*    notice, this list of conditions and the following disclaimer. 
*  - Redistributions in binary form must reproduce the above copyright 
*    notice, this list of conditions and the following disclaimer in the 
*    documentation and/or other materials provided with the distribution. 
*  - Neither the name of the Stanford University nor the names of its 
*    contributors may be used to endorse or promote products derived 
*    from this software without specific prior written permission. 
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, 
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN 
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
* POSSIBILITY OF SUCH DAMAGE. 
*/

/* Note: This code was originally developed by Realistic Dynamics Inc. 
 * Author: Frank C. Anderson 
 */


//=============================================================================
// INCLUDES
//=============================================================================
#include "rdSQPDLL.h"

//=============================================================================
//=============================================================================
/**
 * This class provides an interface specification for optimizing redundant
 * systems.  If a class represents a redundant system for which one would
 * like to find a set of optimal controls, the class should inherit from
 * this class and implement the virtual functions defined here.
 */
namespace OpenSim { 

class RDSQP_API rdOptimizationTarget  
{
//=============================================================================
// DATA
//=============================================================================
public:
	/** Smallest allowable perturbation size for computing derivatives. */
	static const double SMALLDX;
protected:
	/** Number of performance evaluations. */
	int _nEval;
	/** Number of controls. */
	int _nx;
	/** Number of performance criteria. */
	int _np;
	/** Number of nonlinear inequality constraints. */
	int _nineqn;
	/** Number of inequality constraints. */
	int _nineq;
	/** Number of nonlinear equality constraints. */
	int _neqn;
	/** Number of equality constraints. */
	int _neq;
	/** Perturbation size for computing numerical derivatives. */
	double *_dx;

//=============================================================================
// METHODS
//=============================================================================
public:
	virtual ~rdOptimizationTarget();
	rdOptimizationTarget(int aNX=0);
private:
	void setNull();

	// PERFORMANCE AND CONSTRAINTS
public:
	virtual int
		compute(double *x,double *p,double *c) = 0;
	virtual int
		computeGradients(double *dx,double *x,double *dpdx,double *dcdx) = 0;

	// PERFORMANCE
	virtual int
		computePerformance(double *x,double *p) = 0;
	virtual int
		computePerformanceGradient(double *x,double *dpdx) = 0;

	// CONSTRAINT
	virtual int
		computeConstraint(double *x,int i,double *c) = 0;
	virtual int
		computeConstraintGradient(double *x,int i,double *dcdx) = 0;

	// SET AND GET
	virtual void setNumControls(int aNX);
	int getNumControls();
	void setDX(double aVal);
	void setDX(int aIndex,double aVal);
	double getDX(int aIndex);
	double* getDXArray();
	void setNEvaluations(int aN=0);
	int getNEvaluations();
	int getNumContacts();
	int getNC();
	int getNCInequality();
	int getNCInequalityNonlinear();
	int getNCInequalityLinear();
	int getNCEquality();
	int getNCEqualityNonlinear();
	int getNCEqualityLinear();

	// UTILITY
	bool isControlIndexValid(int aIndex);
	void validatePerturbationSize(double &aSize);
};

}; //namespace

#endif // __rdSQP_h__
