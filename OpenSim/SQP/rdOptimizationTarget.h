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
#include "osimSQPDLL.h"
#include <simmath/Optimizer.h>

//=============================================================================
//=============================================================================
/**
 * This class provides an interface specification for optimizing redundant
 * systems.  If a class represents a redundant system for which one would
 * like to find a set of optimal controls, the class should inherit from
 * this class and implement the virtual functions defined here.
 */
namespace OpenSim { 

class OSIMSQP_API rdOptimizationTarget : public SimTK::OptimizerSystem
{
//=============================================================================
// DATA
//=============================================================================
public:
	/** Smallest allowable perturbation size for computing derivatives. */
	static const double SMALLDX;
protected:
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

private:
	// For caching
	mutable SimTK::Vector _cachedConstraintJacobianParameters;
	mutable SimTK::Matrix _cachedConstraintJacobian;
	mutable SimTK::Vector _cachedConstraintParameters;
	mutable SimTK::Vector _cachedConstraint;

//=============================================================================
// METHODS
//=============================================================================
public:
	virtual ~rdOptimizationTarget();
	rdOptimizationTarget(int aNX=0);
private:
	void setNull();

public:
	// SET AND GET
	virtual void setNumControls(int aNX);
	int getNumControls() const;
	void setDX(double aVal);
	void setDX(int aIndex,double aVal);
	double getDX(int aIndex);
	double* getDXArray();
	int getNumContacts() { return 1; } // used to return number of performance criteria, but for now we'll assume 1
	int getNC() const;
	int getNCInequality() const;
	int getNCInequalityNonlinear() const;
	int getNCInequalityLinear() const;
	int getNCEquality() const;
	int getNCEqualityNonlinear() const;
	int getNCEqualityLinear() const;

	// UTILITY
	bool isControlIndexValid(int aIndex);
	void validatePerturbationSize(double &aSize);

	virtual bool prepareToOptimize(double *x) { return false; }
	virtual void printPerformance(double *x);

	// Caching support for FSQP (since it likes to query constraints one at a time)
	void clearCache();
	int computeConstraint(const SimTK::Vector &x, const bool new_coefficients, double &c, int ic) const;
	int computeConstraintGradient(const SimTK::Vector &x, const bool new_coefficients, SimTK::Vector &dcdx, int ic) const;
};

}; //namespace

#endif // _rdOptimizationTarget_h_
