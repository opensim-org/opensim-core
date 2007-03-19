#ifndef _rdSQP_h_
#define _rdSQP_h_
// rdSQP.cpp
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
#include <OpenSim/Common/Mtx.h>
#include "osimSQPDLL.h"
#include "rdOptimizationTarget.h"


//=============================================================================
//=============================================================================
/**
 * This class provides methods for finding the optimal controls of a redundant
 * system by applying sequential quadratic programming techniques.
 */
namespace OpenSim { 

class OSIMSQP_API rdSQP
{
//=============================================================================
// DATA
//=============================================================================
private:
	// OPTIMIZATION TARGET
	rdOptimizationTarget *_target;

	// CONTROLS
	int _nx;
	double *_xtmp,*_dx;

	// PERFORMANCE AND CONSTRAINTS
	int _status;
	int _nc,_nceq;
	double _p,*_c,*_cw;

	// DERIVATIVES
	double *_dpdx,*_dcdx,*_dxda;

	// WORKSPACE
	int _nwd,_nwi;
	double *_wd;
	int *_wi;

	// SQP PARAMETERS
	int _statusOpt;
	int _maxIter;
	double _epsOpt,_convergence;

	// LINE SEARCH PARAMETERS
	int _statusLine;
	int _mfc;
	double _epsLine;
	double _fact[4];
	double _minAlpha,_maxAlpha;
	double _firstAlpha;
	double _alpha[4];

//=============================================================================
// METHODS
//=============================================================================
public:
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
	virtual ~rdSQP();
	static void Delete(void *aPtr);
	rdSQP(rdOptimizationTarget *aTarget);

	//--------------------------------------------------------------------------
	// SET AND GET
	//--------------------------------------------------------------------------
	void setMaxIterations(int aMaxIter);
	int getMaxIterations();
	void setConvergenceCriterion(double aMaxIter);
	double getConvergenceCriterion();
	void setMaxEvaluations(int aMaxEval);
	int getMaxEvaluations();
	void setLineConvergenceCriterion(double aMaxIter);
	double getLineConvergenceCriterion();
	void setMinAlpha(double aMin);
	double getMinAlpha();
	void setMaxAlpha(double aMax);
	double getMaxAlpha();


	//--------------------------------------------------------------------------
	// OPTIMAL CONTROLS
	//--------------------------------------------------------------------------
	int computeOptimalControls(const double *xstart,double *x);

	//--------------------------------------------------------------------------
	// DERIVATIVES
	//--------------------------------------------------------------------------
	static int
	CentralDifferences(rdOptimizationTarget *aTarget,
	 double *dx,double *x,double *dpdx,double *dcdx);

	//--------------------------------------------------------------------------
	// SEARCH DIRECTION
	//--------------------------------------------------------------------------
	void
	computeSearchDirection(int *status,double *x,
	 double *p,double *dpdx,double *c,double *dcdx,double *alpha,
	 double *dxda,double *mu);

	//--------------------------------------------------------------------------
	// LINE SEARCH
	//--------------------------------------------------------------------------
	int lineSearch(double *x,double *dx,double *mu,double *xnew);

//=============================================================================
};	// END class rdSQP

}; //namespace
//=============================================================================
//=============================================================================

#endif // __rdSQP_h__
