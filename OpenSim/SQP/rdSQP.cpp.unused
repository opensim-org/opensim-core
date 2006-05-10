// rdSQP.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	Copyright 2000 Realistic Dynamics, Inc.
// All rights reserved.
//
// CONFIDENTIAL
//
// The material contain within this file is the property of
// Realistic Dynamics, Inc.  Please do not read, copy, or distribute
// without the expressed writen consent of Realistic Dynamics, Inc.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//=============================================================================
// INCLUDES
//=============================================================================
#include "rdSQPDLL.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <NMBLTK/Tools/rdMath.h>
#include "nrODS.h"
#include "rdSQP.h"
#include "rdOptimizationTarget.h"
#include "paramopt.h"


//=============================================================================
// EXPORTED STATIC CONSTANTS
//=============================================================================


//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
rdSQP::
~rdSQP()
{
	Delete(_xtmp);
	Delete(_dx);
	Delete(_c);
	Delete(_dpdx);
	Delete(_dxda);
	Delete(_dcdx);
	Delete(_cw);
	Delete(_wd);
	Delete(_wi);
}
//_____________________________________________________________________________
/**
 * Delete.
 */
void rdSQP::
Delete(void *aPtr)
{
	if(aPtr!=NULL) { delete []aPtr;  aPtr=NULL; }
}

//_____________________________________________________________________________
/**
 * Construct an rdSQP instance based on an optimization target.
 */
rdSQP::
rdSQP(rdOptimizationTarget *aTarget)
{
	// TARGET 
	_target = aTarget;

	// CONTROLS
	_nx = _target->getNX();
	_xtmp = new double[_nx];
	_dx = new double[_nx];

	// PERFORAMNCE AND CONSTRAINTS
	_nc = _target->getNC();
	_nceq = _target->getNCEquality();
	_c = NULL;
	if(_nc>0) {
		_c = new double[_nc];
	}

	// DERIVATIVES
	_dpdx = new double[_nx];
	_dxda = new double[_nx];
	_dcdx = NULL;
	int size = _nc*(_nx+1);
	if(size>0) _dcdx = new double[size];

	// CONSTRAINT WEIGHTS
	_cw = NULL;
	if(_nc>0) _cw = new double[_nc];

	// WORKSPACE SIZE OF vf02ad
	_nwd = 5*_nx*_nx + 19*_nx + + 14 + 3*_nc + (_nc+1)/2 + _nc + 3*(_nx+1);
	_nwi = _nc + 6*(_nx+1) + 1;
	_wd = new double[_nwd];
	_wi = new int[_nwi];

	// SQP PARAMETERS
	_maxIter = 4*_nx;
	_epsOpt = 1.0e-8;
	_convergence = 0.0;

	// LINESEARCH PARAMETERS
	_epsLine = 1.0e-8;
	_minAlpha = 1.0e-6;
	_maxAlpha = 1.0e+6;
	_fact[0] = 2.0;
	_fact[1] = 0.2;
	_fact[2] = 2.0;
	_mfc = 30;
	_firstAlpha = 0.2;
}


//=============================================================================
// GET AND SET
//=============================================================================
//-----------------------------------------------------------------------------
// MAXIMUM NUMBER OF ITERATIONS
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the maximum number of iterations.
 */
void rdSQP::
setMaxIterations(int aMaxIter)
{
	_maxIter = aMaxIter;
}
//_____________________________________________________________________________
/**
 * Get the maximum number of iterations.
 */
int rdSQP::
getMaxIterations()
{
	return(_maxIter);
}
//-----------------------------------------------------------------------------
// CONVERGENCE CRITERION
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the convergence criterion.
 */
void rdSQP::
setConvergenceCriterion(double aEPS)
{
	_epsOpt = aEPS;
	if(_epsOpt<1.0e-15) {
		_epsOpt = 1.0e-15;
		printf("rdSQP.setConvergenceCriterion: WARNING- given value of the ");
		printf("convergence criterion is too small (%lf).\n",aEPS);
		printf("Using _epsOpt = %lf.\n",_epsOpt);
	}
}
//_____________________________________________________________________________
/**
 * Get the convergence criterion.
 */
double rdSQP::
getConvergenceCriterion()
{
	return(_epsOpt);
}
//-----------------------------------------------------------------------------
// LINE SEARCH:  MAX FUNCTION EVALUATIONS
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the maximum number of function evaluations.
 */
void rdSQP::
setMaxEvaluations(int aMaxEval)
{
	_mfc = aMaxEval;
}
//_____________________________________________________________________________
/**
 * Get the maximum number of function evaluations.
 */
int rdSQP::
getMaxEvaluations()
{
	return(_mfc);
}
//-----------------------------------------------------------------------------
// LINE SEARCH:  CONVERGENCE CRITERION
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the convergence criterion for the line search
 */
void rdSQP::
setLineConvergenceCriterion(double aEPS)
{
	_epsLine = aEPS;
	if(_epsLine<1.0e-15) {
		_epsLine = 1.0e-15;
		printf("rdSQP.setLineConvergenceCriterion: WARNING- given value of the ");
		printf("convergence criterion is too small (%lf).\n",aEPS);
		printf("Using _epsLine = %lf.\n",_epsLine);
	}
}
//_____________________________________________________________________________
/**
 * Get the convergence criterion for the line search.
 */
double rdSQP::
getLineConvergenceCriterion()
{
	return(_epsLine);
}
//-----------------------------------------------------------------------------
// LINE SEARCH:  MINIMUM ALPHA
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the minimum alpha.
 */
void rdSQP::
setMinAlpha(double aMin)
{
	_minAlpha = aMin;
	if(_minAlpha<1.0e-15) {
		_minAlpha = 1.0e-15;
		printf("rdSQP.setMinAlpha: WARNING- given value of alpha ");
		printf("is too small (%lf).\n",aMin);
		printf("Using _minAlpha = %lf.\n",_minAlpha);
	}
}
//_____________________________________________________________________________
/**
 * Get the minimum alpha.
 */
double rdSQP::
getMinAlpha()
{
	return(_minAlpha);
}
//-----------------------------------------------------------------------------
// LINE SEARCH:  MAXIMUM ALPHA
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the maximum alpha.
 */
void rdSQP::
setMaxAlpha(double aMax)
{
	_maxAlpha = aMax;
	if(_maxAlpha>1.0e6) {
		_maxAlpha = 1.0e6;
		printf("rdSQP.setMaxAlpha: WARNING- given value of alpha ");
		printf("is too large (%lf).\n",aMax);
		printf("Using _maxAlpha = %lf.\n",_maxAlpha);
		printf("Try rescaling your problem.\n\n");
	}
}
//_____________________________________________________________________________
/**
 * Get the maximum alpha.
 */
double rdSQP::
getMaxAlpha()
{
	return(_maxAlpha);
}



//=============================================================================
// COMPUTE OPTIMAL CONTROLS
//=============================================================================
//_____________________________________________________________________________
/**
 * Compute a set of optimal controls, given the current state of the
 * optimization target.
 *
 * @param x Values of the controls at time t.
 * @param xopt Optimal values of the controls.
 *
 * @return -1 if an error is encountered, 0 otherwize.
 */
int rdSQP::
computeOptimalControls(const double *xstart,double *x)
{
	printf("\nrdSQP.computeOptimalControls: ...\n");

	// COPY xstart INTO x
	int i;
	for(i=0;i<_nx;i++)  x[i] = xstart[i];

	// DERIVATIVES PERTURBATION SIZE
	for(i=0;i<_nx;i++) _dx[i] = 1.0e-8;

	// INITIALIZATIONS
	_target->setNEvaluations();
	_status = 0;
	_statusOpt = 1;
	_statusLine = 0;

	// ITERATE
	for(int iter=0;iter<_maxIter;iter++) {

		printf("\nrdSQP.computeOptimalControls: ITERATION = %d.\n",iter);

		// COMPUTE PERFORMANCE AND CONSTRAINTS
		printf("\trdSQP.computeOptimalControls: computing performance.\n");
		_status = _target->compute(x,&_p,_c);
		printf("\trdSQP(%d):  p=%lf\n",iter,_p);

		// CHECK STATUS
		if(_status<0) {
			printf("rdSQP.computeOptimalControls: error computing performance");
			printf(" and constraints.\n");
			return(-1);
		}

		// COMPUTE DERIVATIVES
		printf("\trdSQP.computeOptimalControls: computing derivatives.\n");
		_status = _target->computeGradients(_dx,x,_dpdx,_dcdx);

		// CHECK STATUS
		if(_status<0) {
			printf("rdSQP.computeOptimalControls: error computing derivatives");
			printf(" of the performance and constraints.\n");
			return(-1);
		}

		// COMPUTE SEARCH DIRECTION
		printf("\trdSQP.computeOptimalControls: computing search direction.\n");
		computeSearchDirection(&_statusOpt,x,&_p,_dpdx,_c,_dcdx,&_alpha[3],
		 _dxda,_cw);

		// CHECK OPTIMIZATION STATUS
		if(_statusOpt==1) {
			printf("rdSQP.computeOptimalControls: converged on iteration %d.\n",
			 iter);
			printf("Number of evaluations = %d.\n",_target->getNEvaluations());
			return(0);
		} else if(_statusOpt<0) {
			printf("rdSQP.computeOptimalControls: error %d on iteration %d\n",
			 _statusOpt,i);
			return(-1);
		}

		// LINE SEARCH
		printf("\trdSQP.computeOptimalControls: line searching.\n");
		_statusLine = lineSearch(x,_dxda,_cw,_xtmp);

		// CHECK LINE SEARCH STATUS
		if(_statusLine==-3) {
			printf("rdSQP.computeOptimalControls: search direction is poor.\n");
			printf("\tTry improving the accuracy of the derivatives.\n");
			return(-1);
		}

		// SET NEW X
		for(i=0;i<_nx;i++)  x[i] = _xtmp[i];
	}

	return(0);
}


//=============================================================================
// COMPUTE SEARCH DIRECTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Compute a search direction which when followed will result in a reduction
 * of the performance criterion while satisfying the constraints
 *
 * @param dx An array of control perturbation values.
 * @param x Values of the controls at time t.
 * @param dpdx The derivatives of the performance criterion.
 *
 * @return -1 if an error is encountered, 0 otherwize.
 */
void rdSQP::
computeSearchDirection(int *status,double *x,
	double *p,double *dpdx,double *c,double *dcdx,double *alpha,
	double *dxda,double *cw)
{
	int ldcdx = _nx + 1;

/*
	// PRINT CONSTRAINT DERIVATIVES
	int i;
	int size = _nc*(_nx+1);
	printf("dcdx =");
	for(i=0;i<size;i++) {
			printf(" %lf",dcdx[i]);
	}
	printf("\n");
*/

	paramopt_(&_nx,&_nc,&_nceq,x,p,dpdx,c,dcdx,&ldcdx,
	 alpha,&_epsOpt,&_convergence,status,
	 _wd,&_nwd,_wi,&_nwi,dxda,cw);
}


//=============================================================================
// DERIVATIVES
//=============================================================================
//_____________________________________________________________________________
/**
 * Compute derivatives of performance and constraints with respect to the
 * controls by central differences.  Note that the dimension of dcdx is
 * dcdx[nc][nx+1] so that this matrix can be sent directly into paramopt.
 *
 * @param dx An array of control perturbation values.
 * @param x Values of the controls at time t.
 * @param dpdx The derivatives of the performance criterion.
 *
 * @return -1 if an error is encountered, 0 otherwize.
 */
int rdSQP::
CentralDifferences(rdOptimizationTarget *aTarget,
	double *dx,double *x,double *dpdx,double *dcdx)
{
	if(aTarget==NULL) return(-1);

	// GET NUMBERS

	// CONTROLS
	int i;
	int nx = aTarget->getNX();  if(nx<=0) return(-1);
	double *xp = new double[nx];  if(xp==NULL) return(-1);
	for(i=0;i<nx;i++) xp[i]=x[i];

	// PERFORMANCE AND CONSTRAINTS
	double pf,pb;
	double *cf=NULL,*cb=NULL;
	int nc = aTarget->getNC();
	if(nc>0) {
		cf = new double[nc];  if(cf==NULL) return(-1);
		cb = new double[nc];  if(cb==NULL) return(-1);
	}

	// INITIALIZE STATUS
	int status = -1;

	// LOOP OVER CONTROLS
	int j,I;
	double rdx;
	for(i=0;i<nx;i++) {

		rdx = 0.5 / dx[i];

		// PERTURB FORWARD
		xp[i] = x[i] + dx[i];
		status = aTarget->compute(xp,&pf,cf);
		if(status<0) return(status);

		// PERTURB BACKWARD
		xp[i] = x[i] - dx[i];
		status = aTarget->compute(xp,&pb,cb);
		if(status<0) return(status);

		// DERIVATIVES OF PERFORMANCE
		dpdx[i] = rdx*(pf-pb);

		// DERIVATIVES OF CONSTRAINTS
		for(j=0;j<nc;j++) {
			I = rdMtx::ComputeIndex(j,nx+1,i);
			dcdx[I] = rdx*(cf[j]-cb[j]);
		}

		// RESTORE CONTROLS
		xp[i] = x[i];
	}

	// CLEANUP
	if(xp!=NULL) { delete []xp;  xp=NULL; }
	if(cf!=NULL) { delete []cf;  cf=NULL; }
	if(cb!=NULL) { delete []cb;  cb=NULL; }

	return(status);
}


//=============================================================================
// LINE SEARCH
//=============================================================================
//_____________________________________________________________________________
/**
 * Search in direction dx for a minimum.
 */
int rdSQP::
lineSearch(double *x,double *dx,double *cw,double *xnew)
{
	int i,nfc,oflag;
	double f[5];

	// SET FIRST FLAG
	oflag = 0;

	// SET FIRST ALPHA
	_alpha[1] = _firstAlpha;

	// COMPUTE PERFORMANCE AND CONSTRAINTS
	_target->compute(x,&_p,_c);

	// ODS FUNCTION (APPEND CONSTRAINTS)
	f[0] = ods_func(_p,_c,cw,_nc,_nceq,1);

	// SEARCH
	do {

		ods(_nx,x,f,dx,_epsLine,_minAlpha,_maxAlpha,_alpha,xnew,_fact,
		 &oflag,&nfc,_mfc);

		switch(oflag){
		case 0:
			printf("Minimum value = %f\n",f[3]);
			printf("The function is minimized at:\n");
			for (i=0;i<_nx;i++) printf("xnew[%d] = %f\n",i,xnew[i]);
			printf("with alpha = %f\n",_alpha[3]);
			break;

		case -1:
			printf("\nToo many functions calls (%d).",nfc);
			return -1;

		case -2:
			printf("\nalpha too large.");
			return -2;

		case -3:
			printf("\nalpha too small.");
			return -3;

		default:
			_target->compute(xnew,&_p,_c);
			f[oflag-1] = ods_func(_p,_c,cw,_nc,_nceq,1);
			//printf("\t\tf[%d]=%lf\n",oflag-1,f[oflag-1]);
			break;

		} // End of switch -----

	} while(oflag);

	return(0);
}
