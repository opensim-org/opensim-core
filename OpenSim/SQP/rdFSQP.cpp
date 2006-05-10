// rdFSQP.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Author:  Frank C. Anderson
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//=============================================================================
// INCLUDES
//=============================================================================
#include <iostream>
#include <string>
#include <CFSQP/cfsqpusr.h>
#include <OpenSim/Tools/rdMath.h>
#include <OpenSim/Tools/Mtx.h>
#include "rdSQPDLL.h"
#include "rdFSQP.h"
#include "rdOptimizationTarget.h"



using namespace OpenSim;
using namespace std;

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
rdFSQP::
~rdFSQP()
{
	if(_bl!=NULL) { delete []_bl;  _bl=NULL; }
	if(_bu!=NULL) { delete []_bu;  _bu=NULL; }
	if(_x!=NULL) { delete []_x;  _x=NULL; }
	if(_mesh!=NULL) { delete []_mesh;  _mesh=NULL; }
	if(_p!=NULL) { delete []_p;  _p=NULL; }
	if(_c!=NULL) { delete []_c;  _c=NULL; }
	if(_lambda!=NULL) { delete []_lambda;  _lambda=NULL; }
}

//_____________________________________________________________________________
/**
 * Construct an rdFSQP instance based on an optimization target.
 */
rdFSQP::
rdFSQP(rdOptimizationTarget *aTarget)
{
	setNull();

	// SQP STUFF- NOT CURRENTLY SUPPORTED
	int i;
	_ncsrl = 0;
	_ncsrn = 0;
	_nfsr = 0;
	int lenmesh = _nfsr+_ncsrn+_ncsrl;  if(lenmesh<1) lenmesh=1;
	_mesh = new int[lenmesh];
	for(i=0;i<lenmesh;i++)  _mesh[i] = 0;

	// TARGET 
	setTarget(aTarget);

	// NEW MAX ITERATIONS
	_maxIter = 4 * _target->getNX();

}


//=============================================================================
// CONSTRUCTION
//=============================================================================
//-----------------------------------------------------------------------------
// OPTIMIZATION TARGET
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set NULL values for the member variables.
 */
void rdFSQP::
setNull()
{
	_target = NULL;
	_mode = 100;
	_printLevel = 1;
	_maxIter = 100;
	_inform = 0;
	_infinity = 1.0e10;
	_eps = 1.0e-8;
	_epseqn = 1.0e-8;
	_udelta = 0.0;
	_ncsrl = 0;
	_ncsrn = 0;
	_nfsr = 0;
	_mesh = NULL;
	_bl = NULL;
	_bu = NULL;
	_x = NULL;
	_p = NULL;
	_c = NULL;
	_lambda = NULL;
}

//=============================================================================
// GET AND SET
//=============================================================================
//-----------------------------------------------------------------------------
// OPTIMIZATION TARGET
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the optimization target.
 *
 * It is permissible to change the target at any time.  Calling this
 * method, however, will destroy any information about the lower and upper
 * bounds for the controls.  The upper and lower bounds therefore need to
 * be set again.
 *
 * @param aTarget New optimization target.
 * @return Previous optimiztion target.
 */
rdOptimizationTarget* rdFSQP::
setTarget(rdOptimizationTarget *aTarget)
{
	rdOptimizationTarget *prev = _target;
	_target = aTarget;

	// CONTROLS
	int nx = _target->getNX();
	if(_x!=NULL) delete[] _x;
	_x = new double[nx];

	// PERFORMANCE
	int i;
	int lenp = _target->getNP() - _nfsr;
	for(i=0;i<_nfsr;i++)  lenp += _mesh[i];
	if(lenp<1) lenp = 1;
	if(_p!=NULL) delete[] _p;
	_p = new double[lenp];

	// CONSTRAINTS
	int lenc = _target->getNC() - _ncsrl - _ncsrn;
	for(i=0;i<_ncsrn;i++)  lenc += _mesh[i+_nfsr];
	for(i=0;i<_ncsrl;i++)  lenc += _mesh[i+_nfsr+_ncsrn];
	if(lenc<1) lenc = 1;
	if(_c!=NULL) delete[] _c;
	_c = new double[lenc];

	// LAGRANGE MULTIPLIERS
	if(_lambda!=NULL) delete[] _lambda;
	_lambda = new double[nx+lenp+lenc];

	// SET DEFAULT BOUNDS
	if(_bl!=NULL) delete[] _bl;
	if(_bu!=NULL) delete[] _bu;
	_bl = new double[nx];
	_bu = new double[nx];
	for(i=0;i<nx;i++) {
		_bl[i] = -_infinity;
		_bu[i] =  _infinity;
	}

	return(prev);
}
//_____________________________________________________________________________
/**
 * Get the current optimization target.
 *
 * @return Optimization target
 */
rdOptimizationTarget* rdFSQP::
getTarget()
{
	return(_target);
}


//-----------------------------------------------------------------------------
// MODE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the mode of fsqp.
 */
void rdFSQP::
setMode(int aMode)
{
	_mode = aMode;
}
//_____________________________________________________________________________
/**
 * Get the mode of fsqp.
 */
int rdFSQP::
getMode()
{
	return(_mode);
}

//-----------------------------------------------------------------------------
// PRINT LEVEL
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the print level.
 */
void rdFSQP::
setPrintLevel(int aLevel)
{
	_printLevel = aLevel;
	if(_printLevel<0) _printLevel=0;
}
//_____________________________________________________________________________
/**
 * Get the print level.
 */
int rdFSQP::
getPrintLevel()
{
	return(_printLevel);
}

//-----------------------------------------------------------------------------
// MAXIMUM NUMBER OF ITERATIONS
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the maximum number of iterations.
 */
void rdFSQP::
setMaxIterations(int aMaxIter)
{
	_maxIter = aMaxIter;
}
//_____________________________________________________________________________
/**
 * Get the maximum number of iterations.
 */
int rdFSQP::
getMaxIterations()
{
	return(_maxIter);
}

//-----------------------------------------------------------------------------
// INFINITY
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the value considered to be infinity.
 */
void rdFSQP::
setInfinity(double aInfinity)
{
	_infinity = aInfinity;
}
//_____________________________________________________________________________
/**
 * Set the value considered to be infinity.
 */
double rdFSQP::
getInfinity()
{
	return(_infinity);
}

//-----------------------------------------------------------------------------
// CONVERGENCE CRITERION
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the convergence criterion.
 */
void rdFSQP::
setConvergenceCriterion(double aEPS)
{
	_eps = aEPS;
	if(_eps<1.0e-15) {
		_eps = 1.0e-15;
		printf("rdFSQP.setConvergenceCriterion: WARNING- given value of the ");
		printf("convergence criterion is too small (%lf).\n",aEPS);
		printf("Using _eps = %lf.\n",_eps);
	}
}
//_____________________________________________________________________________
/**
 * Get the convergence criterion.
 */
double rdFSQP::
getConvergenceCriterion()
{
	return(_eps);
}

//-----------------------------------------------------------------------------
// NONLINEAR EQUALITY CONSTRAINT TOLERANCE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the tolerance allowed in meeting nonlinear equality constraints.
 */
void rdFSQP::
setNonlinearEqualityConstraintTolerance(double aEPSEQN)
{
	_epseqn = aEPSEQN;
	if(_epseqn<1.0e-15) {
		_epseqn = 1.0e-15;
		printf("rdFSQP.setNonlinearEqualityConstraintTolerance: WARNING- ");
		printf(" the given value (%lf) is too small.\n",aEPSEQN);
		printf("Using _epseqn = %lf.\n",_epseqn);
	}
}
//_____________________________________________________________________________
/**
 * Get the tolerance allowed in meeting nonlinear equality constraints.
 */
double rdFSQP::
getNonlinearEqualityConstraintTolerance()
{
	return(_epseqn);
}

//-----------------------------------------------------------------------------
// LOWER BOUND
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set a lower bound on the controls.
 *
 * @param aLower Lower bound for all controls.
 */
void rdFSQP::
setLowerBound(double aLower)
{
	for(int i=0;i<_target->getNX();i++)  _bl[i] = aLower;
}
//_____________________________________________________________________________
/**
 * Set the lower bound for a specified control.
 *
 * @param aIndex Index of the control.
 * @param aLower Lower bound.
 */
void rdFSQP::
setLowerBound(int aIndex,double aLower)
{
	if(aIndex<0) return;
	if(aIndex>=_target->getNX()) return;
	_bl[aIndex] = aLower;
}
//_____________________________________________________________________________
/**
 * Set lower bounds on the controls.
 *
 * @param aLower Array of lower bounds.  The size of aLower should be at
 * least the number of controls.
 */
void rdFSQP::
setLowerBound(double aLower[])
{
	for(int i=0;i<_target->getNX();i++)  _bl[i] = aLower[i];
}
//_____________________________________________________________________________
/**
 * Get lower bounds on the controls.
 * Note that the address is returned and not a copy;
 *
 * @return Pointer the the array of lower bounds.
 */
double* rdFSQP::
getLowerBound()
{
	return(_bl);
}

//-----------------------------------------------------------------------------
// UPPER BOUND
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set a upper bound on the controls.
 *
 * @param aUpper Upper bound for all controls.
 */
void rdFSQP::
setUpperBound(double aUpper)
{
	for(int i=0;i<_target->getNX();i++)  _bu[i] = aUpper;
}
//_____________________________________________________________________________
/**
 * Set the upper bound for a specified control.
 *
 * @param aIndex Index of the control.
 * @param aLower Upper bound.
 */
void rdFSQP::
setUpperBound(int aIndex,double aUpper)
{
	if(aIndex<0) return;
	if(aIndex>=_target->getNX()) return;
	_bu[aIndex] = aUpper;
}
//_____________________________________________________________________________
/**
 * Set upper bounds on the controls.
 *
 * @param aUpper Array of upper bounds.  The size of aUpper should be at
 * least the the number of controls
 */
void rdFSQP::
setUpperBound(double aUpper[])
{
	for(int i=0;i<_target->getNX();i++)  _bu[i] = aUpper[i];
}
//_____________________________________________________________________________
/**
 * Get upper bounds on the controls.
 * Note that the address is returned and not a copy;
 *
 * @return Pointer to the array of upper bounds.
 */
double* rdFSQP::
getUpperBound()
{
	return(_bu);
}


//=============================================================================
// COMPUTE OPTIMAL CONTROLS
//=============================================================================
//_____________________________________________________________________________
/**
 * Compute a set of optimal controls, given the input controls (xin) and
 * the current state of the optimization target.
 *
 * Whether or not the optimization terminates normally, the latest value of
 * the controls are copied to xout.  It is safe to use the same pointer
 * for xin and xout.  However, in all cases the calling routine must
 * allocate enough space for xin and xout.
 *
 * @param xin Values of the controls.
 * @param xout Optimal values of the controls.
 *
 * @return Parameter inform of cfsqp. -1 means a fatal error.
 */
int rdFSQP::
computeOptimalControls(const double *xin,double *xout)
{
	//printf("\nrdFSQP.computeOptimalControls: ...\n");

	// CHECK CONTROL POINTERS
	if(xin==NULL) return(-1);
	if(xout==NULL) return(-1);

	// SET INITIAL X
	int i;
	for(i=0;i<_target->getNX();i++)  _x[i] = xin[i];

	// SET THE CLIENT DATA TO THIS TARGET
	void *cd = _target;

	// OPTIMIZE
	cfsqp(_target->getNX(),_target->getNP(),_nfsr,
		_target->getNCInequalityNonlinear(),_target->getNCInequality(),
		_target->getNCEqualityNonlinear(),_target->getNCEquality(),
		_ncsrl,_ncsrn,_mesh,
		_mode,_printLevel,_maxIter,&_inform,_infinity,_eps,_epseqn,_udelta,
		_bl,_bu,_x,_p,_c,_lambda,pFunc,cFunc,dpdxFunc,dcdxFunc,cd);

	// SET OUTPUT X
	for(i=0;i<_target->getNX();i++)  xout[i] = _x[i];

	return(_inform);
}


//=============================================================================
// STATIC DERIVATIVES
//=============================================================================
//_____________________________________________________________________________
/**
 * Compute derivatives of performance and constraints with respect to the
 * controls by central differences.  Note that the gradient arrays should
 * be allocated as dpdx[nx] and dcdx[nc][nx].
 *
 * @param dx An array of control perturbation values.
 * @param x Values of the controls at time t.
 * @param dpdx The derivatives of the performance criterion.
 * @param dcdx The derivatives of the constraints.
 *
 * @return -1 if an error is encountered, 0 otherwize.
 */
int rdFSQP::
CentralDifferences(rdOptimizationTarget *aTarget,
	double *dx,double *x,double *dpdx,double *dcdx)
{
	if(aTarget==NULL) return(-1);

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
			I = Mtx::ComputeIndex(j,nx,i);
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
//_____________________________________________________________________________
/**
 * Compute derivatives of a constraint with respect to the
 * controls by central differences.
 *
 * @param dx An array of control perturbation values.
 * @param x Values of the controls at time t.
 * @param ic Index of the constraint.
 * @param dcdx The derivatives of the constraints.
 *
 * @return -1 if an error is encountered, 0 otherwize.
 */
int rdFSQP::
CentralDifferencesConstraint(rdOptimizationTarget *aTarget,
	double *dx,double *x,int ic,double *dcdx)
{
	if(aTarget==NULL) return(-1);
	if(x==NULL) return(-1);
	if(dcdx==NULL) return(-1);

	// INITIALIZE CONTROLS
	int nx = aTarget->getNX();  if(nx<=0) return(-1);

	// INITIALIZE STATUS
	int status = -1;

	// LOOP OVER CONTROLS
	int i;
	double rdx;
	double xSave,cb,cf;
	for(i=0;i<nx;i++) {

		rdx = 0.5 / dx[i];

		// SAVE ORIGINAL CONTROL VALUE
		xSave = x[i];

		// PERTURB BACKWARD
		x[i] -= dx[i];
		status = aTarget->computeConstraint(x,ic,&cb);
		if(status<0) return(status);
		x[i] = xSave;

		// PERTURB FORWARD
		x[i] += dx[i];
		status = aTarget->computeConstraint(x,ic,&cf);
		if(status<0) return(status);
		x[i] = xSave;

		// DERIVATIVES OF CONSTRAINTS
		dcdx[i] = rdx*(cf-cb);
	}

	return(status);
}
//_____________________________________________________________________________
/**
 * Compute derivatives of performance with respect to the
 * controls by central differences.  Note that the gradient array should
 * be allocated as dpdx[nx].
 *
 * @param dx An array of control perturbation values.
 * @param x Values of the controls at time t.
 * @param dpdx The derivatives of the performance criterion.
 *
 * @return -1 if an error is encountered, 0 otherwize.
 */
int rdFSQP::
CentralDifferences(rdOptimizationTarget *aTarget,
	double *dx,double *x,double *dpdx)
{
	if(aTarget==NULL) return(-1);

	// CONTROLS
	int i;
	int nx = aTarget->getNX();  if(nx<=0) return(-1);
	double *xp = new double[nx];  if(xp==NULL) return(-1);
	for(i=0;i<nx;i++) xp[i]=x[i];

	// PERFORMANCE
	double pf,pb;

	// INITIALIZE STATUS
	int status = -1;

	// LOOP OVER CONTROLS
	double rdx;
	for(i=0;i<nx;i++) {

		rdx = 0.5 / dx[i];

		// PERTURB FORWARD
		xp[i] = x[i] + dx[i];
		status = aTarget->computePerformance(xp,&pf);
		if(status<0) return(status);

		// PERTURB BACKWARD
		xp[i] = x[i] - dx[i];
		status = aTarget->computePerformance(xp,&pb);
		if(status<0) return(status);

		// DERIVATIVES OF PERFORMANCE
		dpdx[i] = rdx*(pf-pb);

		// RESTORE CONTROLS
		xp[i] = x[i];
	}

	// CLEANUP
	if(xp!=NULL) { delete []xp;  xp=NULL; }

	return(status);
}


//=============================================================================
// STATIC PERFORMANCE AND CONSTRAINT EVALUATIONS
//=============================================================================
//______________________________________________________________________________
/**
 * Compute the performance criterion.
 */
void rdFSQP::
pFunc(int nparam,int j,double *x,double *p,void *cd)
{
	// CAST CLIENT DATA
	rdOptimizationTarget *target = (rdOptimizationTarget *)cd;
	if(target==NULL) {
		printf("rdFSQP::pFunc: ERROR- null target pointer.\n");
		return;
	}

	// COMPUTE
	target->computePerformance(x,p);
}
//______________________________________________________________________________
/**
 * Compute the derivatives of the performance criterion.
 */
void rdFSQP::
dpdxFunc(int nparam,int j,double *x,double *dpdx,
	void (*dummy)(int,int,double *,double *,void *),void *cd)
{
	// CAST CLIENT DATA
	rdOptimizationTarget *target = (rdOptimizationTarget *)cd;
	if(target==NULL) {
		printf("rdFSQP::dpdxFunc: ERROR- null target pointer.\n");
		return;
	}

	// COMPUTE
	target->computePerformanceGradient(x,dpdx);
}

//______________________________________________________________________________
/**
 * Compute the constraints.
 */
void rdFSQP::
cFunc(int nparam,int j,double *x,double *c,void *cd)
{
	// CAST CLIENT DATA
	rdOptimizationTarget *target = (rdOptimizationTarget *)cd;
	if(target==NULL) {
		printf("rdFSQP::cFunc: ERROR- null target pointer.\n");
		return;
	}

	// COMPUTE
	target->computeConstraint(x,j,c);
}
//______________________________________________________________________________
/**
 * Compute the gradient of the constraints.
 */
void rdFSQP::
dcdxFunc(int nparam,int j,double *x,double *dcdx,
	void (*dummy)(int,int,double *,double *,void *),
	void *cd)
{
	// CAST CLIENT DATA
	rdOptimizationTarget *target = (rdOptimizationTarget *)cd;
	if(target==NULL) {
		printf("rdFSQP::dcdxFunc: ERROR- null target pointer.\n");
		return;
	}

	// COMPUTE
	target->computeConstraintGradient(x,j,dcdx);
}

//=============================================================================
// PRINT
//=============================================================================
//______________________________________________________________________________
/**
 * Print the meaning of the value returned by computeOptimalControls().
 */
void rdFSQP::
PrintInform(int aInform,ostream &aOStream)
{
	switch(aInform) {
		case(0):
			aOStream<<"rdFSQP(0): Normal termination.";
			break;
		case(1):
			aOStream<<"rdFSQP(1): User-provided initial guess is infeasible ";
			aOStream<<"for linear constraints\n";
			aOStream<<"and CFSQP is unable to geerate a point satisfying these ";
			aOStream<<"conditions.\n";
			break;
		case(2):
			aOStream<<"rdFSQP(2): The user-provided initial guess is infeasible ";
			aOStream<<"for non-linear inequality constraints\n";
			aOStream<<"and linear constraints, and CFSQP is unable to generate ";
			aOStream<<"a point satisfying these constraints.\n";
			aOStream<<"This may be due to insucient accuracy of the QP solver.\n";
			break;
		case(3):
			aOStream<<"rdFSQP(3): The maximum number of iterations has been ";
			aOStream<<"reached before a solution was obtained.\n";
			break;
		case(4):
			aOStream<<"rdFSQP(4): The line search failed to find a new iterate.";
			aOStream<<" The step size was smaller than\n";
			aOStream<<"the machine precision.\n";
			break;
		case(5):
			aOStream<<"rdFSQP(5): Failure of the QP solver in attempting to construct d0.\n";
			break;
		case(6):
			aOStream<<"rdSQP(6): Failure of the QP solver in attempting to construct d1.\n";
			break;
		case(7):
			aOStream<<"rdSQP(7): Input data are not consistent.  Set the print level";
			aOStream<<" greater than 0 for more information.\n";
			break;
		case(8):
			aOStream<<"rdSQP(8): The new iterate is numerically equivalent to ";
			aOStream<<"the previous iterate,\n";
			aOStream<<"though the stopping criterion is not yet satisfied. ";
			aOStream<<"Relaxing the stopping criterion\n";
			aOStream<<"shouldsolve this problem.\n";
			break;
		case(9):
			aOStream<<"rdSQP(9): One of the penalty parameters exceeded ";
			aOStream<<"the largest allowed bound.\n";
			aOStream<<"The algorithm is having trouble satisfying a non-linear ";
			aOStream<<"equality constraint.\n";
			break;
		default:
			aOStream<<"rdSQP("<<aInform<<"): Unrecognized inform value.\n";
	}
}
