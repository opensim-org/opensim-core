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
#include <OpenSim/Common/rdMath.h>
#include <OpenSim/Common/Mtx.h>
#include "osimSQPDLL.h"
#include "rdFSQP.h"
#include "rdOptimizationTarget.h"



using namespace OpenSim;
using namespace std;
using SimTK::Vector;
using SimTK::Matrix;

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
	_maxIter = 4 * _target->getNumParameters();

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
	int nx = _target->getNumParameters();
	if(_x!=NULL) delete[] _x;
	_x = new double[nx];

	// PERFORMANCE
	int i;
	int lenp = 1 - _nfsr; // _target->getNumContacts() - _nfsr;
	for(i=0;i<_nfsr;i++)  lenp += _mesh[i];
	if(lenp<1) lenp = 1;
	if(_p!=NULL) delete[] _p;
	_p = new double[lenp];

	// CONSTRAINTS
	int lenc = _target->getNumConstraints() - _ncsrl - _ncsrn;
	for(i=0;i<_ncsrn;i++)  lenc += _mesh[i+_nfsr];
	for(i=0;i<_ncsrl;i++)  lenc += _mesh[i+_nfsr+_ncsrn];
	if(lenc<1) lenc = 1;
	if(_c!=NULL) delete[] _c;
	_c = new double[lenc];

	// LAGRANGE MULTIPLIERS
	if(_lambda!=NULL) delete[] _lambda;
	_lambda = new double[nx+lenp+lenc];

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
	int nx = _target->getNumParameters();

	double *bl, *bu;
	if(_target->getHasLimits()) {
		_target->getParameterLimits(&bl,&bu);
	} else {
		bl = new double[nx];
		bu = new double[nx];
		for(int i=0; i<nx; i++) {
			bl[i] = -_infinity;
			bu[i] =  _infinity;
		}
	}

	//printf("\nrdFSQP.computeOptimalControls: ...\n");

	// CHECK CONTROL POINTERS
	if(xin==NULL) return(-1);
	if(xout==NULL) return(-1);

	// SET INITIAL X
	int i;
	for(i=0;i<_target->getNumParameters();i++)  _x[i] = xin[i];

	// SET THE CLIENT DATA TO THIS TARGET
	void *cd = _target;

	// OPTIMIZE
	cfsqp(_target->getNumParameters(),1,_nfsr,
		_target->getNumNonlinearInequalityConstraints(),_target->getNumInequalityConstraints(),
		_target->getNumNonlinearEqualityConstraints(),_target->getNumEqualityConstraints(),
		_ncsrl,_ncsrn,_mesh,
		_mode,_printLevel,_maxIter,&_inform,_infinity,_eps,_epseqn,_udelta,
		bl,bu,_x,_p,_c,_lambda,pFunc,cFunc,dpdxFunc,dcdxFunc,cd);

	// SET OUTPUT X
	for(i=0;i<_target->getNumParameters();i++)  xout[i] = _x[i];

	if(!_target->getHasLimits()) {
		delete[] bl;
		delete[] bu;
	}

	return(_inform);
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
	int nx=target->getNumParameters();
	target->objectiveFunc(Vector(nx,x,true),true,*p);
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
	int nx=target->getNumParameters();
	target->gradientFunc(Vector(nx,x,true),true,Vector(nx,dpdx,true));
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

	// COMPUTE - this calls a wrapper that handles caching for faster constraint computation
	int nx=target->getNumParameters();
	int nc=target->getNumConstraints();
	SimTK::Vector allc(nc);
	target->constraintFunc(Vector(nx,x,true),true,allc);
	*c=allc[j-1];
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

	// COMPUTE - this calls a wrapper that handles caching for faster constraint computation
	int nx=target->getNumParameters();
	int nc=target->getNumConstraints();
	SimTK::Matrix jacobian(nc,nx);
	target->constraintJacobian(Vector(nx,x,true),true,jacobian);
	for(int col=0;col<nx;col++) dcdx[col]=jacobian(j-1,col);
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
			aOStream<<"rdFSQP(0): Normal termination.\n";
			break;
		case(1):
			aOStream<<"rdFSQP(1): User-provided initial guess is infeasible ";
			aOStream<<"for linear constraints\n";
			aOStream<<"and CFSQP is unable to generate a point satisfying these ";
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
			aOStream<<"rdFSQP(6): Failure of the QP solver in attempting to construct d1.\n";
			break;
		case(7):
			aOStream<<"rdFSQP(7): Input data are not consistent.  Set the print level";
			aOStream<<" greater than 0 for more information.\n";
			break;
		case(8):
			aOStream<<"rdFSQP(8): The new iterate is numerically equivalent to ";
			aOStream<<"the previous iterate,\n";
			aOStream<<"though the stopping criterion is not yet satisfied. ";
			aOStream<<"Relaxing the stopping criterion\n";
			aOStream<<"shouldsolve this problem.\n";
			break;
		case(9):
			aOStream<<"rdFSQP(9): One of the penalty parameters exceeded ";
			aOStream<<"the largest allowed bound.\n";
			aOStream<<"The algorithm is having trouble satisfying a non-linear ";
			aOStream<<"equality constraint.\n";
			break;
		default:
			aOStream<<"rdFSQP("<<aInform<<"): Unrecognized inform value.\n";
	}
}
