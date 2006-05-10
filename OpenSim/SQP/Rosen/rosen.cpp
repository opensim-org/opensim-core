// samplePerfOnly.cpp

//==============================================================================
// INCLUDES
//==============================================================================
#include <SU/CFSQP/cfsqpusr.h>
#include <RD/SQP/rdOptimizationTarget.h>
#include <RD/SQP/Test/rosenTarget.h>


//==============================================================================
// INTERNAL FUNCTIONS
//==============================================================================
void pFunc(int nparam,int j,double *x,double *pj,void *cd);
void cFunc(int nparam,int j,double *x,double *cj,void *cd);
void gpFunc(int nparam,int j,double *x,double *dpdx,
		void (*dummy)(int,int,double *,double *,void *),void *cd);
void gcFunc(int nparam,int j,double *x,double *dcdx,
		void (*dummy)(int,int,double *,double *,void *),void *cd);


//==============================================================================
// ENTRY POINT
//==============================================================================
//______________________________________________________________________________
/**
 * A sample C++ routine for testing CFSQP.
 * This sample program passes a pointer to a C++ class through the CFSQP
 * routines to evaluate the performance criterion and constraints.  The
 * ability to do this enables CFSQP to be incorporated in an object
 * oriented programming scheme.
 *
 * 2000_01_12
 * This test program executes correctly.  However, because this is compiled
 * with C++, one must add an the following lines to the cfsqpusr.h file
 *
 * #ifdef __cplusplus
 * extern "C" {
 * #endif
 *
 * ... original .h stuff ...
 * 
 * #ifdef __cplusplus
 * extern "C" {
 * #endif
 *
 * The optimization problem is the Rosenbroch function.  Very difficult
 * if you don't make some sort of 2nd order approximation (i.e., perform
 * a Hessian update).
 */
int main()
{
	// CREATE AN OPTIMIZATION TARGET
	rdOptimizationTarget *target = new rosenTarget();

	// PARAMETERS
	int mode = 100;
	int iprint = 2;
	int miter = 500;
	int inform;
	double bigbnd = 1.0e10;
	double eps = 1.0e-8;
	double epsneq = 0.0;
	double udelta = 0.0;

	// NUMBERS
	int nx = target->getNX();
	int np = 1;
	int nineqn = 0;
	int nineq = target->getNC() - target->getNCEQ();;
	int neqn = target->getNCEQ();
	int neq = 0;

	// SEQUENTIAL
	int ncsrl = 0;
	int ncsrn = 0;
	int nfsr = 0;
	int mesh[] = { 0 };

	// ALLOCATIONS
	double *bl = new double[nx];
	double *bu = new double[nx];
	double *x = new double[nx];
	double *p = new double[np];
	double *c = new double[nineq+neq];
	double *lambda = new double[nineq+neq+np+nx];

	// LOWER BOUND ON CONTROLS
	bl[0] = bl[1] = bl[2] =  -bigbnd;
	bu[0] = bu[1] = bu[2] =  bigbnd;

	// INITIALIZATIONS
	x[0] = -1.0;
	x[1] = 1.0;

	// CLIENT DATA
	void *cd = target;

	// OPTIMIZE
	cfsqp(nx,np,nfsr,nineqn,nineq,neqn,neq,ncsrl,ncsrn,mesh,
		mode,iprint,miter,&inform,bigbnd,eps,epsneq,udelta,bl,bu,
		x,p,c,lambda,pFunc,NULL,gpFunc,NULL,cd);

	// CLEANUP
	delete bl;
	delete bu;
	delete x;
	delete p;
	delete c;
	delete lambda;
}




//==============================================================================
// USER DEFINED FUNCTIONS
//==============================================================================
//______________________________________________________________________________
/**
 * Evaluate the performance criterion.
 */
void
pFunc(int nparam,int j,double *x,double *p,void *cd)
{
	rdOptimizationTarget *target = (rdOptimizationTarget *)cd;
	target->computePerformance(x,p);
}

//______________________________________________________________________________
/**
 * Evaluate the derivatives of the performance criterion.
 */
void
gpFunc(int nparam,int j,double *x,double *dpdx,
	void (*dummy)(int,int,double *,double *,void *),void *cd)
{
	rdOptimizationTarget *target = (rdOptimizationTarget *)cd;
	target->computePerformanceGradient(x,dpdx);
}

//______________________________________________________________________________
/**
 * Evaluate the constraints.
 */
void
cFunc(int nparam,int j,double *x,double *c,void *cd)
{
	rdOptimizationTarget *target = (rdOptimizationTarget *)cd;
	target->computeConstraint(x,j,c);
}
//______________________________________________________________________________
/**
 * Evaluate the gradient of the constraints.
 */
void
gcFunc(int nparam,int j,double *x,double *dcdx,
	void (*dummy)(int,int,double *,double *,void *),
	void *cd)
{
	rdOptimizationTarget *target = (rdOptimizationTarget *)cd;
	target->computeConstraintGradient(x,j,dcdx);
}
