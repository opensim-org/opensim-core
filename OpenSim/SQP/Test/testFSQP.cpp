// samplePerfOnly.cpp

//==============================================================================
// INCLUDES
//==============================================================================
#include <SU/CFSQP/cfsqpusr.h>
#include <RD/SQP/rdOptimizationTarget.h>
#include "quadTarget.h"


//==============================================================================
// INTERNAL FUNCTIONS
//==============================================================================


using namespace OpenSim;
void pFunc(int nparam,int j,double *x,double *pj,void *cd);
void cFunc(int nparam,int j,double *x,double *cj,void *cd);
void dpdxFunc(int nparam,int j,double *x,double *dpdx,
		void (*dummy)(int,int,double *,double *,void *),void *cd);
void dcdxFunc(int nparam,int j,double *x,double *dcdx,
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
 * The optimization problem is a single non-linear performance criterion
 * with bounds on the control variables.
 */
int main(int argc,char **argv)
{

	// PARSE COMMAND LINE
	int nx;
	if(argc!=2) {
		printf("testSQP:  using nx=1\n");
		nx = 1;
	} else {
		sscanf(argv[1],"%d",&nx);
	}

	// CREATE AN OPTIMIZATION TARGET
	rdOptimizationTarget *target = new quadTarget(nx);

	// PARAMETERS
	int mode = 100;
	int iprint = 2;
	int miter = 500;
	int inform;
	double bigbnd = 1.0e10;
	double eps = 1.0e-6;
	double epseqn = 1.0e-8;
	double udelta = 0.0;

	// NUMBERS
	nx = target->getNX();
	int np = 1;
	int nineqn = target->getNCInequalityNonlinear();
	int nineq = target->getNCInequality();
	int neqn = target->getNCEqualityNonlinear();
	int neq = target->getNCEquality();

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
	int i;
	for(i=0;i<nx;i++) {
		bl[i] = -bigbnd;
		bu[i] =  bigbnd;
	}

	// INITIALIZATIONS
	for(i=0;i<nx;i++) {
		x[i] = 10.0;
	}

	// CLIENT DATA
	void *cd = target;

	// OPTIMIZE
	cfsqp(nx,np,nfsr,nineqn,nineq,neqn,neq,ncsrl,ncsrn,mesh,
		mode,iprint,miter,&inform,bigbnd,eps,epseqn,udelta,bl,bu,
		x,p,c,lambda,pFunc,cFunc,dpdxFunc,dcdxFunc,cd);

	// CLEANUP
	delete []bl;
	delete []bu;
	delete []x;
	delete []p;
	delete []c;
	delete []lambda;
	delete target;
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
dpdxFunc(int nparam,int j,double *x,double *dpdx,
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
dcdxFunc(int nparam,int j,double *x,double *dcdx,
	void (*dummy)(int,int,double *,double *,void *),
	void *cd)
{
	rdOptimizationTarget *target = (rdOptimizationTarget *)cd;
	target->computeConstraintGradient(x,j,dcdx);
}
