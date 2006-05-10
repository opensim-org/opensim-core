// testSQP.c


// INCLUDES
#include <stdlib.h>
#include <stdio.h>
#include <RD/Tools/rdMath.h>
#include <RD/SQP/rdSQP.h>
#include "quadTarget.h"



//______________________________________________________________________________
/**
 * Test the rdSQP libraries.
 */
int main(int argc,char **argv)
{

	// PARSE COMMAND LINE
	int i,nx;
	if(argc!=2) {
		printf("testSQP:  using nx=1\n");
		nx = 1;
	} else {
		sscanf(argv[1],"%d",&nx);
	}

	// CONSTRUCT THE OPTIMIZATION TARGET
	quadTarget *target = new quadTarget(nx);

	// CONSTRUCT THE SQP INSTANCE
	rdSQP *sqp = new rdSQP(target);
	sqp->setMaxEvaluations(1000);
	sqp->setMaxIterations(100);
	sqp->setConvergenceCriterion(1.0e-6);
	sqp->setMinAlpha(1.0e-6);
	sqp->setMaxAlpha(1.0e6);

	// CONTROLS
	nx = target->getNX();
	double *x = new double[nx];
	double *xopt = new double[nx];
	for(i=0;i<nx;i++) x[i] = 1.2;

	// PRINT
	printf("\n\n-------------\n");
	printf("testSQP: solving for minimum of N dimensional quadratic.\n");
	printf("initial x:\n");
	for(i=0;i<nx;i++) printf("x[%d]=%lf\n",i,x[i]);

	// PERFORMANCE CONSTRAINTS
	double p;
	int nc = target->getNC();
	int nceq = target->getNCEquality();
	double *c = NULL;
	if(nc>0)  c = new double[nc];

	// INITIAL PERFORMANCE
	target->compute(x,&p,c);
	printf("testSQP:  initial p=%lf\n",p);

	// COMPUTE DERIVATIVES
	sqp->computeOptimalControls(x,xopt);

	// EXAMINE SOLUTION
	target->compute(xopt,&p,c);
	printf("testSQP:  final p=%lf\n",p);

	// CLEANUP
	delete sqp;
	delete target;
	delete []x;
	delete []xopt;
	if(c!=NULL) delete []c;
}
