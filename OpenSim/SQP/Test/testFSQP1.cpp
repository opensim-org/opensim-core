// testFSQP1.cpp


//==============================================================================
// INCLUDES
//==============================================================================
#include <stdlib.h>
#include <stdio.h>
#include <RD/SQP/rdOptimizationTarget.h>
#include <RD/SQP/rdFSQP.h>
#include "quadTarget.h"


//==============================================================================
// ENTRY POINT
//==============================================================================
//______________________________________________________________________________


using namespace OpenSim;
/**
 * A sample C++ routine for testing CFSQP.
 * This sample program passes a pointer to a C++ class through the CFSQP
 * routines to evaluate the performance criterion and constraints.
 * The ability to do this enables CFSQP to be incorporated in an object
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
	quadTarget *target = new quadTarget(nx);

	// CREATE AN FSQP INSTANCE
	rdFSQP *fsqp = new rdFSQP(target);
	fsqp->setConvergenceCriterion(1.0e-6);
	fsqp->setUpperBound(2.2);

	// SET THE INITIAL CONTROLS
	double *x = new double[target->getNX()];
	for(int i=0;i<nx;i++) {
		x[i] = 10.0;
	}

	// OPTIMIZE
	fsqp->computeOptimalControls(x,x);


	// CLEANUP
	delete []x;
	delete fsqp;
	delete target;
}
