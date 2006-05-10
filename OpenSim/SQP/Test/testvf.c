// testvf.c

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "vf02adjz.h"
#define NX 1

//______________________________________________________________________________
/**
 * A simple routine to test vf02ad when called from C.
 */
int main(int argc,char **argv)
{
	// OPTIMIZATION VARIABLES
	int nx=NX,nc=0,nceq=0;
	double x[NX],p,c[1];
	double dpdx[NX],dxda[NX];
	double dcdx[1],mu[1];
	double sum;


	// VFO2AD PARAMETERS
	int lcn = NX+1;
	int iprint = 1;
	int inf = -1;
	int nw = 100000;
	double w[100000];
	double epscOpt = 1.0e-4;

	// INITIAL X
	x[0] = 7;

	for(int i=0;i<20;i++) {

		// COMPUTE PERFORMANCE
		p = x[0]*x[0];

		// COMPUTE DERIVATIVES
		dpdx[0] = 2.0*x[0];

		// VFO2AD
		vf02adjz_(&nx,&nc,&nceq,x,&p,dpdx,c,dcdx,&lcn,&epscOpt,&iprint,&inf,
		 w,&nw,dxda,mu,&sum);

		// CONVERGED?
		if(inf==1) {
			printf("CONVERGED\n");
			break;
		} else if(inf>1) {
			printf("Error %d\n",inf);
			break;
		}

		// LINE SEARCH
		x[0] = sqrt(x[0]);		
	}

	return(0);
}
