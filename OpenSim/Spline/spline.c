// spline.c
// Functions for reading b-spline parameters from file and evaluating
// b-splines.
// Author: Clay Anderson
// Creation Date: 2000_07_28

//==============================================================================
// INCLUDES
//==============================================================================
#include <stdlib.h>
#include <stdio.h>
#include "spline.h"


//==============================================================================
// GLOBAL VARIABLES
//==============================================================================
static int _nsp = 0;
static SplineStruct *_spPos[MAXSIZE];
static SplineStruct *_spVel[MAXSIZE];
static SplineStruct *_spAcc[MAXSIZE];



//==============================================================================
// CONSTRUCTION
//==============================================================================

//______________________________________________________________________________
/**
 * Read b-spline parameters from an open file and construct a spline structure.
 *
 * VARIABLES:
 * 	fp			file pointer
 *
 */
SplineStruct*
constructSpline(FILE *fp)
{
	char dum[MAXSIZE];
	int i;

	// ALLOCATE SPLINE STRUCTURE
	SplineStruct *sp = (SplineStruct*)malloc(sizeof(SplineStruct));

	// NAME
	fscanf(fp,"%s",sp->name);

	// ORDER
	fscanf(fp,"%s",dum);
	fscanf(fp,"%d",&(sp->order));

	// INTERVAL
	fscanf(fp,"%s",dum);
	fscanf(fp,"%lf",&(sp->ti));
	fscanf(fp,"%lf",&(sp->tf));

	// KNOTS
	fscanf(fp,"%s",dum);
	fscanf(fp,"%d",&(sp->nknots));
	for(i=0;i<sp->nknots;i++) {
		fscanf(fp,"%lf",&(sp->knots[i]));
	}

	// COEFFICIENTS
	fscanf(fp,"%s",dum);
	fscanf(fp,"%d",&(sp->ncoefs));
	for(i=0;i<sp->ncoefs;i++) {
		fscanf(fp,"%lf",&(sp->coefs[i]));
	}

	return(sp);
}


//==============================================================================
// EVALUATION
//==============================================================================

//------------------------------------------------------------------------------
// MATLAB STYLE
//------------------------------------------------------------------------------
//______________________________________________________________________________
/**
 * Evaluate a spline at x.
 * 
 * This function is based on Matlab's spval.m.
 *
 * 2000_08_10
 * This routine assumes that the end knots have the proper multiplicity.
 *
 */
double
spval(SplineStruct *sp,double x)
{
	// INITIALIZATIONS
	int i,j;
	int k = sp->order;
	double *t = sp->knots;
	double *a = sp->coefs;

	// GET THE KNOT INEX
	int index = knotIndex(sp,x);

	// INITIALIZE THE PERTINENT KNOT VALUES
	double tx[MAXSIZE];
	for(i=0;i<2*(k-1);i++) {
		tx[i] = t[index-k+2+i] - x;
	}

	// INITIALIZE THE SEED VALUES OF b
	double num,den,b[MAXSIZE];
	for(j=0,i=index-k+1;i<=index;i++,j++) {
		b[j] = a[i];
	}

	// LOOP
	for(int r=0;r<k-1;r++) {
		for(i=0;i<k-r;i++) {
			num = tx[i+k-1]*b[i] - tx[i+r]*b[i+1];
			den = tx[i+k-1] - tx[i+r];
			b[i] = num/den;
		}
	}
	return(b[0]);
}

//------------------------------------------------------------------------------
// RECURSIVE NO MULTIPLICITY from Lancaster and Salkaulkas (1986).
//------------------------------------------------------------------------------
//______________________________________________________________________________
/**
 * Evaluate the function, s(x), given its b-spline form sp.
 */
double
s(SplineStruct *sp,double x)
{
	double sval=0.0;
	int i;
	int n = sp->order;

	// FIND THE INDEX
	int index = knotIndex(sp,x);

	// LOOP
	for(i=index-n+1;i<=index+1;i++) {
		sval += sp->coefs[i]*b(sp->knots,i,n,x);
	}

	return(sval);
}
//______________________________________________________________________________
/**
 * Compute the b-spline for a given spline (sp) and x.
 * This computation is performed using the recursive formula developed by
 * de Boor.  For details, see the reference
 *
 * Lancaster P., Salkauskas K. (1986).  Curve and surface fitting: A
 * introduction.  Academic Press, San Diego.  p. 99.
 *
 * This algorithm does not allow for multiplicity (i.e., knots which
 * have the same break points).
 *
 * VARIABLES:
 *		k		Array of knots.  It is assumed that k is the appropriate size.
 *				The knots are the values of the independent variable at which
 *				the curve is broken into its separate polynomial pieces.
 *		i		The index of the b-spine.
 *		n		Order of the spline.
 *		x		Independent variable.
 */
double
b(double *k,int i,int n,double x)
{
	double bval;
	double num1,num2;
	double den1,den2;

	num1 = x - k[i];					num2 = k[i+n] - x;
	den1 = k[i+n-1] - k[i];			den2 = k[i+n] - k[i+1];

	// CHECK FOR ZERO DENOMINATORS
	if((den1==0.0)&&(n!=1)) {
		printf("WARNING- den1 divide by zero! x=%lf,i=%d,n=%d\n",x,i,n);
	}
	if((den2==0.0)&&(n!=1)) {
		printf("WARNING- divide by zero! x=%lf,i=%d,n=%d\n",x,i,n);
	}

	// DOWN TO ORDER 1
	if(n==1) {
		if(x < k[i]) {
			bval = 0;
		} else if(x >= k[i+1]) {
			bval = 0;
		} else {
			bval = 1;
		}

	// ORDERS HIGHER THAN 1
	} else {
		bval = b(k,i,n-1,x)*num1/den1 + b(k,i+1,n-1,x)*num2/den2;
	}

	return(bval);
}

//------------------------------------------------------------------------------
// DE BOOR ALGORITHM  FROM Faren (1990).
//------------------------------------------------------------------------------
//______________________________________________________________________________
/**
 * 2000_08_10
 * DOESN'T WORK - There is a time lag in the function.
 *
 * Return the value of a spline given the spline and the value of the
 * independent variable.
 *
 * This routine is based on a an implementation published in:
 *
 * Farin, Gerald (1990). Curves and surfaces for computer aided geometric
 * design, A practical guide. 2nd Edition. Academic Press, Inc., Boston. p. 170.
 *
 * VARIABLES:
 * 	spline	structure which contains the spline parameters.
 *		t			the independent variable (e.g., time).
 */
double
evaluateSpline(SplineStruct *sp,double t)
{
	int i,j,k;

	// CHECK t
	if((t<sp->ti)||(t>sp->tf)) {
		printf("evaluateSpline: ERROR- t isn't within defined spline interval");
		return(0.0);
	}

	// DETERMIN KNOT INDEX
	int n = sp->order;
	i = knotIndex(sp,t);

	// PRINT PARAMETERS
	//printf("i=%d,knot[%d]=%lf,knot[%d]=%lf,t=%lf\n",
	// i,i,sp->knots[i],i+1,sp->knots[i+1],t);
	
	// COPY COEFFICIENTS
	double c[MAXSIZE];
	for(j=0;j<sp->ncoefs;j++) c[j] = 0.0;
	for(j=0;j<sp->ncoefs;j++) c[j] = sp->coefs[j];
	//for(j=i-n+1;j<=i+1;j++)  c[j] = sp->coefs[j];
	//printf("initialized(%d,%d)",i-n+1,i+1);

	// RECURSIVE EVALUATION
	int minc=1000000,maxc=-1000000;
	double t1,t2;
	for(k=1;k<=n;k++) {
		for(j=i+1;j>=i-n+k+1;j--) {

			t1 = (sp->knots[j+n-k] - t) /
			     (sp->knots[j+n-k] - sp->knots[j-1]);
			t2 = 1.0 - t1;

			c[j] = t1*c[j-1] + t2*c[j];
			if(j>maxc) maxc=j;
			if((j-1)<minc) minc=j-1; 
		}
	}
	printf("  accessed(%d,%d)\n",minc,maxc);
	

	return(c[i+1]);
}


//==============================================================================
// UTILITY
//==============================================================================
//______________________________________________________________________________
/**
 * Get the knot index for the interval in which x falls.
 */
int
knotIndex(SplineStruct *sp,double x)
{
	int i;
	for(i=0;i<sp->nknots;i++) {
		if((sp->knots[i]<=x)&&(x<sp->knots[i+1])) break;
		//if((sp->knots[i-1]<=x)&&(x<sp->knots[i])) break;
	}
	return(i);
}
//______________________________________________________________________________
/**
 * Print a spline structure.
 */
void
printSpline(SplineStruct *sp)
{
	int i;

	// NAME
	printf("name= %s\n",sp->name);

	// ORDER
	printf("order= %d\n",sp->order);

	// INTERVAL
	printf("interval=  %lf %lf\n",sp->ti,sp->tf);

	// KNOTS
	printf("nknots= %d\n",sp->nknots);
	for(i=0;i<sp->nknots;i++) {
		printf("%lf ",sp->knots[i]);
	}
	printf("\n");

	// COEFFICIENTS
	printf("ncoefs= %d\n",sp->ncoefs);
	for(i=0;i<sp->ncoefs;i++) {
		printf("%lf ",sp->coefs[i]);
	}
	printf("\n");
}


//==============================================================================
// HIGH-LEVEL ROUTINES FOR QUERRYING A SUITE OF SPLINES
//==============================================================================
//------------------------------------------------------------------------------
// CONSTRUCTION
//------------------------------------------------------------------------------
//______________________________________________________________________________
/**
 * Initialize a suite of splines and store the spline structures in the
 * global array spPos[MAXSIZE], spVel[MAXSIZE], and spAcc[MAXSIZE].
 *
 * The suite is a sequence of files which all have some base name in common
 * and are appended by a number (starting with 1, not 0) which are associated
 * with some state of the system.
 *
 * Note that it is currently assumed that each spline file contains a spline
 * representation for the displacement, velocity, and acceleration of
 * some state.
 *
 * ARGUMENTS:
 *		aBaseName	the base name of a suite of spline files.
 *		aN				the number of files in the suite.
 *
 * RETURNS:
 * 	0		successful initialization.
 *		-1		error encountered.
 */
int
constructsplinesuite_(char* aBaseName,int &aN)
{
	FILE *fp;
	int i;
	char fileName[MAXSIZE];

	// INTRO
	printf("\nConstructing splines for suite %s...\n\n",aBaseName);

	// CHECK STORAGE SPACE
	int n = aN;
	if(n>MAXSIZE) {
		printf("initializeSplines: ERROR- too many states...\n");
		printf("\tThe currentl limit is %d.  %d were asked for.\n",MAXSIZE,n);
		return(-1);
	}

	// NULL OUT ALL GLOBAL SPLINE ARRAYS
	for(i=0;i<MAXSIZE;i++) {
		_spPos[i] = NULL;
		_spVel[i] = NULL;
		_spAcc[i] = NULL;
	}

	// LOOP TO CONSTRUCT ALL SPLINES
	int state;
	for(i=0;i<n;i++) {

		// SET STATE NUMBER
		state = i + 1;

		// OPEN FILE
		sprintf(fileName,"%s%d",aBaseName,state);
		fp = fopen(fileName,"r");
		if(fp==NULL) {
			printf("constructSplineSuite: ERROR- could not open %s.\n",fileName);
			return(-1);
		}

		// CONSTRUCT AND STORE SPLINES
		printf("initializeSpline: constructing for state %d.\n",state);
		_spPos[i] = constructSpline(fp);
		_spVel[i] = constructSpline(fp);
		_spAcc[i] = constructSpline(fp);
		_nsp++;

		// CLOSE FILE
		fclose(fp);
	}
	printf("\n");

	return(0);
}

//------------------------------------------------------------------------------
// DESCRUCTION
//------------------------------------------------------------------------------
//______________________________________________________________________________
/**
 * Free the memory associated with the spline suite.
 */
void
destroysplinesuite_()
{
	printf("destroySplineSuite: freeing allocated memory.\n\n");
	for(int i=0;i<_nsp;i++) {
		if(_spVel[i]!=NULL) { free(_spVel[i]); _spVel[i]=NULL; }
		if(_spAcc[i]!=NULL) { free(_spAcc[i]); _spAcc[i]=NULL; }
	}
	_nsp = 0;
}

//------------------------------------------------------------------------------
// EVALUATION
//------------------------------------------------------------------------------
//______________________________________________________________________________
/**
 * Evaluate the postion or displacement of state aI at time aT.
 * Note that state 1 is indexed by 0, and state n by n-1.
 */
double
sppos_(int &aI,double &aT)
{
	if(!isStateIndexValid(aI)) return(1.0);
	return( spval(_spPos[aI],aT) );
}
//______________________________________________________________________________
/**
 * Evaluate the velocity of a state at time aT.
 * Note that state 1 is indexed by 0, and state n by n-1.
 */
double
spvel_(int &aI,double &aT)
{
	if(!isStateIndexValid(aI)) return(1.0);
	return( spval(_spVel[aI],aT) );
}
//______________________________________________________________________________
/**
 * Evaluate the acceleration of a state at time aT.
 * Note that state 1 is indexed by 0, and state n by n-1.
 */
double
spacc_(int &aI,double &aT)
{
	if(!isStateIndexValid(aI)) return(1.0);
	return( spval(_spAcc[aI],aT) );
}
//______________________________________________________________________________
/**
 * Is a state index valid.
 */
bool
isStateIndexValid(int aI)
{
	if((aI<0)||(aI>=_nsp)) {
		printf("isStateIndexValid: ERROR- index of %d is out of bounds.\n",aI);
		printf("\tA state index, i, should be 0<= i <%d.\n",_nsp);
		return(false);
	}
	return(true);
}



//==============================================================================
// ENTRY POINT FOR TESTING
//==============================================================================
//______________________________________________________________________________
/**
 * Main routine for testing.
int
main(int argc,char **argv)
{
	// TEST EVALUATIONS
	//testSplineEvaluations();

	// TEST HIGH-LEVEL SUITE ROUTINES
	testHighLevel();

	return(0);
}
*/


//______________________________________________________________________________
/**
 * Routine for testing high-level spline routines.
 */
void
testHighLevel()
{
	// INITIALIZATION
	int n = 23;
	constructsplinesuite_("subave.sp",n);

	// EVALUATION
	double t=0.0;
	for(int i=-1;i<24;i++) {
		printf("spPos%d(0.0) = %lf\n",i+1,sppos_(i,t));
		printf("spVel%d(0.0) = %lf\n",i+1,spvel_(i,t));
		printf("spAcc%d(0.0) = %lf\n",i+1,spacc_(i,t));
		printf("\n");
	}

	// DESTRUCTION
	destroysplinesuite_();
}


//______________________________________________________________________________
/**
 * Routine for testing spline evaluations.
 */
void
testSplineEvaluations()
{
	// OPEN FILE
	FILE *fout,*fp = fopen("subave.sp23","r");

	// CONSTRUCT
	SplineStruct *sp = constructSpline(fp);
	SplineStruct *dsp = constructSpline(fp);
	SplineStruct *ddsp = constructSpline(fp);

	// CLOSE FILE
	fclose(fp);

	// PRINT
	//printSpline(sp);
	//printSpline(dsp);
	//printSpline(ddsp);

	// EVALUATE
	double value;
	fout = fopen("spout","w");
	double n = 10000;
	double t,dt = (sp->tf - sp->ti) / (double)n;
	for(t=sp->ti;t<sp->tf;t+=dt) {
		value = spval(ddsp,t);
		fprintf(fout,"%lf %lf\n",t,value);
	}

	// CLEANUP
	free(sp);
	fclose(fout);
}

