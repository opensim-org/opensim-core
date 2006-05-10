#ifndef _spline_h_
#define _spline_h_
// spline.h
// Functions for reading b-spline parameters from file and evaluating
// b-splines.
// Author: Clay Anderson
// Creation Date: 2000_07_28


// SO THAT ROUTINES MAY BE CALLED FROM C
#ifdef __cplusplus
extern "C" {
#endif

//==============================================================================
// INCLUDES
//==============================================================================


//==============================================================================
// DEFINES
//==============================================================================
#define MAXSIZE 256

//==============================================================================
// TYPEDEFS
//==============================================================================
//______________________________________________________________________________
/**
 * The spline structure holds the parameters which define the spline.
 */
typedef struct {
	char name[MAXSIZE];
	int order;
	int nknots,ncoefs;
	double ti,tf;
	double knots[MAXSIZE];
	double coefs[MAXSIZE];
} SplineStruct;


//==============================================================================
// METHODS
//==============================================================================

//------------------------------------------------------------------------------
// CONSTRUCTION
//------------------------------------------------------------------------------
SplineStruct* constructSpline(FILE *fp);

//------------------------------------------------------------------------------
// EVALUATION
//------------------------------------------------------------------------------
double spval(SplineStruct *sp,double x);
double s(SplineStruct *sp,double x);
double b(double *k,int i,int n,double x);
double evaluateSpline(SplineStruct *sp, double x);

//------------------------------------------------------------------------------
// UTILITY
//------------------------------------------------------------------------------
int knotIndex(SplineStruct *sp,double x);
void printSpline(SplineStruct *sp);

//------------------------------------------------------------------------------
// HIGH-LEVEL ROUTINES FOR QUERRYING A SUITE OF SPLINES
//------------------------------------------------------------------------------
int constructsplinesuite_(char *aBaseName,int &aN);
void destroysplinesuite_();
double sppos_(int &aN,double &aT);
double spvel_(int &aN,double &aT);
double spacc_(int &aN,double &aT);
bool isStateIndexValid(int aI);


//------------------------------------------------------------------------------
// TESTING
//------------------------------------------------------------------------------
void testHighLevel();
void testSplineEvaluations();


#ifdef __cplusplus
}
#endif




