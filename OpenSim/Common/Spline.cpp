/* -------------------------------------------------------------------------- *
 *                            OpenSim:  Spline.cpp                            *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Frank C. Anderson                                               *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied    *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

/* Note: This code was originally developed by Realistic Dynamics Inc. 
 * Author: Frank C. Anderson 
 */


// INCLUDES
#include "Spline.h"
#include <string.h>

// DEFINES


using namespace OpenSim;
#define MAXSIZE 2048


//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
Spline::~Spline()
{
	if(_coefs!=NULL) { delete[] _coefs;  _coefs=NULL; }
	if(_knots!=NULL) { delete[] _knots;  _knots=NULL; }
	if(_tx!=NULL) { delete[] _tx;  _tx=NULL; }
	if(_b!=NULL) { delete[] _b;  _b=NULL; }
}
//_____________________________________________________________________________
/**
 * Construct a b-spline representation of a smooth function.
 *
 *	@param aFileName Name of a valid spline file.
 */
Spline::
Spline(const char *aFileName)
{
	// BASIC INITIALIZATION
	null();

	// CHECK FILE NAME
	if(aFileName==NULL) {
		printf("Spline.initialize: ERROR- file name is NULL.\n");
		_status = -1;
		return;
	}
	if(strlen(aFileName)<=0) {
		printf("Spline.initialize: ERROR- file name has no length.\n");
		_status = -1;
		return;
	}

	// OPEN FILE
	FILE *fp = fopen(aFileName,"r");
	if(fp==NULL) {
		printf("Spline.initialize: ERROR- failed to open file %s.\n",
		 aFileName);
		_status = -1;
		return;
	}

	// CONTRUCT BASED ON FILE
	_status = initialize(fp);

	// CLOSE FILE
	fclose(fp);

	// CHECK STATUS
	if(_status<0) {
		printf("Spline.Spline: WARNING- spline failed to initialize.\n");
		return;
	}

	// ALLOCATE WORKING ARRAYS
	_tx = new double[2*(_order-1)];
	_b = new double[2*_nknots];

	_status = 0;
}
//_____________________________________________________________________________
/**
 * Construct a b-spline representation of a smooth function.
 *
 *	@param a pointer to an openned spline file.
 */
Spline::
Spline(FILE *aFP)
{
	// BASIC INITIALIZATION
	null();

	// CHECK FILE POINTER
	if(aFP==NULL) {
		printf("Spline.Spline(FILE*): ERROR- invalid file pointer.\n");
		_status = -1;
		return;
	}

	// INITIALIZE
	_status = initialize(aFP);

	// CHECK STATUS
	if(_status<0) {
		printf("Spline.Spline: WARNING- spline failed to initialize.\n");
		return;
	}

	// ALLOCATE WORKING ARRAYS
	_tx = new double[2*(_order-1)];
	_b = new double[2*_nknots];

	_status = 0;
}
//_____________________________________________________________________________
/**
 * Initialize spline based on the contents of a file.
 *
 *	@param aFileName Name of a valid spline file.
 */
int Spline::
initialize(FILE *aFP)
{
	if(aFP==NULL) return(-1);
	char dum[MAXSIZE];
	int i,status;

	// NAME
	status = fscanf(aFP,"%s",_name);
	if(checkFileStatus(status)!=0) return(-1);

	// ORDER
	status = fscanf(aFP,"%s",dum);
	if(checkFileStatus(status)!=0) return(-1);
	fscanf(aFP,"%d",&_order);
	if(checkFileStatus(status)!=0) return(-1);
	if(_order<=0) {
		printf("Spline.initialize: ERROR- invalid order (%d).\n",
		 _order);
		return(-1);
	}

	// INTERVAL
	fscanf(aFP,"%s",dum);
	if(checkFileStatus(status)!=0) return(-1);
	fscanf(aFP,"%lf",&_ti);
	if(checkFileStatus(status)!=0) return(-1);
	fscanf(aFP,"%lf",&_tf);
	if(checkFileStatus(status)!=0) return(-1);
	if(_ti>_tf) {
		printf("Spline.initialize: ERROR- invalid interval (%lf to %lf).\n",
		 _ti,_tf);
		return(-1);
	}

	// NUMBER OF KNOTS
	fscanf(aFP,"%s",dum);
	if(checkFileStatus(status)!=0) return(-1);
	fscanf(aFP,"%d",&_nknots);
	if(checkFileStatus(status)!=0) return(-1);
	if(_nknots<=0) {
		printf("Spline.initialize: ERROR- invalid number of knots (%d).\n",
		 _nknots);
		return(-1);
	}

	// ALLOCATE KNOTS ARRAY
	_knots = new double[_nknots];
	if(_knots==NULL) {
		printf("Spline.initialize: ERROR- not enough memory for %d knots.\n",
		 _nknots);
		return(-1);
	}

	// KNOT VALUES
	for(i=0;i<_nknots;i++) {
		fscanf(aFP,"%lf",&_knots[i]);
		if(checkFileStatus(status)!=0) return(-1);
	}

	// NUMBER OF COEFFICIENTS
	fscanf(aFP,"%s",dum);
	if(checkFileStatus(status)!=0) return(-1);
	fscanf(aFP,"%d",&_ncoefs);
	if(checkFileStatus(status)!=0) return(-1);
	if(_ncoefs<=0) {
		printf("Spline.initialize: ERROR- invalid number of");
		printf(" coefficents (%d).\n",_ncoefs);
		return(-1);
	}

	// ALLOCATE COEFFICIENTS ARRAY
	_coefs = new double[_ncoefs];
	if(_coefs==NULL) {
		printf("Spline.initialize: ERROR- not enough memory for %d",_ncoefs);
		printf(" coefficents.\n");
		return(-1);
	}

	// COEFFICENT VALUES
	for(i=0;i<_ncoefs;i++) {
		status = fscanf(aFP,"%lf",&_coefs[i]);
		if(checkFileStatus(status)!=0) return(-1);
	}

	return(0);
}
//_____________________________________________________________________________
/**
 * NULL or zero data.
 */
void Spline::
null()
{
	_status = -1;
	_ti = 0.0;
	_tf = 0.0;
	_order = 0;
	_nknots = 0;
	_ncoefs = 0;
	_knots = NULL;
	_coefs = NULL;
	_tx = NULL;
	_b = NULL;
}
//_____________________________________________________________________________
/**
 * Check the status of a file read.
 *
 *	@param aFileName Name of a valid spline file.
 */
int Spline::
checkFileStatus(int aStatus)
{
	if(aStatus==EOF) {
		printf("Spline.checkFileStatus: ERROR during file read.\n");
		return(-1);
	}
	return(0);
}


//=============================================================================
// EVALUATION
//=============================================================================
//______________________________________________________________________________
/**
 * Get the knot index for the interval in which x falls.
 * If x does not fall within a valid interval, -1 is returned.
 */
int Spline::
getKnotIndex(double x) const
{
	int index=-1;
	int i;
	for(i=0;i<_nknots-1;i++) {
		if((_knots[i]<=x)&&(x<_knots[i+1])) {
			index = i;
			break;
		}
	}
	return(index);
}
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
double Spline::calcValue(const SimTK::Vector& x) const
{
	// INITIALIZATIONS
	assert(x.size()==1);
	int i,j;
	int k = _order;
	double *t = _knots;
	double *a = _coefs;

	// GET THE KNOT INEX
	int index = getKnotIndex(x[0]);
	if(index<0) {
		printf("Spline.evaluate: WARNING- x (%lf) is not",x[0]);
		printf(" on the spline interval (%lf to %lf).\n",_ti,_tf);
		return(0.0);
	}

	// INITIALIZE THE PERTINENT KNOT VALUES
	for(i=0;i<2*(k-1);i++) {
		_tx[i] = t[index-k+2+i] - x[0];
	}

	// INITIALIZE THE SEED VALUES OF b
	double num,den;
	for(j=0,i=index-k+1;i<=index;i++,j++) {
		_b[j] = a[i];
	}

	// LOOP
	for(int r=0;r<k-1;r++) {
		for(i=0;i<k-r-1;i++) {
			num = _tx[i+k-1]*_b[i] - _tx[i+r]*_b[i+1];
			den = _tx[i+k-1] - _tx[i+r];
			_b[i] = num/den;
		}
	}
	return(_b[0]);
}


//=============================================================================
// PRINTING
//=============================================================================
//______________________________________________________________________________
/**
 * Print a spline.
 */
void Spline::
print()
{
	int i;

	// NAME
	printf("name= %s\n",_name);

	// ORDER
	printf("order= %d\n",_order);

	// INTERVAL
	printf("interval=  %lf %lf\n",_ti,_tf);

	// KNOTS
	printf("nknots= %d\n",_nknots);
	for(i=0;i<_nknots;i++) {
		printf("%lf ",_knots[i]);
	}
	printf("\n");

	// COEFFICIENTS
	printf("ncoefs= %d\n",_ncoefs);
	for(i=0;i<_ncoefs;i++) {
		printf("%lf ",_coefs[i]);
	}
	printf("\n");
}
