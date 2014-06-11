/* -------------------------------------------------------------------------- *
 *                             OpenSim:  Mtx.cpp                              *
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
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

/* Note: This code was originally developed by Realistic Dynamics Inc. 
 * Author: Frank C. Anderson 
 */

#include "osimCommonDLL.h"
#include "Mtx.h"
#include <string.h> // for memcpy in linux


//=============================================================================
// STATIC VARIABLES
//=============================================================================


using namespace OpenSim;
using SimTK::Vec3;

int Mtx::_PSpaceSize = 0;
int Mtx::_WSpaceSize = 0;
double** Mtx::_P1Space = NULL;
double** Mtx::_P2Space = NULL;
double*  Mtx::_WSpace = NULL;
static const double eps = std::numeric_limits<double>::epsilon();


//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================

//-----------------------------------------------------------------------------
// INTERPOLATION
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Linearly interpolate or extrapolate an array of data.
 */
void Mtx::
Interpolate(int aN,double aT1,double *aY1,double aT2,double *aY2,
	double aT,double *aY)
{
	// DETERMINE PERCENT OF INTERVAL
	int i;
	double pct;
	double num = aT-aT1;
	double den = aT2-aT1;
	if(den==0.0) {
		pct = 0.0;
	} else {
		pct = num/den;
	}

	// LOOP OVER ARRAY
	for(i=0;i<aN;i++) {
		if(pct==0.0) {
			aY[i] = aY1[i];
		} else {
			aY[i] = aY1[i] + pct*(aY2[i]-aY1[i]);
		}
	}
}
//_____________________________________________________________________________
/**
 * Linearly interpolate or extrapolate two data points.
 */
double Mtx::
Interpolate(double aT1,double aY1,double aT2,double aY2,double aT)
{
	// DETERMINE PERCENT OF INTERVAL
	double pct;
	double num = aT-aT1;
	double den = aT2-aT1;
	if(den==0.0) {
		pct = 0.0;
	} else {
		pct = num/den;
	}

	// INTERP
	double y;
	if(pct==0.0) {
		y = aY1;
	} else {
		y = aY1 + pct*(aY2-aY1);
	}
	return(y);
}


//=============================================================================
// TRANSLATION AND ROTATION
//=============================================================================
//-----------------------------------------------------------------------------
// TRANSLATION
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Translate a point by a specified amount.
 *
 * @param aX Amount to translate in the X direction.
 * @param aY Amount to translate in the Y direction.
 * @param aZ Amount to translate in the Z direction.
 * @param aP Point to translate.
 * @param rP Translated point.  aP and rP may coincide in memory.
 */
void Mtx::
Translate(double aX,double aY,double aZ,const double aP[3],double rP[3])
{
	rP[0] = aP[0] + aX;
	rP[1] = aP[1] + aY;
	rP[2] = aP[2] + aZ;
}

//-----------------------------------------------------------------------------
// ROTATE BY RADIANS
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Rotate a point about the X, Y, or Z axis by a specified amount.
 *
 * @param aXYZ Specify whether to rotate about the X (aXYZ=0), Y (aXYZ=1),
 * or Z (aXYZ=2) axes.  If aXYZ is not 0, 1, or 2, no rotation is performed.
 * @param aRadians Amount of rotation in radians.
 * @param aP Point to rotate.
 * @param rP Rotated point.  aP and rP may coincide in memory.
 */
void Mtx::
Rotate(int aXYZ,double aRadians,const double aP[3],double rP[3])
{
	if(aP==NULL) return;
	if(rP==NULL) return;

	// COPY
	double p0 = aP[0];
	double p1 = aP[1];
	double p2 = aP[2];

	// COMPUTE SIN AND COS
	double c = cos(aRadians);
	double s = sin(aRadians);

	// X
	if(aXYZ==0) {
		rP[0] =  p0;
		rP[1] =  c*p1 - s*p2;
		rP[2] =  s*p1 + c*p2;

	// Y
	} else if(aXYZ==1) {
		rP[0] =  c*p0 + s*p2;
		rP[1] =  p1;
		rP[2] = -s*p0 + c*p2;

	// Z
	} else if(aXYZ==2) {
		rP[0] =  c*p0 - s*p1;
		rP[1] =  s*p0 + c*p1;
		rP[2] =  p2;
	}
}
//_____________________________________________________________________________
/**
 * Rotate a point about a specified axis by a specified amount.
 *
 * @param aAxis Axis about which to rotate.  The axis is not assumed to be
 * a unit vector; however, if the axis is the zero vector, no rotation
 * is performed.
 * @param aRadians Amount of rotation in radians.
 * @param aP Point to rotate.
 * @param rP Rotated point.  aP and rP may coincide in memory.
 */
void Mtx::
Rotate(const double aAxis[3],double aRadians,const double aP[3],double rP[3])
{
	if(aAxis==0) return;
	if(aP==NULL) return;
	if(rP==NULL) return;

}

//-----------------------------------------------------------------------------
// ROTATE BY DEGREES
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Rotate a point about the X axis by a specified amount.
 *
 * @param aXYZ Specify whether to rotate about the X (aXYZ=0), Y (aXYZ=1),
 * or Z (aXYZ=2) axis.  If aXYZ is not 0, 1, or 2, no rotation is performed.
 * @param aDegrees Amount of rotation in degrees.
 * @param aP Point to rotate.
 * @param rP Rotated point.  aP and rP may coincide in memory.
 */
void Mtx::
RotateDeg(int aXYZ,double aDegrees,const double aP[3],double rP[3])
{
	Rotate(aXYZ,SimTK_DEGREE_TO_RADIAN*aDegrees,aP,rP);
}
//_____________________________________________________________________________
/**
 * Rotate a point about a specified axis by a specified amount.
 *
 * @param aAxis Axis about which to rotate.
 * @param aDegrees Amount of rotation in degrees.
 * @param aP Point to rotate.
 * @param rP Rotated point.  aP and rP may coincide in memory.
 */
void Mtx::
RotateDeg(const double aAxis[3],double aDegrees,const double aP[3],double rP[3])
{
	Rotate(aAxis,SimTK_DEGREE_TO_RADIAN*aDegrees,aP,rP);
}


//=============================================================================
// MATRIX ARITHMATIC
//=============================================================================
//_____________________________________________________________________________
/**
 * Assign a square matrix to the identidy matrix:  rI = I.
 * The matrix must be square.
 *
 * @param aN Dimension of the matrix (aN=nRows, aN=nCols).
 * @param rI Ouput identity matrix (rI = I).
 * @return 0 on success, -1 on error.
 *
 */
int Mtx::
Identity(int aN,double *rI)
{
	if(rI==NULL) return(-1);
	if(aN<=0) return(-1);

	// ASSIGN
	int i,j;
	for(i=0;i<aN;i++) {
		for(j=0;j<aN;j++,rI++) {
			if(i==j) {
				*rI = 1.0;
			} else {
				*rI = 0.0;
			}
		}
	}

	return(0);
}
//_____________________________________________________________________________
/**
 * Add two matrices.
 *
 * If the arguments are not valid, then a -1 is returned.
 * Othersise, 0 is returned.
 *
 * It is permissible for aM to coincide with either aM1 or aM2.
 */
int Mtx::
Add(int aNR,int aNC,const double *aM1,const double *aM2,double *aM)
{
	if(aM1==NULL) return(-1);
	if(aM2==NULL) return(-1);
	if(aM ==NULL) return(-1);
	if(aNR<=0) return(-1);
	if(aNC<=0) return(-1);

	// MULTIPLY
	int i,n=aNR*aNC;
	for(i=0;i<n;i++,aM1++,aM2++,aM++)  *aM = *aM1 + *aM2;

	return(0);
}
//_____________________________________________________________________________
/**
 * Subtract two matrices.
 *
 * If the arguments are not valid, then a -1 is returned.
 * Othersise, 0 is returned.
 *
 * It is permissible for aM to coincide with either aM1 or aM2. aM1 - aM2
 */
int Mtx::
Subtract(int aNR,int aNC,const double *aM1,const double *aM2,double *aM)
{
	if(aM1==NULL) return(-1);
	if(aM2==NULL) return(-1);
	if(aM ==NULL) return(-1);
	if(aNR<=0) return(-1);
	if(aNC<=0) return(-1);

	// MULTIPLY
	int i,n=aNR*aNC;
	for(i=0;i<n;i++,aM1++,aM2++,aM++)  *aM = *aM1 - *aM2;

	return(0);
}
//_____________________________________________________________________________
/**
 * Multiply a matrix by a scalar.
 *
 * It is permissible for aM and rM to coincide in memory.
 *
 * @param aNR Number of rows in aM.
 * @param aNC Number of columns in aM.
 * @param aM Matirx laid out in memory as aM[aNR][aNC].
 * @param aScalar Scalar value by which to multiply aM.
 * @param rM Result of aScalar * aM.
 * @return -1 if an error is encountered, 0 otherwise.
 */
int Mtx::
Multiply(int aNR,int aNC,const double *aM,double aScalar,double *rM)
{
	if(aM==NULL) return(-1);
	if(rM ==NULL) return(-1);
	if(aNR<=0) return(-1);
	if(aNC<=0) return(-1);

	// MULTIPLY
	int i,n=aNR*aNC;
	for(i=0;i<n;i++,aM++,rM++)  *rM = *aM * aScalar;

	return(0);
}
//_____________________________________________________________________________
/**
 * Multiply two matrices.
 *
 * If the arguments are not valid (aM1,aM2,aM==NULL), then a -1 is returned.
 * Othersise, 0 is returned.
 *
 * It is permissible for aM to overlap with either aM1 or aM2.
 */
int Mtx::
Multiply(int aNR1,int aNCR,int aNC2,const double *aM1,const double *aM2,
	double *rM)
{
	if(aM1==NULL) return(-1);
	if(aM2==NULL) return(-1);
	if(rM ==NULL) return(-1);
	if(aNR1<=0) return(-1);
	if(aNCR<=0) return(-1);
	if(aNC2<=0) return(-1);

	// ENSURE WORKSPACE CAPACITY
	EnsureWorkSpaceCapacity(aNR1*aNC2);

	// SET POINTER INTO WORKSPACE
	double *m = _WSpace;

	// MULTIPLY
	const double *ij1=NULL,*ij2=NULL;
	double result,*ij=NULL;
	int r1,cr,c2;
	for(r1=0,ij=m;r1<aNR1;r1++) {

		for(c2=0;c2<aNC2;c2++,ij++) {

			// SET POINTERS TO BEGINNIG OF ROW OF aM1 AND COLUMN OF aM2
			ij1 = aM1 + r1*aNCR;
			ij2 = aM2 + c2;

			// MULTIPLY ROW OF aM1 AND COLUMN OF aM2
			for(result=0.0,cr=0;cr<aNCR;cr++,ij1++,ij2+=aNC2) {
				result += (*ij1) * (*ij2);
			}
			*ij = result;
		}
	}

	// COPY RESULTS INTO rM
	memcpy(rM,m,aNR1*aNC2*sizeof(double));

	return(0);
}
//_____________________________________________________________________________
/**
 * Compute the inverse of a matrix.
 *
 * If the arguments are not valid (aM==NULL), then -1 is returned.
 * If the matrix is not invertible, then -2 is returned.
 * Othersise, 0 is returned.
 *
 * It is permissible for aM to overlap in memory with aMInv.
 */
int Mtx::
Invert(int aN,const double *aM,double *rMInv)
{
	if(aN<=0) return(-1);
	if(aM==NULL) return(-1);
	if(rMInv==NULL) return(-1);

	// VARIABLE DECLARATIONS
	double *M,**Mp,**Mr,**Ip,**Ir,*Mrj,*Irj,*Mij,*Iij,d;
	int r,i,j,n;

	// ENSURE WORKSPACE CAPACITY
	EnsureWorkSpaceCapacity(aN*aN);
	EnsurePointerSpaceCapacity(aN);

	// INITIALIZE M (A COPY OF aM)
	n = aN*aN*sizeof(double);
	M = _WSpace;
	memcpy(M,aM,n);

	// INITIALIZE rMInv TO THE IDENTITY MATRIX
	memset(rMInv,0,n);
	for(r=0,Irj=rMInv,n=aN+1;r<aN;r++,Irj+=n)  *Irj=1.0;

	// INITIALIZE ROW POINTERS
	Mp = _P1Space;		// POINTER TO BEGINNING OF POINTER1 SPACE
	Mr = _P1Space;		// ROW POINTERS INTO M
	Ip	= _P2Space;		// POINTER TO BEGINNING OF POINTER2 SPACE
	Ir = _P2Space;		// ROW POINTERS INTO aMInv
	for(r=0;r<aN;r++,Mr++,Ir++) {
		i = r*aN;
		*Mr = M + i;
		*Ir = rMInv + i;
	}

	// REDUCE MATRIX TO UPPER TRIAGULAR USING ROW OPERATIONS
	for(r=0,n=aN-1;r<n;r++) {

		// SWAP
		if(std::fabs(*(Mrj=Mp[r]+r)) < eps ) {
			for(i=r+1;i<aN;i++) if(std::fabs(*(Mp[i]+r)) > eps ) break;

			// NON-INVERTIBLE?
			if(i==aN) return(-2);

			// SWAP
			Mrj=Mp[r];	Mp[r]=Mp[i];	Mp[i]=Mrj;	Mrj=Mp[r]+r;
			for(j=0,Irj=Ip[r],Iij=Ip[i];j<aN;j++) {
					d= *Irj;  *(Irj++) = *Iij;	*(Iij++) = d; }
		}

		// REDUCE
		for(i=r+1;i<aN;i++) {
			if(std::fabs(*(Mij=Mp[i]+r)) < eps ) continue;
			Mrj=Mp[r]+r;
			d = (*Mij)/(*Mrj);	*(Mij++)=0.0;	Mrj++;
			for(j=r+1;j<aN;j++) *(Mij++) -= *(Mrj++)*d;
			Irj=Ip[r];	Iij=Ip[i];
			for(j=0;j<aN;j++) *(Iij++) -= *(Irj++)*d;
		}
	}

	// NORMALIZE LAST ROW OF M AND rMInv
	if(std::fabs(*(Mrj=Mp[r]+r)) < eps) return(-2);
	d = 1.0 / *Mrj;	*Mrj=1.0;
	for(j=0,Irj=Ip[r];j<aN;j++) *(Irj++) *= d;

	// DIAGONALIZE USING ROW OPERATIONS
	for(r=aN-1;r>0;) {
		for(i=r-1;i>=0;i--) {
			if(std::fabs(*(Mij=Mp[i]+r)) < eps) continue;
			d = *Mij;	*Mij=0.0;
			for(j=0,Iij=Ip[i],Irj=Ip[r];j<aN;j++) *(Iij++) -= *(Irj++)*d;
		}
		r--;
		d = 1.0 / *(Mrj=Mp[r]+r);	*Mrj=1.0;
		for(j=0,Irj=Ip[r];j<aN;j++) *(Irj++) *= d;
	}

	return(0);
}
//_____________________________________________________________________________
/**
 * Transpose a matrix.
 *
 * If the arguments are invalid (e.g.,aM==NULL), then a -1 is returned.
 * Othersise, 0 is returned.
 *
 * It is permissible for aM to overlap in memory with aMT.
 */
int Mtx::
Transpose(int aNR,const int aNC,const double *aM,double *rMT)
{
	if(aNR<=0) return(-1);
	if(aNC<=0) return(-1);
	if(aM==NULL) return(-1);
	if(rMT==NULL) return(-1);

	// ENSURE WORKSPACE CAPACITY
	int n = aNR*aNC;
	EnsureWorkSpaceCapacity(n);

	// SET UP COUNTERS AND POINTERS
	int r,c;
	const double *Mrc;
	double *Mcr;
	double *MT = _WSpace;

	// TRANSPOSE
	for(r=0,Mrc=aM;r<aNR;r++) {
		Mcr = MT + r;
		for(c=0;c<aNC;c++,Mcr+=aNR,Mrc++) {
			*Mcr = *(Mrc);
		}
	}

	// COPY RESULTS
	memcpy(rMT,MT,n*sizeof(double));

	return(0);
}
//_____________________________________________________________________________
/**
 * Print a matrix.
 */
void Mtx::
Print(int aNR,int aNC,const double *aM,int aPrecision)
{
	int r,c,I;

	// HEADER
	char format0[256];  sprintf(format0,"   %%%dd:",6+aPrecision);
	printf("  ");
	for(c=0;c<aNC;c++) printf(format0,c);
	printf("\n");

	// ROWS
	char format1[256];  sprintf(format1,"  %%4.%dle",aPrecision);
	char format2[256];  sprintf(format2,"   %%4.%dle",aPrecision);
	for(r=0;r<aNR;r++) {
		printf("%6d:",r);
		for(c=0;c<aNC;c++) {
			I = ComputeIndex(r,aNC,c);
			if(aM[I]<0.0) {
				printf(format1,aM[I]);
			} else {
				printf(format2,aM[I]);
			}
		}
		printf("\n");
	}
}


//=============================================================================
// INDEX OPERATIONS
//=============================================================================
//_____________________________________________________________________________
/**
 * Starting at aIndex, move through the array aT and find the index of
 * the element whose value is closest to but less than aTime.  It is assumed
 * that the array aT is monotonically increasing and has at least 2 elements.
 *
 * If aTime lies outside the interval of aT, the index of either the first
 * point or second to last point is returned depending on whether aTime
 * is below or above the interval of aT.
 *
 * -1 is if an error is encountered.
 */
int Mtx::
FindIndex(int aIndex,double aTime,int aNT,double *aT)
{
	// ERROR CHECK
	if(aNT<=1) return(-1);
	if(aT==NULL) return(-1);

	// MAKE SURE aIndex IS VALID
	if((aIndex>=aNT)||(aIndex<0)) aIndex=0;

	// CHECK FOR BELOW RANGE
	if(aTime<=aT[0]) return(0);

	// CHECK FOR ABOVE RANGE
	if(aTime>=aT[aNT-1]) return(aNT-2);

	// SEARCH BACKWARDS
	int i;
	if(aT[aIndex]>aTime) {
		for(i=aIndex-1;i>=0;i--) {
			if(aT[i]<=aTime) return(i);
		}

	// SEARCH FORWARDS
	} else {
		for(i=aIndex+1;i<aNT;i++) {
			if(aT[i]>aTime) return(i-1);
		}
	}

	return(-1);
}
//_____________________________________________________________________________
/**
 *	Scan from the beginning of the array, aX, and find the index of the
 * element such that the element's value is less than or equal to aValue and
 * the next element's value is greater than aValue.
 *
 * If no value in the array is greater than aValue, the index of the last
 * element in the array is returned.
 *
 * If no value in the array is less than or equal to aValue, -1 is returned.
 */
int Mtx::
FindIndexLess(int aNX,double *aX,double aValue)
{
	if(aX==NULL) return(-1);

	int i,index=-1;
	for(i=0;i<aNX;i++) {
		if(aX[i]<=aValue) {
			index = i;
		}
		if(aX[i]>aValue) break;
	}

	return(index);
}
//_____________________________________________________________________________
/**
 *	Scan from the end of the array, aX, and find the index of the
 * element such that the element's value is greater than or equal to aValue
 * and such that the next element's value is less than aValue.
 *
 * If no value in the array is less than aValue, the index of the first
 * element in the array is returned.
 *
 * If no value in the array is greater than or equal to aValue, -1 is returned.
 */
int Mtx::
FindIndexGreater(int aNX,double *aX,double aValue)
{
	if(aX==NULL) return(-1);

	int i,index=-1;
	for(i=aNX-1;i>=0;i--) {
		if(aX[i]>=aValue) {
			index = i;
		}
		if(aX[i]<aValue) break;
	}

	return(index);
}
//_____________________________________________________________________________
/**
 *	Compute the index for an element of a three dimensional matrix as though
 * the matrix were laid out in one dimension.
 */
int Mtx::
ComputeIndex(int i2,int n1,int i1)
{
	int i =  i1 + i2*n1;
	return(i);
}
//_____________________________________________________________________________
/**
 *	Compute the index for an element of a three dimensional matrix as though
 * the matrix were laid out in one dimension.
 */
int Mtx::
ComputeIndex(int i3,int n2,int i2,int n1,int i1)
{
	int i =  i1 + i2*n1 + i3*n2*n1;
	return(i);
}

//_____________________________________________________________________________
/**
 * Get elements *,i2,i1 of a three dimensional matrix as an array, where *
 * varies along the 3rd dimension, i2 the 2nd, and i1 the 1st.  The first
 * dimension is the dimension which varies most rapidly when the data
 * is laid out in a one dimensional array, and the second dimension is the
 * one which varies second most rapidly, and so on.
 *
 * For now, it is assumed that parameter a has enough memory allocated to
 * hold the array.
 */
void Mtx::
GetDim3(int n3,int n2,int n1,int i2,int i1,double *m,double *a)
{
	int i3,I;
	for(i3=0;i3<n3;i3++) {
		I = ComputeIndex(i3,n2,i2,n1,i1);
		a[i3] = m[I];
	}
}
//_____________________________________________________________________________
/**
 * Set elements *,i2,i1 of a three dimensional matrix to the values in array a,
 * where * varies along the 3rd dimension, i2 the 2nd, and i1 the 1st.  The
 * first dimension is the dimension which varies most rapidly when the data
 * is laid out in a one dimensional array, and the second dimension is the
 * one which varies second most rapidly, and so on.
 */
void Mtx::
SetDim3(int n3,int n2,int n1,int i2,int i1,double *m,double *a)
{
	int i3,I;
	for(i3=0;i3<n3;i3++) {
		I = ComputeIndex(i3,n2,i2,n1,i1);
		m[I] = a[i3];
	}
}


//=============================================================================
// WORKSPACE MANAGEMENT
//=============================================================================
//_____________________________________________________________________________
/**
 * Ensure that the work space is at least of size aN.
 *
 * If the capacity could not be increased to aN, -1 is returned.  Otherwise,
 * 0 is returned.
 */
int Mtx::
EnsureWorkSpaceCapacity(int aN)
{
	if(aN>_WSpaceSize) {

		_WSpaceSize = aN;

		// DELETE EXISTING ALLOCATION
		if(_WSpace!=NULL) delete[] _WSpace;

		// W
		_WSpace = new double[_WSpaceSize];
		if(_WSpace==NULL) { _WSpaceSize=0;  return(-1); }
	}

	return(0);
}
//_____________________________________________________________________________
/**
 * Ensure that the pointer spaces is at least of size aN.
 *
 * If the capacity could not be increased to aN, -1 is returned.  Otherwise,
 * 0 is returned.
 */
int Mtx::
EnsurePointerSpaceCapacity(int aN)
{
	if(aN>_PSpaceSize) {

		_PSpaceSize = aN;

		// DELETE EXISTING ALLOCATIONS
		if(_P1Space!=NULL) delete[] _P1Space;
		if(_P2Space!=NULL) delete[] _P2Space;

		// P1
		_P1Space = new double*[_PSpaceSize];
		if(_P1Space==NULL) { _PSpaceSize=0;  return(-1); }

		// P2
		_P2Space = new double*[aN];
		if(_P2Space==NULL) { delete[] _P1Space;  _PSpaceSize=0;  return(-1); }
	}

	return(0);
}
//_____________________________________________________________________________
/**
 * Free the work and pointer spaces.
 */
void Mtx::
FreeWorkAndPointerSpaces()
{
	if(_WSpace!=NULL) { delete[] _WSpace;  _WSpace=NULL; }
	if(_P1Space!=NULL) { delete[] _P1Space;  _P1Space=NULL; }
	if(_P2Space!=NULL) { delete[] _P2Space;  _P2Space=NULL; }
	_WSpaceSize = 0;
	_PSpaceSize = 0;
}
