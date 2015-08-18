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

/*static*/ thread_local int Mtx::_PSpaceSize = 0;
/*static*/ thread_local int Mtx::_WSpaceSize = 0;
/*static*/ thread_local double** Mtx::_P1Space = NULL;
/*static*/ thread_local double** Mtx::_P2Space = NULL;
/*static*/ thread_local double*  Mtx::_WSpace = NULL;

static const double eps = std::numeric_limits<double>::epsilon();


//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================

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
 * Otherwise, 0 is returned.
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
    Mp = _P1Space;      // POINTER TO BEGINNING OF POINTER1 SPACE
    Mr = _P1Space;      // ROW POINTERS INTO M
    Ip  = _P2Space;     // POINTER TO BEGINNING OF POINTER2 SPACE
    Ir = _P2Space;      // ROW POINTERS INTO aMInv
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
            Mrj=Mp[r];  Mp[r]=Mp[i];    Mp[i]=Mrj;  Mrj=Mp[r]+r;
            for(j=0,Irj=Ip[r],Iij=Ip[i];j<aN;j++) {
                    d= *Irj;  *(Irj++) = *Iij;  *(Iij++) = d; }
        }

        // REDUCE
        for(i=r+1;i<aN;i++) {
            if(std::fabs(*(Mij=Mp[i]+r)) < eps ) continue;
            Mrj=Mp[r]+r;
            d = (*Mij)/(*Mrj);  *(Mij++)=0.0;   Mrj++;
            for(j=r+1;j<aN;j++) *(Mij++) -= *(Mrj++)*d;
            Irj=Ip[r];  Iij=Ip[i];
            for(j=0;j<aN;j++) *(Iij++) -= *(Irj++)*d;
        }
    }

    // NORMALIZE LAST ROW OF M AND rMInv
    if(std::fabs(*(Mrj=Mp[r]+r)) < eps) return(-2);
    d = 1.0 / *Mrj; *Mrj=1.0;
    for(j=0,Irj=Ip[r];j<aN;j++) *(Irj++) *= d;

    // DIAGONALIZE USING ROW OPERATIONS
    for(r=aN-1;r>0;) {
        for(i=r-1;i>=0;i--) {
            if(std::fabs(*(Mij=Mp[i]+r)) < eps) continue;
            d = *Mij;   *Mij=0.0;
            for(j=0,Iij=Ip[i],Irj=Ip[r];j<aN;j++) *(Iij++) -= *(Irj++)*d;
        }
        r--;
        d = 1.0 / *(Mrj=Mp[r]+r);   *Mrj=1.0;
        for(j=0,Irj=Ip[r];j<aN;j++) *(Irj++) *= d;
    }

    return(0);
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
