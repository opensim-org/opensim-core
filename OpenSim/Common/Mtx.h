#ifndef _Mtx_h_
#define _Mtx_h_
// Mtx.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c) 2005, Stanford University. All rights reserved. 
* Redistribution and use in source and binary forms, with or without 
* modification, are permitted provided that the following conditions
* are met: 
*  - Redistributions of source code must retain the above copyright 
*    notice, this list of conditions and the following disclaimer. 
*  - Redistributions in binary form must reproduce the above copyright 
*    notice, this list of conditions and the following disclaimer in the 
*    documentation and/or other materials provided with the distribution. 
*  - Neither the name of the Stanford University nor the names of its 
*    contributors may be used to endorse or promote products derived 
*    from this software without specific prior written permission. 
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, 
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN 
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
* POSSIBILITY OF SUCH DAMAGE. 
*/

/* Note: This code was originally developed by Realistic Dynamics Inc. 
 * Author: Frank C. Anderson 
 */


// INCLUDES
#include "rdTools.h"


//=============================================================================
//=============================================================================
/**
 * A class for performing vector and matrix operations.  Most all the
 * methods in this class are static.
 */
namespace OpenSim { 

class RDTOOLS_API Mtx
{
//=============================================================================
// DATA
//=============================================================================
private:
	static int _PSpaceSize;
	static int _WSpaceSize;
	static double **_P1Space;
	static double **_P2Space;
	static double *_WSpace;

//=============================================================================
// METHODS
//=============================================================================
public:
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
	//Mtx();
	//virtual ~Mtx();

	//--------------------------------------------------------------------------
	// VECTOR
	//--------------------------------------------------------------------------
	static double Angle(const double *aV1,const double *aV2);
	static double Normalize(int aN,const double aV[],double rV[]);
	static double Magnitude(int aN,const double aV[]);
	static double DotProduct(int aN,const double aV1[],const double aV2[]);
	static int CrossProduct(double *aV1,double *aV2,double *aV);
	static void
	 Interpolate(int aN,double aT1,double *aY1,double aT2,double *aY2,
	 double t,double *aY);
	static double
	 Interpolate(double aT1,double aY1,double aT2,double aY2,
	 double t);

	//--------------------------------------------------------------------------
	// TRANSLATION AND ROTATION
	//--------------------------------------------------------------------------
	static void
		Translate(double aX,double aY,double aZ,const double aP[3],double rP[3]);
	static void
		Rotate(int aXYZ,double aRadians,const double aP[3],double rP[3]);
	static void
		Rotate(const double aAxis[3],double aRadians,const double aP[3],
		double rP[3]);
	static void
		RotateDeg(int aXYZ,double aDegrees,const double aP[3],double rP[3]);
	static void
		RotateDeg(const double aAxis[3],double aDegrees,const double aP[3],
		double rP[3]);

	//--------------------------------------------------------------------------
	// MATRIX
	//--------------------------------------------------------------------------
	static int Identity(int aNR,double *rI);
	static int Assign(int aNR,int aNC,double aScalar,double *rM);
	static int Assign(int aNR,int aNC,const double *aM,double *rM);
	static int Add(int aNR,int aNC,const double *aM1,double aScalar,double *aM);
	static int Add(int aNR,int aNC,const double *aM1,const double *aM2,double *aM);
	static int Subtract(int aNR,int aNC,const double *aM1,const double *aM2,
		double *aM);
	static int Multiply(int aNR,int aNC,const double *aM,double aScalar,double *rM);
	static int Multiply(int aNR1,int aNCR,int aNC2,const double *aM1,
		const double *aM2,double *aM);
	static int Invert(int aN,const double *aM,double *aMInv);
	static int Transpose(int aNR,int aNC,const double *aM,double *aMT);
	static void Print(int aNR,int aNC,const double *aM,int aPrecision=8);

	//--------------------------------------------------------------------------
	// INDEX OPERATIONS
	//--------------------------------------------------------------------------
	static int FindIndex(int aStartIndex,double aTime,int aNT,double *aT);
	static int FindIndexLess(int aNX,double *aX,double aValue);
	static int FindIndexGreater(int aNX,double *aX,double aValue);
	static int ComputeIndex(int i2,int n1,int i1);
	static int ComputeIndex(int i3,int n2,int i2,int n1,int i1);
	static void GetDim3(int n3,int n2,int n1,int i2,int i1,double *m,double *a);
	static void SetDim3(int n3,int n2,int n1,int i2,int i1,double *m,double *a);

	//--------------------------------------------------------------------------
	// WORKSPACE MANAGEMENT
	//--------------------------------------------------------------------------
	static int EnsureWorkSpaceCapacity(int aN);
	static int EnsurePointerSpaceCapacity(int aN);
	static void FreeWorkAndPointerSpaces();

//=============================================================================
};	// END class Mtx

}; //namespace
//=============================================================================
//=============================================================================

#endif  // __Mtx_h__
