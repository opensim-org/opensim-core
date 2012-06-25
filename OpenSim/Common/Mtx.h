#ifndef _Mtx_h_
#define _Mtx_h_
// Mtx.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c)  2005, Stanford University. All rights reserved. 
* Use of the OpenSim software in source form is permitted provided that the following
* conditions are met:
* 	1. The software is used only for non-commercial research and education. It may not
*     be used in relation to any commercial activity.
* 	2. The software is not distributed or redistributed.  Software distribution is allowed 
*     only through https://simtk.org/home/opensim.
* 	3. Use of the OpenSim software or derivatives must be acknowledged in all publications,
*      presentations, or documents describing work in which OpenSim or derivatives are used.
* 	4. Credits to developers may not be removed from executables
*     created from modifications of the source.
* 	5. Modifications of source code must retain the above copyright notice, this list of
*     conditions and the following disclaimer. 
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
*  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
*  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
*  SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
*  TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
*  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR BUSINESS INTERRUPTION) OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
*  WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/* Note: This code was originally developed by Realistic Dynamics Inc. 
 * Author: Frank C. Anderson 
 */


// INCLUDES
#include "osimCommonDLL.h"
#include <math.h>
#include <stdio.h>
#include "SimTKcommon.h"


namespace OpenSim { 

/** @cond **/ // hide from Doxygen

//=============================================================================
//=============================================================================
/**
 * A class for performing vector and matrix operations.  Most all the
 * methods in this class are static.
 */
class OSIMCOMMON_API Mtx
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
	//_____________________________________________________________________________
	/**
	* Find the angle between two vectors:  theta = acos( aV1*aV2/(|aV1|*|aV2|)).
	*
	* @param aV1 Vector 1.
	* @param aV2 Vector 2.
	* @return Angle between two vectors in radians.
	*/
	inline static double Angle(const SimTK::Vec3& aV1, const SimTK::Vec3& aV2){
		SimTK::Vec3 v1(aV1);	v1.normalize();
		SimTK::Vec3 v2(aV2);	v2.normalize();
		return (acos( ~v1 * v2));
	}
	//_____________________________________________________________________________
	/**
	* Normalize a vector.
	*
	* If aV has a magnitude of zero, all elements of rV are set to 0.0.
	* It is permissible for aV and rV to coincide in memory.
	*
    * @param aN     Obsolete -- always pass as a "3".
	* @param aV     Vector to be normalized.
	* @param rV     Result of the normalization.
	* @returns      Magnitude of aV.
	*/
	inline static double Normalize(int aN,const SimTK::Vec3& aV,SimTK::Vec3& rV){
		double mag = aV.norm();
		if (mag ==0) rV = 0.0; else rV = 1/mag * aV;
		return mag;
	}
	/**
	* Compute the magnitude of a vector.
	*
    * @param aN     Obsolete -- always pass as a "3".
	* @param aV     Vector.
	* @returns      Square root of the dot product aV*aV.
	*/
	inline static double Magnitude(int aN,const SimTK::Vec3& aV){
		return aV.norm();
	}
	//_____________________________________________________________________________
	/**
	* Compute the dot product of two vectors.
	*
	* If the arguments are not valid (aV1=aV2=NULL), 0.0 is returned.
	*/
	//_____________________________________________________________________________
	inline static double DotProduct(int aN,const SimTK::Vec3& aV1,const SimTK::Vec3& aV2){
		return ~aV1 * aV2;
	}
	//_____________________________________________________________________________
	/**
	* Compute the cross product of two vectors.
	*
	* If the arguments are not valid (aR=aS=NULL), -1 is returned.
	*/
	inline static void CrossProduct(const SimTK::Vec3& aV1, const SimTK::Vec3& aV2, SimTK::Vec3& aV){
		 aV = aV1 % aV2;
	};
	/**
	* Get a unit vector that is perpendicular to a specified vector.  The unit
	* vector is arbitrary, in the sense that it is one of an infinite number of
	* vectors that are perpendicular to the original specified vector.
	*
	* @param aV Input vector.
	* @param rV Perpendicular unit vector.
	*/
	static void PerpendicularUnitVector(const SimTK::Vec3& aV, SimTK::Vec3& rV)
	{
		SimTK::UnitVec3 unit(aV);
		rV = unit.perp().asVec3();
	}
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
	//static int Assign(int aNR,int aNC,double aScalar,double *rM);
	//static int Assign(int aNR,int aNC,const double *aM,double *rM);
	//static int Add(int aNR,int aNC,const double *aM1,double aScalar,double *aM);
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

/** @endcond **/

}; //namespace
//=============================================================================
//=============================================================================

#endif  // __Mtx_h__
