#ifndef _rdMath_h_
#define _rdMath_h_
// rdMath.h
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


#include <math.h>
#include "SimTKcommon.h"
#include "osimCommonDLL.h"
#include "SimTKcommon.h"


namespace OpenSim { 

//=============================================================================
//=============================================================================
/**
 * This class provides basic math functions and constants.
 */
class OSIMCOMMON_API rdMath
{
//=============================================================================
// DATA
//=============================================================================
public:
	static const double PI_2;
	static const double SMALL;
	static const double ZERO;
	static const double MINUS_INFINITY;
	static const double PLUS_INFINITY;

//=============================================================================
// METHODS
//=============================================================================
public:
	//--------------------------------------------------------------------------
	// NAN stuff
	//--------------------------------------------------------------------------
	// TODO: eventually we should stop using our own NAN and use C++'s own NaN or something better
	// Problem with our NAN is that we're using a small number close to zero so some code might
	// incorrectly round it off to zero (e.g. if comparing it to zero using IsEqual)
	// Should use isNAN instead of hcecking rdMath::getNAN() because once we switch to a real NaN value the
	// == check will fail even if it is set to NaN!
	static bool isNAN(double val) { return !(val ==val); }
	static double getNAN() {return SimTK::CNT<SimTK::Real>::getNaN();}

	//--------------------------------------------------------------------------
	// MIN / MAX
	//--------------------------------------------------------------------------
	//template<class T> static inline T Min(T a,T b) { return a <= b ? a : b; }
	//template<class T> static inline T Max(T a,T b) { return a >= b ? a : b; }
	template<class T> static inline T Clamp(T x,T xmin,T xmax) { return x<xmin ? xmin : (x>xmax ? xmax : x); }

	//--------------------------------------------------------------------------
	// ARITHMATIC
	//--------------------------------------------------------------------------
	static double CopySign(double aMag,double aSign);
	static bool IsEqual(double aValue1,double aValue2,double aTol);
	static bool IsZero(double aValue);
	
	//--------------------------------------------------------------------------
	// EXPONENTIAL STEP FUNCTIONS
	//--------------------------------------------------------------------------
	static double SigmaUp(double tau,double to,double t);
	static double SigmaDn(double tau,double to,double t);

	//--------------------------------------------------------------------------
	// CUBIC STEP FUNCTION
	//--------------------------------------------------------------------------
	static double Step(double t, double t0, double t1);

	//--------------------------------------------------------------------------
	// CURVE FITTING
	//--------------------------------------------------------------------------
	static int
		FitParabola(double aX1,double aY1,double aX2,double aY2,
		double aX3,double aY3,double *rC0,double *rC1,double *rC2);
	static double
		Interpolate(double aX1,double aY1,double aX2,double aY2,double aX);

	//--------------------------------------------------------------------------
	// GEOMETRY
	//--------------------------------------------------------------------------
	static void
		ComputeNormal(double aP1X,double aP1Y,double aP1Z,
		double aP2X,double aP2Y,double aP2Z,
		double aP3X,double aP3Y,double aP3Z,
				  SimTK::Vec3& rNormal);
	static bool
		IntersectLines(SimTK::Vec3& p1, SimTK::Vec3& p2,
		SimTK::Vec3& p3, SimTK::Vec3& p4,
		SimTK::Vec3& pInt1, double& s,
		SimTK::Vec3& pInt2, double& t);
	static bool
		IntersectLineSegPlane(SimTK::Vec3& pt1, SimTK::Vec3& pt2,
		SimTK::Vec3& plane, double d, SimTK::Vec3& inter);
	static void
		ConvertAxisAngleToQuaternion(const SimTK::Vec3& axis,
		double angle, double quat[4]);
	static void
		GetClosestPointOnLineToPoint(SimTK::Vec3& pt, SimTK::Vec3& linePt, SimTK::Vec3& line,
									  SimTK::Vec3& closestPt, double& t);
	static void
		Make3x3DirCosMatrix(double angle, double mat[][3]);
	static void
		ConvertAxisAngleTo4x4DirCosMatrix(const SimTK::Vec3& axis, double angle, double mat[][4]);
	static double
		CalcDistanceSquaredBetweenPoints(SimTK::Vec3& point1, SimTK::Vec3& point2);
	static double
		CalcDistanceSquaredPointToLine(SimTK::Vec3& point, SimTK::Vec3& linePt, SimTK::Vec3& line);
	static void
		RotateMatrixAxisAngle(double matrix[][4], const SimTK::Vec3& axis, double angle);
	static void
		ConvertQuaternionToMatrix(const double quat[4], double matrix[][4]);
	static void
		RotateMatrixQuaternion(double matrix[][4], const double quat[4]);
	static void
		RotateMatrixXBodyFixed(double matrix[][4], double angle);
	static void
		RotateMatrixYBodyFixed(double matrix[][4], double angle);
	static void
		RotateMatrixZBodyFixed(double matrix[][4], double angle);


//=============================================================================
};	// END class rdMath

}; //namespace
//=============================================================================
//=============================================================================

#endif // __rdMath_h__

