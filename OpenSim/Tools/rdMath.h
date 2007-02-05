#ifndef _rdMath_h_
#define _rdMath_h_
// rdMath.h
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


#include "rdTools.h"

// Need these undefined to make this work on linux
#ifdef NAN
#undef NAN
#endif
#ifdef INFINITY
#undef INFINITY
#endif


namespace OpenSim { 

class Line;
class Plane;


//=============================================================================
//=============================================================================
/**
 * This class provides basic math functions and constants.
 */
class RDTOOLS_API rdMath
{
//=============================================================================
// DATA
//=============================================================================
public:
	static const double PI;
	static const double PI_2;
	static const double RTD;
	static const double DTR;
	static const double SMALL;
	static const double ZERO;
	static const double NAN;
	static const double INFINITY;
	static const double MINUS_INFINITY;
	static const double PLUS_INFINITY;

//=============================================================================
// METHODS
//=============================================================================
public:

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
	static int
		ComputeIntersection(const Line *aLine,const Plane *aPlane,
		double rPoint[3]);
	static void
		ComputeNormal(double aP1X,double aP1Y,double aP1Z,
		double aP2X,double aP2Y,double aP2Z,
		double aP3X,double aP3Y,double aP3Z,
		double rNormal[3]);
	static bool
		IntersectLines(double p1[3], double p2[3],
		double p3[3], double p4[3],
		double pInt1[3], double& s,
		double pInt2[3], double& t);
	static bool
		IntersectLineSegPlane(double pt1[3], double pt2[3],
		double plane[3], double d, double inter[3]);
	static void
		ConvertAxisAngleToQuaternion(const double axis[3],
		double angle, double quat[4]);
	static void
		GetClosestPointOnLineToPoint(double pt[3], double linePt[3],
		double line[3], double closestPt[3], double& t);
	static void
		Make3x3DirCosMatrix(double angle, double mat[][3]);
	static void
		ConvertAxisAngleTo4x4DirCosMatrix(const double axis[3], double angle, double mat[][4]);
	static double
		CalcDistanceSquaredBetweenPoints(double point1[], double point2[]);
	static double
		CalcDistanceSquaredPointToLine(double point[], double linePt[], double line[]);
	static void
		RotateMatrixAxisAngle(double matrix[][4], const double axis[3], double angle);
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

