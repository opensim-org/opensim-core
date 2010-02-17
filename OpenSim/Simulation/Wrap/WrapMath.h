#ifndef _WrapMath_h_
#define _WrapMath_h_
// WrapMath.h
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
#include <OpenSim/Simulation/osimSimulationDLL.h>
#include "SimTKcommon.h"


namespace OpenSim { 

//=============================================================================
//=============================================================================
/**
 * This class provides basic math functions and constants for wrapping surfaces.
 */
class OSIMSIMULATION_API WrapMath
{

//=============================================================================
// METHODS
//=============================================================================
public:
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


//=============================================================================
};	// END class WrapMath

}; //namespace
//=============================================================================
//=============================================================================

#endif // __WrapMath_h__

