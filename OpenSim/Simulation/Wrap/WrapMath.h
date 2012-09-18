#ifndef _WrapMath_h_
#define _WrapMath_h_
/* -------------------------------------------------------------------------- *
 *                            OpenSim:  WrapMath.h                            *
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

