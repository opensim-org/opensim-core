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
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
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

#include <OpenSim/Simulation/osimSimulationDLL.h>
#include "SimTKcommon/SmallMatrix.h"
#include "SimTKcommon/internal/UnitVec.h"

namespace OpenSim { 

/** @cond **/ // hide from Doxygen

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
        SimTK::UnitVec3& plane, double d, SimTK::Vec3& inter);
    static void
        GetClosestPointOnLineToPoint(SimTK::Vec3& pt, SimTK::Vec3& linePt, SimTK::Vec3& line,
                                      SimTK::Vec3& closestPt, double& t);
    inline static double CalcDistanceSquaredBetweenPoints(
        SimTK::Vec3& point1, SimTK::Vec3& point2) {
        return (point1 - point2).normSqr();
    }
    inline static double CalcDistanceSquaredPointToLine(
            SimTK::Vec3& point, SimTK::Vec3& linePt, SimTK::Vec3& line){
        SimTK::Vec3 pToLinePt = (linePt - point);
        SimTK::Vec3 n = line.normalize();
        return (pToLinePt - (~pToLinePt * n) * n).normSqr();
    };
    /**
     * Normalize a vector or Zero it out if norm < Epsilon.
     *
     * If aV has a magnitude of zero, all elements of rV are set to 0.0.
     * It is permissible for aV and rV to coincide in memory.
     *
     * @param aV     Vector to be normalized.
     * @param rV     Result of the normalization.
     * @returns      Magnitude of aV.
     */
    inline static double NormalizeOrZero(const SimTK::Vec3& aV, SimTK::Vec3& rV) {
        double mag = aV.norm();
        if (mag >= SimTK::Eps)
            rV = aV.scalarMultiply(1.0 / mag);
        else
            rV.setToZero();
        return mag;
    }

//=============================================================================
};  // END class WrapMath

/** @endcond **/

}; //namespace
//=============================================================================
//=============================================================================

#endif // __WrapMath_h__

