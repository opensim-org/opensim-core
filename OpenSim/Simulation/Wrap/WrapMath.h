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

    /**
     * Compute the intersection between a line (p1->p2) and
     * another line (p3->p4). If the lines do not intersect,
     * this function returns the closest point on each line
     * to the other line.
     *
     * @param p1 first point on first line
     * @param p2 second point on first line
     * @param p3 first point on second line
     * @param p4 second point on second line
     * @param pInt1 point on first line that is closest to second line
     * @param s parameterized distance along first line from p1 to pInt1
     * @param pInt2 point on second line that is closest to first line
     * @param t parameterized distance along second line from p3 to pInt2
     *
     * @return false if lines are parallel, true otherwise
     */
    static bool IntersectLines(const SimTK::Vec3& p1,
                               const SimTK::Vec3& p2,
                               const SimTK::Vec3& p3,
                               const SimTK::Vec3& p4,
                               SimTK::Vec3& pInt1,
                               double& s,
                               SimTK::Vec3& pInt2,
                               double& t);

    /**
     * Compute the intersection of a line segment and a plane
     *
     * @param pt1 first point on line
     * @param pt2 second point on line
     * @param plane normal vector of plane
     * @param d normal distance of plane to origin
     * @param inter intersection point of line and plane
     *
     * @return true if line segment and plane intersect, false otherwise
     */
    static bool IntersectLineSegPlane(const SimTK::Vec3& pt1,
                                      const SimTK::Vec3& pt2,
                                      const SimTK::UnitVec3& plane,
                                      double d,
                                      SimTK::Vec3& inter);

    /**
     * Calculate the point (closestPt) on a line (linePt, line)
     * that is closest to a point (pt). 'line' does not need to
     * be normalized.
     *
     * @param pt the point
     * @param linePt a point on the line
     * @param line defines the line passing through linePt
     * @param closestPt the closest point
     * @param t parameterized distance from linePt along line to closestPt
     */
    static void GetClosestPointOnLineToPoint(const SimTK::Vec3& pt,
                                             const SimTK::Vec3& linePt,
                                             const SimTK::Vec3& line,
                                             SimTK::Vec3& closestPt,
                                             double& t);

    /**
     * Calculate the squared distance between two points (i.e. v = p2-p1; return dot(v, v);)
     *
     * @param  point1 first point
     * @param  point2 second point
     *
     * @return the squared distance between the two points
     */
    static double CalcDistanceSquaredBetweenPoints(const SimTK::Vec3& point1,
                                                   const SimTK::Vec3& point2);

    /**
     * Calculate the squared distance between a point and an infinitely long line
     *
     * @brief CalcDistanceSquaredPointToLine
     * @param point     the point
     * @param linePt    a point on the line
     * @param lineDir   the line's direction
     *
     * @return the shortest (squared) distance between the point and the line
     */
    static double CalcDistanceSquaredPointToLine(const SimTK::Vec3& point,
                                                 const SimTK::Vec3& linePt,
                                                 const SimTK::Vec3& lineDir);

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
    static double NormalizeOrZero(const SimTK::Vec3& aV, SimTK::Vec3& rV);

//=============================================================================
};  // END class WrapMath

/** @endcond **/

}; //namespace
//=============================================================================
//=============================================================================

#endif // __WrapMath_h__

