/* -------------------------------------------------------------------------- *
 *                           OpenSim:  WrapMath.cpp                           *
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

//=============================================================================
// INCLUDES
//=============================================================================
#include <math.h>
#include "WrapMath.h"
#include <OpenSim/Common/SimmMacros.h>


using namespace OpenSim;
using namespace std;
using SimTK::Vec3;
using SimTK::Mat33;

//=============================================================================
// EXPORTED STATIC CONSTANTS
//=============================================================================

#define LINE_EPSILON 0.00001


bool WrapMath::IntersectLines(const SimTK::Vec3& p1,
                              const SimTK::Vec3& p2,
                              const SimTK::Vec3& p3,
                              const SimTK::Vec3& p4,
                              SimTK::Vec3& pInt1,
                              double& s,
                              SimTK::Vec3& pInt2,
                              double& t)
{
    SimTK::Vec3 vec1 = p2 - p1;

    double mag1 = WrapMath::NormalizeOrZero(vec1, vec1);

    SimTK::Vec3 vec2 = p4 - p3;

    double mag2 = WrapMath::NormalizeOrZero(vec2, vec2);

    SimTK::Vec3 cross_prod = vec1 % vec2;

    double denom = cross_prod.normSqr();

    if (EQUAL_WITHIN_ERROR(denom,0.0)) {
        s = t = SimTK::NaN;
        return false;
    }

    Mat33 mat;

    mat[0][0] = p3[0] - p1[0];
    mat[0][1] = p3[1] - p1[1];
    mat[0][2] = p3[2] - p1[2];
    mat[1][0] = vec1[0];
    mat[1][1] = vec1[1];
    mat[1][2] = vec1[2];
    mat[2][0] = cross_prod[0];
    mat[2][1] = cross_prod[1];
    mat[2][2] = cross_prod[2];

    t = det(mat) / denom;

    pInt2 = p3 + t * (vec2);

    mat[1][0] = vec2[0];
    mat[1][1] = vec2[1];
    mat[1][2] = vec2[2];

    s = det(mat) / denom;

    pInt1 = p1 + s * (vec1);

    s /= mag1;
    t /= mag2;

    return true;
}


bool WrapMath::IntersectLineSegPlane(const SimTK::Vec3& pt1,
                                     const SimTK::Vec3& pt2,
                                     const SimTK::UnitVec3& plane,
                                     double d,
                                     SimTK::Vec3& inter)
{
    SimTK::Vec3 vec = pt2 - pt1;

    double dotprod = (~vec*plane);

    if (DABS(dotprod) < LINE_EPSILON)
        return false;

    double t = (-d - plane[0]*pt1[0] - plane[1]*pt1[1] - plane[2]*pt1[2]) / dotprod;

    if ((t < -LINE_EPSILON) || (t > 1.0 + LINE_EPSILON))
        return false;

    inter = pt1 + (t * vec);
    return true;
}


void WrapMath::GetClosestPointOnLineToPoint(const SimTK::Vec3& pt,
                                            const SimTK::Vec3& linePt,
                                            const SimTK::Vec3& line,
                                            SimTK::Vec3& closestPt,
                                            double& t)
{
    SimTK::Vec3 v1, v2;

    v1 = pt - linePt;

    v2 = line;
    double mag = WrapMath::NormalizeOrZero(v1, v1);
    double mag2 = WrapMath::NormalizeOrZero(v2, v2);
    t = (~v1*v2) * mag;

    closestPt = linePt + t * v2;
    t = t / mag2;
}

double WrapMath::CalcDistanceSquaredBetweenPoints(const SimTK::Vec3& point1,
                                                  const SimTK::Vec3& point2)
{
    return (point1 - point2).normSqr();
}

double WrapMath::CalcDistanceSquaredPointToLine(const SimTK::Vec3& point,
                                                const SimTK::Vec3& linePt,
                                                const SimTK::Vec3& lineDir)
{
    SimTK::Vec3 pToLinePt = (linePt - point);
    SimTK::Vec3 n = lineDir.normalize();
    return (pToLinePt - (~pToLinePt * n) * n).normSqr();
}

double WrapMath::NormalizeOrZero(const SimTK::Vec3& aV, SimTK::Vec3& rV)
{
    double mag = aV.norm();
    if (mag >= SimTK::Eps)
        rV = aV.scalarMultiply(1.0 / mag);
    else
        rV.setToZero();
    return mag;
}
