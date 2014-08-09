/* -------------------------------------------------------------------------- *
 *                           OpenSim:  WrapMath.cpp                           *
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

//=============================================================================
// INCLUDES
//=============================================================================
#include <math.h>
#include "WrapMath.h"
#include <OpenSim/Common/Mtx.h>
#include <OpenSim/Common/SimmMacros.h>
#include "SimTKcommon.h"


using namespace OpenSim;
using namespace std;
using SimTK::Vec3;
using SimTK::Mat33;

//=============================================================================
// EXPORTED STATIC CONSTANTS
//=============================================================================

#define LINE_EPSILON 0.00001



//=============================================================================
// GEOMETRY
//=============================================================================
//_____________________________________________________________________________

//_____________________________________________________________________________
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
 * @return false if lines are parallel, true otherwise
 */
bool WrapMath::
IntersectLines(SimTK::Vec3& p1, SimTK::Vec3& p2, SimTK::Vec3& p3, SimTK::Vec3& p4,
               SimTK::Vec3& pInt1, double& s, SimTK::Vec3& pInt2, double& t)
{
    SimTK::Vec3 cross_prod, vec1, vec2;

    vec1 = p2 - p1;

    double mag1 = Mtx::Normalize(3, vec1, vec1);

    vec2 = p4 - p3;

    double mag2 = Mtx::Normalize(3, vec2, vec2);

    Mtx::CrossProduct(vec1, vec2, cross_prod);

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

    t = CALC_DETERMINANT(mat) / denom;

    pInt2 = p3 + t * (vec2);

    mat[1][0] = vec2[0];
    mat[1][1] = vec2[1];
    mat[1][2] = vec2[2];

    s = CALC_DETERMINANT(mat) / denom;

    pInt1 = p1 + s * (vec1);

    s /= mag1;
    t /= mag2;

    return true;
}

/* Compute the intersection of a line segment and a plane
 * @param pt1 first point on line
 * @param pt2 second point on line
 * @param plane normal vector of plane
 * @param d normal distance of plane to origin
 * @param inter intersection point of line and plane
 * @return true if line segment and plane intersect, false otherwise
 */
bool WrapMath::
IntersectLineSegPlane(SimTK::Vec3& pt1, SimTK::Vec3& pt2,
                      SimTK::Vec3& plane, double d,
                      SimTK::Vec3& inter)
{
    SimTK::Vec3 vec;

    MAKE_3DVECTOR(pt1,pt2,vec);
    double dotprod = Mtx::DotProduct(3, vec,plane);

    if (DABS(dotprod) < LINE_EPSILON)
        return false;

    double t = (-d - plane[0]*pt1[0] - plane[1]*pt1[1] - plane[2]*pt1[2]) / dotprod;

    if ((t < -LINE_EPSILON) || (t > 1.0 + LINE_EPSILON))
        return false;

    inter[0] = pt1[0] + (t * vec[0]);
    inter[1] = pt1[1] + (t * vec[1]);
    inter[2] = pt1[2] + (t * vec[2]);

    return true;
}

/* Convert an axis/angle rotation into a quaternion
 * @param axis the axis of rotation
 * @param angle the angle, in radians
 * @param quat the quaternion
 */
void WrapMath::
ConvertAxisAngleToQuaternion(const SimTK::Vec3& axis, double angle, double quat[4])
{
    quat[0] = axis[0];
    quat[1] = axis[1];
    quat[2] = axis[2];
    quat[3] = 0.0;

    double n = sqrt(quat[0] * quat[0] + quat[1] * quat[1] + quat[2] * quat[2]);

    if (NOT_EQUAL_WITHIN_ERROR(n, 0.0))
    {
        double halfAngle = 0.5 * angle;
        double s = sin(halfAngle) / n;

        quat[0] *= s;
        quat[1] *= s;
        quat[2] *= s;
        quat[3] = cos(halfAngle);
    }
}

/* Calculate the point (closestPt) on a line (linePt, line)
 * that is closest to a point (pt). 'line' does not need to
 * be normalized.
 * @param pt the point
 * @param linePt a point on the line
 * @param line defines the line passing through linePt
 * @param closestPt the closest point
 * @param t parameterized distance from linePt along line to closestPt
 */
void WrapMath::
GetClosestPointOnLineToPoint(SimTK::Vec3& pt, SimTK::Vec3& linePt, SimTK::Vec3& line,
                             SimTK::Vec3& closestPt, double& t)
{
    SimTK::Vec3 v1, v2;

    v1 = pt - linePt;

    v2 = line;
    double mag = Mtx::Normalize(3, v1, v1);
    double mag2 = Mtx::Normalize(3, v2, v2);
    t = Mtx::DotProduct(3, v1, v2) * mag;

    closestPt = linePt + t * v2;
    t = t / mag2;
}

/* Make a 3x3 direction cosine matrix for a
 * rotation about the X axis.
 * @param angle the rotation angle, in radians
 * @param m the 3x3 matrix
 */
void WrapMath::
Make3x3DirCosMatrix(double angle, double mat[][3])
{
    mat[0][0] = 1.0;
    mat[0][1] = 0.0;
    mat[0][2] = 0.0;

    mat[1][0] = 0.0;
    mat[1][1] = cos(angle);
    mat[1][2] = sin(angle);

    mat[2][0] = 0.0;
    mat[2][1] = -mat[1][2];
    mat[2][2] = mat[1][1];
}

/* Make a 4x4 direction cosine matrix from an
 * axis/angle rotation.
 * @param axis the axis of rotation
 * @param angle the angle, in radians
 * @param mat the matrix
 */
void WrapMath::
ConvertAxisAngleTo4x4DirCosMatrix(const SimTK::Vec3& axis, double angle, double mat[][4])
{
    SimTK::Vec3 normAxis;

    Mtx::Identity(4, (double*)mat);
    Mtx::Normalize(3, axis, normAxis);

    double cl = cos(angle);
    double sl = sin(angle);
    double omc = 1.0 - cl;

    // the following matrix is taken from Kane's 'Spacecraft Dynamics,' pp 6-7
    mat[0][0] = cl + normAxis[0]*normAxis[0]*omc;
    mat[1][0] = -normAxis[2]*sl + normAxis[0]*normAxis[1]*omc;
    mat[2][0] = normAxis[1]*sl + normAxis[2]*normAxis[0]*omc;
    mat[0][1] = normAxis[2]*sl + normAxis[0]*normAxis[1]*omc;
    mat[1][1] = cl + normAxis[1]*normAxis[1]*omc;
    mat[2][1] = -normAxis[0]*sl + normAxis[1]*normAxis[2]*omc;
    mat[0][2] = -normAxis[1]*sl + normAxis[2]*normAxis[0]*omc;
    mat[1][2] = normAxis[0]*sl + normAxis[1]*normAxis[2]*omc;
    mat[2][2] = cl + normAxis[2]*normAxis[2]*omc;
}

/* Compute the square of the distance between two
 * points.
 * @param point1 the first point
 * @param point2 the second point
 * @return the square of the distance
 */
double WrapMath::
CalcDistanceSquaredBetweenPoints(SimTK::Vec3& point1, SimTK::Vec3& point2)
{
    SimTK::Vec3 vec = point2 - point1;

    return vec[0]*vec[0]+vec[1]*vec[1]+vec[2]*vec[2];
}

/* Compute the square of the distance between a point
 * and a line.
 * @param point the point
 * @param linePt a point on the line
 * @param line defines the line passing through linePt
 * @return the square of the distance
 */
double WrapMath::
CalcDistanceSquaredPointToLine(SimTK::Vec3& point, SimTK::Vec3& linePt, SimTK::Vec3& line)
{
    double t;
    Vec3 ptemp;

    // find the closest point on line
    GetClosestPointOnLineToPoint(point, linePt, line, ptemp, t);

    return CalcDistanceSquaredBetweenPoints(point, ptemp);
}
/* Rotate a 4x4 transform matrix by 'angle' radians about axis 'axis'.
 * @param matrix The 4x4 transform matrix
 * @param axis The axis about which to rotate
 * @param angle the amount to rotate, in radians
 */
void WrapMath::
RotateMatrixAxisAngle(double matrix[][4], const SimTK::Vec3& axis, double angle)
{
    double quat[4];

    ConvertAxisAngleToQuaternion(axis, angle, quat);
    RotateMatrixQuaternion(matrix, quat);
}

/* Make a 4x4 transform matrix from a quaternion.
 * @param matrix The 4x4 transform matrix
 * @param axis The axis about which to rotate
 * @param angle the amount to rotate, in radians
 */
void WrapMath::
ConvertQuaternionToMatrix(const double quat[4], double matrix[][4])
{
    double Nq = quat[0] * quat[0] + quat[1] * quat[1] + quat[2] * quat[2] + quat[3] * quat[3];
    double s = (Nq > 0.0) ? (2.0 / Nq) : 0.0;

    double xs = quat[0] * s,   ys = quat[1] * s,   zs = quat[2] * s;
    double wx = quat[3] * xs,  wy = quat[3] * ys,  wz = quat[3] * zs;
    double xx = quat[0] * xs,  xy = quat[0] * ys,  xz = quat[0] * zs;
    double yy = quat[1] * ys,  yz = quat[1] * zs,  zz = quat[2] * zs;

    matrix[0][0] = 1.0 - (yy + zz);
    matrix[0][1] = xy + wz;
    matrix[0][2] = xz - wy;
    matrix[1][0] = xy - wz;
    matrix[1][1] = 1.0 - (xx + zz);
    matrix[1][2] = yz + wx;
    matrix[2][0] = xz + wy;
    matrix[2][1] = yz - wx;
    matrix[2][2] = 1.0 - (xx + yy);

    matrix[0][3] = matrix[1][3] = matrix[2][3] = matrix[3][0] = matrix[3][1] = matrix[3][2] = 0.0;
    matrix[3][3] = 1.0;
}

/* Rotate a 4x4 transform matrix by a quaternion.
 * @param matrix The 4x4 transform matrix
 * @param quat The quaternion
 */
void WrapMath::
RotateMatrixQuaternion(double matrix[][4], const double quat[4])
{
    // append a quaternion rotation to a matrix
    double n[4][4];

    ConvertQuaternionToMatrix(quat, n);

    Mtx::Multiply(4, 4, 4, (double*)matrix, (double*)n, (double*)matrix);
}
