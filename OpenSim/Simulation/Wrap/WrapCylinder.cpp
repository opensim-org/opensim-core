/* -------------------------------------------------------------------------- *
 *                         OpenSim:  WrapCylinder.cpp                         *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Peter Loan                                                      *
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

//=============================================================================
// INCLUDES
//=============================================================================
#include "WrapCylinder.h"
#include <OpenSim/Simulation/Model/PathPoint.h>
#include "PathWrap.h"
#include "WrapResult.h"
#include "WrapMath.h"
#include <OpenSim/Common/SimmMacros.h>
#include <OpenSim/Common/Mtx.h>
#include <sstream>

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace OpenSim;
using SimTK::Vec3;

static const char* wrapTypeName = "cylinder";
static Vec3 p0(0.0, 0.0, -1.0);
static Vec3 dn(0.0, 0.0, 1.0);
#define MAX_ITERATIONS    100
#define TANGENCY_THRESHOLD (0.1 * SimTK_DEGREE_TO_RADIAN) // find tangency to within 1 degree

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
* Default constructor.
*/
WrapCylinder::WrapCylinder() :
WrapObject(),
_radius(_radiusProp.getValueDbl()),
_length(_lengthProp.getValueDbl())
{
    setNull();
    setupProperties();
}

//_____________________________________________________________________________
/**
* Destructor.
*/
WrapCylinder::~WrapCylinder()
{
}

//_____________________________________________________________________________
/**
* Copy constructor.
*
* @param aWrapCylinder WrapCylinder to be copied.
*/
WrapCylinder::WrapCylinder(const WrapCylinder& aWrapCylinder) :
WrapObject(aWrapCylinder),
_radius(_radiusProp.getValueDbl()),
_length(_lengthProp.getValueDbl())
{
    setNull();
    setupProperties();
    copyData(aWrapCylinder);
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
* Set the data members of this WrapCylinder to their null values.
*/
void WrapCylinder::setNull()
{
}

//_____________________________________________________________________________
/**
* Connect properties to local pointers.
*/
void WrapCylinder::setupProperties()
{
    // BASE CLASS
    //WrapObject::setupProperties();

    _radiusProp.setName("radius");
    _radiusProp.setValue(-1.0);
    _propertySet.append(&_radiusProp);

    _lengthProp.setName("length");
    _lengthProp.setValue(1.0);
    _propertySet.append(&_lengthProp);
}

//_____________________________________________________________________________
/**
 * Scale the cylinder's dimensions. The base class scales the origin
 * of the cylinder in the body's reference frame.
 *
 * @param aScaleFactors The XYZ scale factors.
 */
void WrapCylinder::scale(const SimTK::Vec3& aScaleFactors)
{
   // Base class, to scale origin in body frame
   WrapObject::scale(aScaleFactors);

    SimTK::Vec3 localScaleVector[3];

   // _pose.x() holds the ellipsoid's X axis expressed in the
   // body's reference frame, _pose.y() holds the Y, and
   // _pose.z() holds the Z. Multiplying these vectors by
   // the scale factor vector gives localScaleVector[]. The magnitudes
   // of the localScaleVectors gives the amount to scale the cylinder
   // in the XYZ dimensions. The wrap cylinder is oriented along
   // the Z axis, so the length is scaled by the Z scale factor,
   // and the radius is scaled by the average of the X and Y scale factors.
    for (int i=0; i<3; i++) {
        localScaleVector[0][i] = _pose.x()[i] * aScaleFactors[i];
        localScaleVector[1][i] = _pose.y()[i] * aScaleFactors[i];
        localScaleVector[2][i] = _pose.z()[i] * aScaleFactors[i];
    }
   _radius *= ((localScaleVector[0].norm() + localScaleVector[1].norm()) * 0.5);
   _length *= localScaleVector[2].norm();
}

//_____________________________________________________________________________
/**
* Perform some set up functions that happen after the
* object has been deserialized or copied.
*
* @param aModel pointer to OpenSim model.
*/
void WrapCylinder::connectToModelAndBody(Model& aModel, PhysicalFrame& aBody)
{
    // Base class
    Super::connectToModelAndBody(aModel, aBody);

    // maybe set a parent pointer, _body = aBody;
    if (_radius < 0.0)
    {
        string errorMessage = "Error: radius for WrapCylinder " + getName() + " was either not specified, or is negative.";
        throw Exception(errorMessage);
    }
/*  Cylinder* cyl = new Cylinder(_radius, _length);
    setGeometryQuadrants(cyl);
*/
}

//_____________________________________________________________________________
/**
* Copy data members from one WrapCylinder to another.
*
* @param aWrapCylinder WrapCylinder to be copied.
*/
void WrapCylinder::copyData(const WrapCylinder& aWrapCylinder)
{
    // BASE CLASS
    WrapObject::copyData(aWrapCylinder);

    _radius = aWrapCylinder._radius;
    _length = aWrapCylinder._length;
}

//_____________________________________________________________________________
/**
 * Get the name of the type of wrap object ("cylinder" in this case)
 *
 * @return A string representing the type of wrap object
 */
const char* WrapCylinder::getWrapTypeName() const
{
    return wrapTypeName;
}

//_____________________________________________________________________________
/**
 * Get a string holding the dimensions definition that SIMM would
 * use to describe this object. This is a rather ugly convenience
 * function for outputting SIMM joint files.
 *
 * @return A string containing the dimensions of the wrap object
 */
string WrapCylinder::getDimensionsString() const
{
    stringstream dimensions;
    dimensions << "radius " << _radius << "\nheight " << _length;

    return dimensions.str();
}

//=============================================================================
// OPERATORS
//=============================================================================
//_____________________________________________________________________________
/**
* Assignment operator.
*
* @return Reference to this object.
*/
WrapCylinder& WrapCylinder::operator=(const WrapCylinder& aWrapCylinder)
{
    // BASE CLASS
    WrapObject::operator=(aWrapCylinder);

    return(*this);
}

//=============================================================================
// WRAPPING
//=============================================================================
//_____________________________________________________________________________
/**
 * Calculate the wrapping of one line segment over the cylinder.
 *
 * @param aPoint1 One end of the line segment
 * @param aPoint2 The other end of the line segment
 * @param aPathWrap An object holding the parameters for this line/cylinder pairing
 * @param aWrapResult The result of the wrapping (tangent points, etc.)
 * @param aFlag A flag for indicating errors, etc.
 * @return The status, as a WrapAction enum
 */
int WrapCylinder::wrapLine(const SimTK::State& s, SimTK::Vec3& aPoint1, SimTK::Vec3& aPoint2,
                                    const PathWrap& aPathWrap, WrapResult& aWrapResult, bool& aFlag) const
{
    double dist, p11_dist, p22_dist, t, dot1, dot2, dot3, dot4, d, sin_theta,
        *r11, *r22, alpha, beta, r_squared = _radius * _radius;
    double dist1, dist2;
    double t12, t00;

    Vec3 pp, vv, uu, r1a, r1b, r2a, r2b, apex, plane_normal, sum_musc, 
        r1am, r1bm, r2am, r2bm, p11, p22, r1p, r2p, axispt, near12, 
        vert1, vert2, mpt, apex1, apex2, l1, l2, near00;

    int i, return_code = wrapped;
    bool r1_inter, r2_inter;
    bool constrained   = (bool) (_wrapSign != 0);
    bool far_side_wrap = false, long_wrap = false;

    // In case you need any variables from the previous wrap, copy them from
    // the PathWrap into the WrapResult, re-normalizing the ones that were
    // un-normalized at the end of the previous wrap calculation.
    const WrapResult& previousWrap = aPathWrap.getPreviousWrap();
    aWrapResult.factor = previousWrap.factor;
    for (i = 0; i < 3; i++)
    {
        aWrapResult.r1[i] = previousWrap.r1[i] * previousWrap.factor;
        aWrapResult.r2[i] = previousWrap.r2[i] * previousWrap.factor;
        aWrapResult.c1[i] = previousWrap.c1[i];
        aWrapResult.sv[i] = previousWrap.sv[i];
    }

    aFlag = false;
    aWrapResult.wrap_path_length = 0.0;
    aWrapResult.wrap_pts.setSize(0);

    // abort if aPoint1 or aPoint2 is inside the cylinder.
    if (WrapMath::CalcDistanceSquaredPointToLine(aPoint1, p0, dn) < r_squared ||
        WrapMath::CalcDistanceSquaredPointToLine(aPoint2, p0, dn) < r_squared)
    {
        return insideRadius;
    }

    // Find the closest intersection between the muscle line segment and the line
    // segment from one end of the cylinder to the other. This intersection is
    // used in several places further down in the code to check for various
    // wrapping conditions.
    SimTK::Vec3 cylStart, cylEnd;
    cylStart[0] = cylEnd[0] = 0.0;
    cylStart[1] = cylEnd[1] = 0.0;
    cylStart[2] = -0.5 * _length;
    cylEnd[2] = 0.5 * _length;

    WrapMath::IntersectLines(aPoint1, aPoint2, cylStart, cylEnd, near12, t12, near00, t00);

    // abort if the cylinder is unconstrained and p1p2 misses the cylinder.
    // Use the return values from the above call to IntersectLines()
    // to perform the check.
    if ( ! constrained)
    {
        if (WrapMath::CalcDistanceSquaredBetweenPoints(near12, near00) < r_squared && t12 > 0.0 && t12 < 1.0)
        {
            return_code = mandatoryWrap;
        }
        else
        {
            return noWrap;
        }
    }

    // find points p11 & p22 on the cylinder axis closest aPoint1 & aPoint2
    WrapMath::GetClosestPointOnLineToPoint(aPoint1, p0, dn, p11, t);
    WrapMath::GetClosestPointOnLineToPoint(aPoint2, p0, dn, p22, t);

    // find preliminary tangent point candidates r1a & r1b
    MAKE_3DVECTOR(p11, aPoint1, vv);

    p11_dist = Mtx::Normalize(3, vv, vv);

    sin_theta = _radius / p11_dist;

    dist = _radius * sin_theta;

    for (i = 0; i < 3; i++)
        pp[i] = p11[i] + dist * vv[i];

    dist = sqrt(r_squared - dist * dist);

    Mtx::CrossProduct(dn, vv, uu);

    for (i = 0; i < 3; i++)
    {
        r1a[i] = pp[i] + dist * uu[i];
        r1b[i] = pp[i] - dist * uu[i];
    }

    // find preliminary tangent point candidates r2a & r2b
    MAKE_3DVECTOR(p22, aPoint2, vv);

    p22_dist = Mtx::Normalize(3, vv, vv);

    sin_theta = _radius / p22_dist;

    dist = _radius * sin_theta;

    for (i = 0; i < 3; i++)
        pp[i] = p22[i] + dist * vv[i];

    dist = sqrt(r_squared - dist * dist);

    Mtx::CrossProduct(dn, vv, uu);

    for (i = 0; i < 3; i++)
    {
        r2a[i] = pp[i] + dist * uu[i];
        r2b[i] = pp[i] - dist * uu[i];
    }

    // choose the best preliminary tangent points r1 & r2 from the 4 candidates.
    if (constrained)
    {
        SimTK::Vec3 sum_r;

        if (DSIGN(aPoint1[_wrapAxis]) == _wrapSign || DSIGN(aPoint2[_wrapAxis]) == _wrapSign)
        {
            // If either muscle point is on the constrained side, then check for intersection
            // of the muscle line and the cylinder. If there is an intersection, then
            // you've found a mandatory wrap. If not, then if one point is not on the constrained
            // side and the closest point on the line is not on the constrained side, you've
            // found a potential wrap. Otherwise, there is no wrap.
            // Use the return values from the previous call to IntersectLines()
            // to perform these checks.
            if (WrapMath::CalcDistanceSquaredBetweenPoints(near12, near00) < r_squared && t12 > 0.0 && t12 < 1.0)
            {
                return_code = mandatoryWrap;
            }
            else
            {
                if (DSIGN(aPoint1[_wrapAxis]) != DSIGN(aPoint2[_wrapAxis]) && DSIGN(near12[_wrapAxis]) != _wrapSign)
                {
                    return_code = wrapped;
                }
                else
                {
                    return noWrap;
                }
            }
        }

        MAKE_3DVECTOR(p11, r1a, r1am);
        MAKE_3DVECTOR(p11, r1b, r1bm);
        MAKE_3DVECTOR(p22, r2a, r2am);
        MAKE_3DVECTOR(p22, r2b, r2bm);

        alpha = Mtx::Angle(r1am, r2bm);
        beta = Mtx::Angle(r1bm, r2am);

        // check to see which of the four tangent points should be chosen by seeing which
        // ones are on the 'active' portion of the cylinder. If r1a and r1b are both on or
        // both off the active portion, then use r2a and r2b to decide.
        if (DSIGN(r1a[_wrapAxis]) == _wrapSign && DSIGN(r1b[_wrapAxis]) == _wrapSign)
        {
            if (DSIGN(r2a[_wrapAxis]) == _wrapSign)
            {
                COPY_1X3VECTOR(r1b, aWrapResult.r1);
                COPY_1X3VECTOR(r2a, aWrapResult.r2);
                if (alpha > beta)
                    far_side_wrap = false;
                else
                    far_side_wrap = true;
            }
            else
            {
                COPY_1X3VECTOR(r1a, aWrapResult.r1);
                COPY_1X3VECTOR(r2b, aWrapResult.r2);
                if (alpha > beta)
                    far_side_wrap = true;
                else
                    far_side_wrap = false;
            }
        }
        else if (DSIGN(r1a[_wrapAxis]) == _wrapSign && DSIGN(r1b[_wrapAxis]) != _wrapSign)
        {
            COPY_1X3VECTOR(r1a, aWrapResult.r1);
            COPY_1X3VECTOR(r2b, aWrapResult.r2);
            if (alpha > beta)
                far_side_wrap = true;
            else
                far_side_wrap = false;
        }
        else if (DSIGN(r1a[_wrapAxis]) != _wrapSign && DSIGN(r1b[_wrapAxis]) == _wrapSign)
        {
            COPY_1X3VECTOR(r1b, aWrapResult.r1);
            COPY_1X3VECTOR(r2a, aWrapResult.r2);
            if (alpha > beta)
                far_side_wrap = false;
            else
                far_side_wrap = true;
        }
        else if (DSIGN(r1a[_wrapAxis]) != _wrapSign && DSIGN(r1b[_wrapAxis]) != _wrapSign)
        {
            if (DSIGN(r2a[_wrapAxis]) == _wrapSign)
            {
                COPY_1X3VECTOR(r1b, aWrapResult.r1);
                COPY_1X3VECTOR(r2a, aWrapResult.r2);
                if (alpha > beta)
                    far_side_wrap = false;
                else
                    far_side_wrap = true;
            }
            else if (DSIGN(r2b[_wrapAxis]) == _wrapSign)
            {
                COPY_1X3VECTOR(r1a, aWrapResult.r1);
                COPY_1X3VECTOR(r2b, aWrapResult.r2);
                if (alpha > beta)
                    far_side_wrap = true;
                else
                    far_side_wrap = false;
            }
            else // none of the four tangent points is on the active portion
            {
                if (alpha > beta)
                {
                    COPY_1X3VECTOR(r1a, aWrapResult.r1);
                    COPY_1X3VECTOR(r2b, aWrapResult.r2);
                    far_side_wrap = true;
                }
                else
                {
                    COPY_1X3VECTOR(r1b, aWrapResult.r1);
                    COPY_1X3VECTOR(r2a, aWrapResult.r2);
                    far_side_wrap = true;
                }
            }
        }
        // determine if the resulting tangent points create a short wrap
        // (less than half the cylinder) or a long wrap.
        for (i = 0; i < 3; i++)
        {
            sum_musc[i] = (aWrapResult.r1[i] - aPoint1[i]) + (aWrapResult.r2[i] - aPoint2[i]);
            sum_r[i] = (aWrapResult.r1[i] - p11[i]) + (aWrapResult.r2[i] - p22[i]);
        }

        if (Mtx::DotProduct(3, sum_r, sum_musc) < 0.0)
            long_wrap = true;
    }
    else
    {
        MAKE_3DVECTOR(p11, r1a, r1am);
        MAKE_3DVECTOR(p11, r1b, r1bm);
        MAKE_3DVECTOR(p22, r2a, r2am);
        MAKE_3DVECTOR(p22, r2b, r2bm);

        Mtx::Normalize(3, r1am, r1am);
        Mtx::Normalize(3, r1bm, r1bm);
        Mtx::Normalize(3, r2am, r2am);
        Mtx::Normalize(3, r2bm, r2bm);

        dot1 = Mtx::DotProduct(3, r1am, r2am);
        dot2 = Mtx::DotProduct(3, r1am, r2bm);
        dot3 = Mtx::DotProduct(3, r1bm, r2am);
        dot4 = Mtx::DotProduct(3, r1bm, r2bm);

        if (dot1 > dot2 && dot1 > dot3 && dot1 > dot4)
        {
            COPY_1X3VECTOR(r1a, aWrapResult.r1);
            COPY_1X3VECTOR(r2a, aWrapResult.r2);
            r11 = &r1b[0];
            r22 = &r2b[0];
        }
        else if (dot2 > dot3 && dot2 > dot4)
        {
            COPY_1X3VECTOR(r1a, aWrapResult.r1);
            COPY_1X3VECTOR(r2b, aWrapResult.r2);
            r11 = &r1b[0];
            r22 = &r2a[0];
        }
        else if (dot3 > dot4)
        {
            COPY_1X3VECTOR(r1b, aWrapResult.r1);
            COPY_1X3VECTOR(r2a, aWrapResult.r2);
            r11 = &r1a[0];
            r22 = &r2b[0];
        }
        else
        {
            COPY_1X3VECTOR(r1b, aWrapResult.r1);
            COPY_1X3VECTOR(r2b, aWrapResult.r2);
            r11 = &r1a[0];
            r22 = &r2a[0];
        }
    }

    // bisect angle between r1 & r2 vectors to find the apex edge of the
    // cylinder:
    MAKE_3DVECTOR(p11, aWrapResult.r1, uu);
    MAKE_3DVECTOR(p22, aWrapResult.r2, vv);

    for (i = 0; i < 3; i++)
        vv[i] = uu[i] + vv[i];

    Mtx::Normalize(3, vv, vv);

    // find the apex point by using a ratio of the lengths of the
    // aPoint1-p11 and aPoint2-p22 segments:
    t = p11_dist / (p11_dist + p22_dist);

    // find point along muscle line according to calculated t value
    for (i = 0; i < 3; i++)
        mpt[i] = aPoint1[i] + t * (aPoint2[i] - aPoint1[i]);

    // find closest point on cylinder axis to mpt
    WrapMath::GetClosestPointOnLineToPoint(mpt, p0, dn, axispt, t);

    // find normal of plane through aPoint1, aPoint2, axispt
    MAKE_3DVECTOR(axispt, aPoint1, l1);
    MAKE_3DVECTOR(axispt, aPoint2, l2);

    Mtx::Normalize(3, l1, l1);
    Mtx::Normalize(3, l2, l2);

    Mtx::CrossProduct(l1, l2, plane_normal);
    Mtx::Normalize(3, plane_normal, plane_normal);

    // cross plane normal and cylinder axis (each way) to
    // get vectors pointing from axispt towards mpt and
    // away from mpt (you can't tell which is which yet).
    Mtx::CrossProduct(dn, plane_normal, vert1);
    Mtx::Normalize(3, vert1, vert1);
    Mtx::CrossProduct(plane_normal, dn, vert2);
    Mtx::Normalize(3, vert2, vert2);

    // now find two potential apex points, one along each vector
    for (i = 0; i < 3; i++)
    {
        apex1[i] = axispt[i] + _radius * vert1[i];
        apex2[i] = axispt[i] + _radius * vert2[i];
    }

    // Now use the distance from these points to mpt to
    // pick the right one.
    dist1 = WrapMath::CalcDistanceSquaredBetweenPoints(mpt, apex1);
    dist2 = WrapMath::CalcDistanceSquaredBetweenPoints(mpt, apex2);
    if (far_side_wrap)
    {
        if (dist1 < dist2)
        {
            for (i = 0; i < 3; i++)
                apex[i] = apex2[i];
        }
        else
        {
            for (i = 0; i < 3; i++)
                apex[i] = apex1[i];
        }
    }
    else
    {
        if (dist1 < dist2)
        {
            for (i = 0; i < 3; i++)
                apex[i] = apex1[i];
        }
        else
        {
            for (i = 0; i < 3; i++)
                apex[i] = apex2[i];
        }
    }

    // determine how far to slide the preliminary r1/r2 along their
    // "edge of tangency" with the cylinder by intersecting the aPoint1-ax
    // line with the plane formed by aPoint1, aPoint2, and apex:
    MAKE_3DVECTOR(apex, aPoint1, uu);
    MAKE_3DVECTOR(apex, aPoint2, vv);

    Mtx::Normalize(3, uu, uu);
    Mtx::Normalize(3, vv, vv);

    Mtx::CrossProduct(uu, vv, plane_normal);
    Mtx::Normalize(3, plane_normal, plane_normal);

    d = - aPoint1[0] * plane_normal[0] - aPoint1[1] * plane_normal[1] - aPoint1[2] * plane_normal[2];

    for (i = 0; i < 3; i++)
    {
        r1a[i] = aWrapResult.r1[i] - 10.0 * dn[i];
        r2a[i] = aWrapResult.r2[i] - 10.0 * dn[i];

        r1b[i] = aWrapResult.r1[i] + 10.0 * dn[i];
        r2b[i] = aWrapResult.r2[i] + 10.0 * dn[i];
    }

    r1_inter = WrapMath::IntersectLineSegPlane(r1a, r1b, plane_normal, d, r1p);
    r2_inter = WrapMath::IntersectLineSegPlane(r2a, r2b, plane_normal, d, r2p);

    if (r1_inter)
    {
        WrapMath::GetClosestPointOnLineToPoint(r1p, p11, p22, r1a, t);

        if (WrapMath::CalcDistanceSquaredBetweenPoints(r1a, p22) < WrapMath::CalcDistanceSquaredBetweenPoints(p11, p22))
            for (i = 0; i < 3; i++)
                aWrapResult.r1[i] = r1p[i];
    }

    if (r2_inter)
    {
        WrapMath::GetClosestPointOnLineToPoint(r2p, p11, p22, r2a, t);

        if (WrapMath::CalcDistanceSquaredBetweenPoints(r2a, p11) < WrapMath::CalcDistanceSquaredBetweenPoints(p22, p11))
            for (i = 0; i < 3; i++)
                aWrapResult.r2[i] = r2p[i];
    }

    // Now that you have r1 and r2, check to see if they are beyond the
    // [display] length of the cylinder. If both are, there should be
    // no wrapping. Since the axis of the cylinder is the Z axis, and
    // the cylinder is centered on Z=0, check the Z components of r1 and r2
    // to see if they are within _length/2.0 of zero.
    if ((aWrapResult.r1[2] < -_length/2.0 || aWrapResult.r1[2] > _length/2.0) && (aWrapResult.r2[2] < -_length/2.0 || aWrapResult.r2[2] > _length/2.0))
        return noWrap;

    // make the path and calculate the muscle length:
    _make_spiral_path(aPoint1, aPoint2, long_wrap, aWrapResult);

    aFlag = true;

    return return_code;

}

//_____________________________________________________________________________
/**
 * Calculate the wrapping points along a spiral path on the cylinder from aPoint1
 * to aPoint2. This function may slide aPoint1 and aPoint2 axially along the
 * cylinder's surface to achieve tangency to within 1 degree at the two points.
 *
 * @param aPoint1 One end of the spiral path
 * @param aPoint2 The other end of the spiral path
 * @param far_side_wrap Boolean indicating if the wrap is the long way around
 * @param aWrapResult The result of the wrapping (tangent points, etc.)
 */
void WrapCylinder::_make_spiral_path(SimTK::Vec3& aPoint1,
                                                 SimTK::Vec3& aPoint2,
                                                 bool far_side_wrap,
                                                 WrapResult& aWrapResult) const
{
    double x, y, t, axial_dist, theta;
    Vec3 r1a, r2a, uu, vv, ax, axial_vec, wrap_pt;
    double sense = far_side_wrap ? -1.0 : 1.0;
    double m[4][4];
    int i, iterations = 0;

restart_spiral_wrap:

    aWrapResult.wrap_pts.setSize(0);

    // determine the axial vector

    WrapMath::GetClosestPointOnLineToPoint(aWrapResult.r1, p0, dn, r1a, t);
    WrapMath::GetClosestPointOnLineToPoint(aWrapResult.r2, p0, dn, r2a, t);

    MAKE_3DVECTOR(r1a, r2a, axial_vec);

    axial_dist = Mtx::Magnitude(3, axial_vec);

    // determine the radial angle
    MAKE_3DVECTOR(r1a, aWrapResult.r1, uu);
    MAKE_3DVECTOR(r2a, aWrapResult.r2, vv);

    for (i = 0; i < 3; i++)
    {
        uu[i] /= _radius;
        vv[i] /= _radius;
    }

    theta = Mtx::Angle(uu,vv);

    if (far_side_wrap)
        theta = 2.0 * SimTK_PI - theta;

    // use Pythagoras to calculate the length of the spiral path (imaging
    // a right triangle wrapping around the surface of a cylinder)
    x = _radius * theta;
    y = axial_dist;

    aWrapResult.wrap_path_length = sqrt(x * x + y * y);

    // build path segments
    Mtx::CrossProduct(uu, vv, ax);
    Mtx::Normalize(3, ax, ax);

    Mtx::CrossProduct(ax, uu, vv);

    m[0][0] = ax[0]; m[0][1] = ax[1]; m[0][2] = ax[2]; m[0][3] = 0.0;
    m[1][0] = uu[0]; m[1][1] = uu[1]; m[1][2] = uu[2]; m[1][3] = 0.0;
    m[2][0] = vv[0]; m[2][1] = vv[1]; m[2][2] = vv[2]; m[2][3] = 0.0;
    m[3][0] = 0.0;   m[3][1] = 0.0;   m[3][2] = 0.0;   m[3][3] = 1.0;

    // Each muscle segment on the surface of the cylinder should be
    // 0.002 meters long. This assumes the model is in meters, of course.
    int numWrapSegments = (int) (aWrapResult.wrap_path_length / 0.002);
    if (numWrapSegments < 1)
        numWrapSegments = 1;

    for (i = 0; i < numWrapSegments; i++)
    {
        double t = (double) i / numWrapSegments;

        _calc_spiral_wrap_point(r1a, axial_vec, m, ax, sense, t, theta, wrap_pt);

        // adjust r1/r2 tangent points if necessary to achieve tangency with
        // the spiral path:
        if (i == 1 && iterations < MAX_ITERATIONS)
        {
            bool did_adjust_r2 = false;
            bool did_adjust_r1 = _adjust_tangent_point(aPoint1, dn, aWrapResult.r1, wrap_pt);

            SimTK::Vec3 temp_wrap_pt;

            _calc_spiral_wrap_point(r1a, axial_vec, m, ax, sense, 1.0 - t, theta, temp_wrap_pt);

            did_adjust_r2 = _adjust_tangent_point(aPoint2, dn, aWrapResult.r2, temp_wrap_pt);

            if (did_adjust_r1 || did_adjust_r2)
            {
                iterations++;
                goto restart_spiral_wrap;
            }
        }

        aWrapResult.wrap_pts.append(wrap_pt);
    }
}

//_____________________________________________________________________________
/**
 * Calculate a new point along a spiral wrapping path.
 *
 * @param r1a An existing point on the spiral path
 * @param axial_vec Vector from r1a parallel to cylinder axis
 * @param m A transform matrix used for the radial component
 * @param axis Axis of the cylinder
 * @param sense The direction of the spiral
 * @param t Parameterized amount of angle for this point
 * @param theta The total angle of the spiral on the cylinder
 * @param wrap_pt The new point on the spiral path
 */
void WrapCylinder::_calc_spiral_wrap_point(const SimTK::Vec3& r1a,
                                                         const SimTK::Vec3& axial_vec,
                                                         double m[4][4],
                                                         const SimTK::Vec3& axis,
                                                         double sense,
                                                         double t,
                                                         double theta,
                                                         SimTK::Vec3& wrap_pt) const
{
    double n[4][4];
    int i, j;

    for (i = 0; i < 4; i++)
        for (j = 0; j < 4; j++)
            n[i][j] = m[i][j];

    WrapMath::RotateMatrixAxisAngle(n, axis, sense * t * theta);

    for (i = 0; i < 3; i++)
    {
        double radial_component = _radius * n[1][i];
        double axial_component = t * axial_vec[i];

        wrap_pt[i] = r1a[i] + radial_component + axial_component;
    }
}

//_____________________________________________________________________________
/**
 * Determine whether the specified tangent point 'r1'
 * needs to be adjusted or not (so that the line from
 * pt1 to r1 is tangent to the cylinder to within 1 degree).
 * If yes, slide it in the appropriate direction and return
 * true, otherwise return false.
 *
 * @param pt1 One point on the line segment being wrapped
 * @param dn Direction vector of cylinder axis
 * @param r1 The tangent point to be adjusted
 * @param w1 A wrapping point (?)
 * @return Whether or not the point was adjusted
 */
bool WrapCylinder::_adjust_tangent_point(SimTK::Vec3& pt1,
                                                      SimTK::Vec3& dn,
                                                      SimTK::Vec3& r1,
                                                      SimTK::Vec3& w1) const
{
    SimTK::Vec3 pr_vec, rw_vec;
    double alpha, omega, t;
    int i;
    bool did_adust = false;

    MAKE_3DVECTOR(pt1, r1, pr_vec);
    MAKE_3DVECTOR(r1, w1, rw_vec);

    Mtx::Normalize(3, pr_vec, pr_vec);
    Mtx::Normalize(3, rw_vec, rw_vec);

    alpha = acos(Mtx::DotProduct(3, pr_vec, dn));
    omega = acos(Mtx::DotProduct(3, rw_vec, dn));

    if (fabs(alpha - omega) > TANGENCY_THRESHOLD)
    {
        double p1w1_t, p1aw1a_t;
        SimTK::Vec3 save, p1a, w1a, p1w1_int, p1aw1a_int;

        WrapMath::GetClosestPointOnLineToPoint(pt1, r1, dn, p1a, t);
        WrapMath::GetClosestPointOnLineToPoint(w1, r1, dn, w1a, t);

        WrapMath::IntersectLines(pt1, w1, p1a, w1a, p1w1_int, p1w1_t, p1aw1a_int, p1aw1a_t);

        for (i = 0; i < 3; i++)
        {
            save[i] = r1[i];
            r1[i] += 1.5 * (p1aw1a_int[i] - r1[i]);
        }

        did_adust = true;
    }

    return did_adust;
}
