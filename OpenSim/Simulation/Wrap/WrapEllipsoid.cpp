/* -------------------------------------------------------------------------- *
 *                        OpenSim:  WrapEllipsoid.cpp                         *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
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
#include "WrapEllipsoid.h"
#include "PathWrap.h"
#include "WrapResult.h"
#include "WrapMath.h"
#include <OpenSim/Common/SimmMacros.h>
#include <OpenSim/Common/ModelDisplayHints.h>
#include <OpenSim/Common/ScaleSet.h>

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace OpenSim;
using SimTK::Vec3;

static const char* wrapTypeName = "ellipsoid";
#define ELLIPSOID_TOLERANCE_1 1e-4     // tolerance for pt_to_ellipsoid() special case detection
#define ELLIPSOID_TINY        0.00000001
#define MU_BLEND_MIN          0.7073   // 100% fan (must be greater than cos(45)!)
#define MU_BLEND_MAX          0.9      // 100% Frans
#define NUM_FAN_SAMPLES       300      // IMPORTANT: larger numbers produce less jitter
#define NUM_DISPLAY_SAMPLES   30
#define N_STEPS               16
#define SV_BOUNDARY_BLEND     0.3

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
* Default constructor.
*/
WrapEllipsoid::WrapEllipsoid()
{
    constructProperties();
}

//_____________________________________________________________________________
/**
* Destructor.
*/
WrapEllipsoid::~WrapEllipsoid()
{
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
* Connect properties to local pointers.
*/
void WrapEllipsoid::constructProperties()
{
    // BASE CLASS
    //WrapObject::setupProperties();

    SimTK::Vec3 defaultDimensions = {0.05, 0.05, 0.05};
    constructProperty_dimensions(defaultDimensions);
}

//_____________________________________________________________________________
/**
* Perform some set up functions that happen after the
* object has been deserialized or copied.
*
* @param aModel
*/
void WrapEllipsoid::extendFinalizeFromProperties()
{
    // Base class
    WrapObject::extendFinalizeFromProperties();

    if (get_dimensions()[0] <= 0.0 || get_dimensions()[1] <= 0.0 || get_dimensions()[2] <= 0.0)
    {
        string errorMessage = "Error: Dimensions the WrapEllipsoid radii cannot be less than or equal to 0.";
        throw Exception(errorMessage);
    }
/*
    Ellipsoid* ellipsoid = new Ellipsoid();
    ellipsoid->setEllipsoidParams(_dimensions[0], _dimensions[1], _dimensions[2]);
    setGeometryQuadrants(ellipsoid);
*/
}

void WrapEllipsoid::extendScale(const SimTK::State& s, const ScaleSet& scaleSet)
{
    Super::extendScale(s, scaleSet);

    // Get scale factors (if an entry for the Frame's base Body exists).
    const Vec3& scaleFactors = getScaleFactors(scaleSet, getFrame());
    if (scaleFactors == ModelComponent::InvalidScaleFactors)
        return;

    // _pose.x() holds the ellipsoid's X-axis expressed in the body's reference
    // frame. The elementwise product of this vector and the scaleFactors vector
    // gives the amount that the ellipsoid must be scaled in the X dimension.
    // Similar for the Y and Z dimensions.
    Vec3 localScaleVector[3];

    localScaleVector[0] = _pose.x().elementwiseMultiply(scaleFactors);
    localScaleVector[1] = _pose.y().elementwiseMultiply(scaleFactors);
    localScaleVector[2] = _pose.z().elementwiseMultiply(scaleFactors);

    SimTK::Vec3 previousDimensions(get_dimensions());
    for (int i = 0; i < 3; ++i)
        previousDimensions[i] *= localScaleVector[i].norm();
    set_dimensions(previousDimensions);
}

//_____________________________________________________________________________
/**
 * Get the name of the type of wrap object ("ellipsoid" in this case)
 *
 * @return A string representing the type of wrap object
 */
const char* WrapEllipsoid::getWrapTypeName() const
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
string WrapEllipsoid::getDimensionsString() const
{
    stringstream dimensions;
    dimensions << "radius " << get_dimensions()[0] << " " << get_dimensions()[1] << " " << get_dimensions()[2];

    return dimensions.str();
}

//_____________________________________________________________________________
/**
 * Get the radii of the ellipsoid.
 *
 * @return A Vec3 containing the principle radii along the three axes
 */
SimTK::Vec3 WrapEllipsoid::getRadii() const
{
    return get_dimensions();
}

//=============================================================================
// WRAPPING
//=============================================================================
//_____________________________________________________________________________
/**
 * Calculate the wrapping of one line segment over the ellipsoid.
 *
 * @param aPoint1 One end of the line segment
 * @param aPoint2 The other end of the line segment
 * @param aPathWrap An object holding the parameters for this line/ellipsoid pairing
 * @param aWrapResult The result of the wrapping (tangent points, etc.)
 * @param aFlag A flag for indicating errors, etc.
 * @return The status, as a WrapAction enum
 */
int WrapEllipsoid::wrapLine(const SimTK::State& s, SimTK::Vec3& aPoint1, SimTK::Vec3& aPoint2,
                                     const PathWrap& aPathWrap, WrapResult& aWrapResult, bool& aFlag) const
{
    int i, j, bestMu;
    SimTK::Vec3 p1, p2, m, a, p1p2, p1m, p2m, f1, f2, p1c1, r1r2, vs, t, mu;
    double ppm, aa, bb, cc, disc, l1, l2,
        p1e, p2e, vs4, dist, fanWeight = -SimTK::Infinity;
    double t_sv[3][3], t_c1[3][3];
    bool far_side_wrap = false;
   static SimTK::Vec3 origin(0,0,0);

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

    aFlag = true;
    aWrapResult.wrap_pts.setSize(0);

    // This algorithm works best if the coordinates (aPoint1, aPoint2,
    // origin, _dimensions) are all somewhat close to 1.0. So use
    // the ellipsoid dimensions to calculate a multiplication factor that
    // will be applied to all of the coordinates. You want to use just
    // the ellipsoid dimensions because they do not change from one call to the
    // next. You don't want the factor to change because the algorithm uses
    // some vectors (r1, r2, c1) from the previous call.
    aWrapResult.factor = 3.0 / get_dimensions().sum();

    for (i = 0; i < 3; i++)
    {
        p1[i] = aPoint1[i] * aWrapResult.factor;
        p2[i] = aPoint2[i] * aWrapResult.factor;
        m[i]  = origin[i] * aWrapResult.factor;
        a[i]  = get_dimensions()[i] * aWrapResult.factor;
    }

    p1e = -1.0;
    p2e = -1.0;

    for (i = 0; i < 3;i++)
    {
        p1e += SQR((p1[i] - m[i]) / a[i]);
        p2e += SQR((p2[i] - m[i]) / a[i]);
    }

    // check if p1 and p2 are inside the ellipsoid
    if (p1e < -0.0001 || p2e < -0.0001)
    {
        // p1 or p2 is inside the ellipsoid
        aFlag = false;
        aWrapResult.wrap_path_length = 0.0;

        // transform back to starting coordinate system
        for (i = 0; i < 3; i++)
        {
            aWrapResult.r1[i] /= aWrapResult.factor;
            aWrapResult.r2[i] /= aWrapResult.factor;
        }

        return insideRadius;
    }

    p1p2 = p1 - p2;
    p1m = p1 - m;
    WrapMath::NormalizeOrZero(p1m, p1m);
    p2m = p2 - m;
    WrapMath::NormalizeOrZero(p2m, p2m);

    ppm = (~p1m*p2m) - 1.0;   // angle between p1->m and p2->m: -2.0 to 0.0

    if (fabs(ppm) < 0.0001)
    {
        // vector p1m and p2m are collinear
        aFlag = false;
        aWrapResult.wrap_path_length = 0.0;

        // transform back to starting coordinate system
        for (i = 0; i < 3; i++)
        {
            aWrapResult.r1[i] /= aWrapResult.factor;
            aWrapResult.r2[i] /= aWrapResult.factor;
        }

        return noWrap;
    }

    // check if the line through p1 and p2 intersects the ellipsoid
    for (i = 0; i < 3;i++)
    {
        f1[i] = p1p2[i] / a[i];
        f2[i] = (p2[i] - m[i]) / a[i];
    }
    aa = (~f1*f1);
    bb = 2.0 * (~f1*f2);
    cc = (~f2*f2) - 1.0;
    disc = SQR(bb) - 4.0 * aa * cc;

    if (disc < 0.0)
    {
        // no intersection
        aFlag = false;
        aWrapResult.wrap_path_length = 0.0;

        // transform back to starting coordinate system
        for (i = 0; i < 3; i++)
        {
            aWrapResult.r1[i] /= aWrapResult.factor;
            aWrapResult.r2[i] /= aWrapResult.factor;
        }

        return noWrap;
    }

    l1 = (-bb + sqrt(disc)) / (2.0 * aa);
    l2 = (-bb - sqrt(disc)) / (2.0 * aa);

    if ( ! (0.0 < l1 && l1 < 1.0) || ! (0.0 < l2 && l2 < 1.0) )
    {
        // no intersection
        aFlag = false;
        aWrapResult.wrap_path_length = 0.0;

        // transform back to starting coordinate system
        for (i = 0; i < 3; i++)
        {
            aWrapResult.r1[i] /= aWrapResult.factor;
            aWrapResult.r2[i] /= aWrapResult.factor;
        }

        return noWrap;
    }

    // r1 & r2: intersection points of p1->p2 with the ellipsoid
    for (i = 0; i < 3; i++)
    {
        aWrapResult.r1[i] = p2[i] + l1 * p1p2[i];
        aWrapResult.r2[i] = p2[i] + l2 * p1p2[i];
    }

    // ==== COMPUTE WRAPPING PLANE (begin) ====

    r1r2 = aWrapResult.r2 - aWrapResult.r1;

    // (1) Frans technique: choose the most parallel coordinate axis, then set
    // 'sv' to the point along the muscle line that crosses the plane where
    // that major axis equals zero.  This takes advantage of the special-case
    // handling in pt_to_ellipsoid() that reduces the 3d point-to-ellipsoid
    // problem to a 2d point-to-ellipse problem.  The 2d case returns a nice
    // c1 in situations where the "fan" has a sharp discontinuity.
    WrapMath::NormalizeOrZero(p1p2, mu);

    for (i = 0; i < 3; i++)
    {
        mu[i] = fabs(mu[i]);

        t[i] = (m[i] - aWrapResult.r1[i]) / r1r2[i];

        for (j = 0; j < 3; j++)
            t_sv[i][j] = aWrapResult.r1[j] + t[i] * r1r2[j];

        findClosestPoint(a[0], a[1], a[2], t_sv[i][0], t_sv[i][1], t_sv[i][2], &t_c1[i][0], &t_c1[i][1], &t_c1[i][2], i);
    }

    // pick most parallel major axis
    for (bestMu = 0, i = 1; i < 3; i++)
        if (mu[i] > mu[bestMu])
            bestMu = i;

    if (aPathWrap.getMethod() == PathWrap::hybrid ||
         aPathWrap.getMethod() == PathWrap::axial)
    {
        if (aPathWrap.getMethod() == PathWrap::hybrid && mu[bestMu] > MU_BLEND_MIN)
        {
            // If Frans' technique produces an sv that is not within the r1->r2
            // line segment, then that means that sv will be outside the ellipsoid.
            // This can create an sv->c1 vector that points roughly 180-degrees
            // opposite to the fan solution's sv->c1 vector.  This creates problems
            // when interpolating between the Frans and fan solutions because the
            // interpolated c1 can become collinear to the muscle line during
            // interpolation.  Therefore we detect Frans-solution sv points near
            // the ends of r1->r2 here, and fade out the Frans result for them.

            double s = 1.0;

            if (t[bestMu] < 0.0 || t[bestMu] > 1.0)
                s = 0.0;

            else if (t[bestMu] < SV_BOUNDARY_BLEND)
                s = t[bestMu] / SV_BOUNDARY_BLEND;

            else if (t[bestMu] > (1.0 - SV_BOUNDARY_BLEND))
                s = (1.0 - t[bestMu]) / SV_BOUNDARY_BLEND;

            if (s < 1.0)
                mu[bestMu] = MU_BLEND_MIN + s * (mu[bestMu] - MU_BLEND_MIN);
        }

        if (aPathWrap.getMethod() == PathWrap::axial || mu[bestMu] > MU_BLEND_MIN)
        {
            // if the Frans solution produced a strong result, copy it into
            // sv and c1.
            for (i = 0; i < 3; i++)
            {
                aWrapResult.c1[i] = t_c1[bestMu][i];
                aWrapResult.sv[i] = t_sv[bestMu][i];
            }
        }

        if (aPathWrap.getMethod() == PathWrap::hybrid && mu[bestMu] < MU_BLEND_MAX)
        {
            // (2) Fan technique: sample the fan at fixed intervals and average the
            // fan "blade" vectors together to determine c1.  This only works when
            // the fan is smoothly continuous.  The sharper the discontinuity, the
            // more jumpy c1 becomes.
            SimTK::Vec3 v_sum(0,0,0);

            for (i = 0; i < 3; i++)
                t_sv[2][i] = aWrapResult.r1[i] + 0.5 * r1r2[i];

            for (i = 1; i < NUM_FAN_SAMPLES - 1; i++)
            {
                SimTK::Vec3 v;
                double tt = (double) i / NUM_FAN_SAMPLES;

                for (j = 0; j < 3; j++)
                    t_sv[0][j] = aWrapResult.r1[j] + tt * r1r2[j];

                findClosestPoint(a[0], a[1], a[2], t_sv[0][0], t_sv[0][1], t_sv[0][2], &t_c1[0][0], &t_c1[0][1], &t_c1[0][2]);

                for (int k=0; k<3; k++) 
                    v[k] = t_c1[0][k] - t_sv[0][k];

                WrapMath::NormalizeOrZero(v, v);

                // add sv->c1 "fan blade" vector to the running total
                for (j = 0; j < 3; j++)
                    v_sum[j] += v[j];

            }
            // use vector sum to determine c1
            WrapMath::NormalizeOrZero(v_sum, v_sum);

            for (i = 0; i < 3; i++)
                t_c1[0][i] = t_sv[2][i] + v_sum[i];

            if (mu[bestMu] <= MU_BLEND_MIN)
            {
                findClosestPoint(a[0], a[1], a[2], t_c1[0][0], t_c1[0][1], t_c1[0][2], &aWrapResult.c1[0], &aWrapResult.c1[1], &aWrapResult.c1[2]);

                for (i = 0; i < 3; i++)
                    aWrapResult.sv[i] = t_sv[2][i];

                fanWeight = 1.0;
            }
            else
            {
                double tt = (mu[bestMu] - MU_BLEND_MIN) / (MU_BLEND_MAX - MU_BLEND_MIN);

                double oneMinusT = 1.0 - tt;

                findClosestPoint(a[0], a[1], a[2], t_c1[0][0], t_c1[0][1], t_c1[0][2], &t_c1[1][0], &t_c1[1][1], &t_c1[1][2]);

                for (i = 0; i < 3; i++)
                {
                    t_c1[2][i] = tt * aWrapResult.c1[i] + oneMinusT * t_c1[1][i];

                    aWrapResult.sv[i] = tt * aWrapResult.sv[i] + oneMinusT * t_sv[2][i];
                }
                findClosestPoint(a[0], a[1], a[2], t_c1[2][0], t_c1[2][1], t_c1[2][2], &aWrapResult.c1[0], &aWrapResult.c1[1], &aWrapResult.c1[2]);

                fanWeight = oneMinusT;
            }
        }
    }
    else // method == midpoint
    {
        for (i = 0; i < 3; i++)
            aWrapResult.sv[i] = aWrapResult.r1[i] + 0.5 * (aWrapResult.r2[i] - aWrapResult.r1[i]);

        findClosestPoint(a[0], a[1], a[2], aWrapResult.sv[0], aWrapResult.sv[1], aWrapResult.sv[2], &aWrapResult.c1[0], &aWrapResult.c1[1], &aWrapResult.c1[2]);
    }

    // ==== COMPUTE WRAPPING PLANE (end) ====

    // The old way of initializing r1 used the intersection point
    // of p1p2 and the ellipsoid. This caused the muscle path to
    // "jump" to the other side of the ellipsoid as sv[] came near
    // a plane of the ellipsoid. It jumped to the other side while
    // c1[] was still on the first side. The new way of initializing
    // r1 sets it to c1 so that it will stay on c1's side of the
    // ellipsoid.
    {
        bool use_c1_to_find_tangent_pts = true;

        if (aPathWrap.getMethod() == PathWrap::axial)
            use_c1_to_find_tangent_pts = (bool) (t[bestMu] > 0.0 && t[bestMu] < 1.0);

        if (use_c1_to_find_tangent_pts)
            for (i = 0; i < 3; i++)
                aWrapResult.r1[i] = aWrapResult.r2[i] = aWrapResult.c1[i];
    }

    // if wrapping is constrained to one half of the ellipsoid,
    // check to see if we need to flip c1 to the active side of
    // the ellipsoid.
    if (_wrapSign != 0)
    {
        dist = aWrapResult.c1[_wrapAxis] - m[_wrapAxis];

        if (DSIGN(dist) != _wrapSign)
        {
            SimTK::Vec3 orig_c1=aWrapResult.c1;


            aWrapResult.c1[_wrapAxis] = - aWrapResult.c1[_wrapAxis];

            aWrapResult.r1 = aWrapResult.r2 = aWrapResult.c1;

            if (EQUAL_WITHIN_ERROR(fanWeight, -SimTK::Infinity))
                fanWeight = 1.0 - (mu[bestMu] - MU_BLEND_MIN) / (MU_BLEND_MAX - MU_BLEND_MIN);

            if (fanWeight > 1.0)
                fanWeight = 1.0;

            if (fanWeight > 0.0)
            {
                SimTK::Vec3 tc1; 
                double bisection = (orig_c1[_wrapAxis] + aWrapResult.c1[_wrapAxis]) / 2.0;

                aWrapResult.c1[_wrapAxis] = aWrapResult.c1[_wrapAxis] + fanWeight * (bisection - aWrapResult.c1[_wrapAxis]);

                tc1 = aWrapResult.c1;

                findClosestPoint(a[0], a[1], a[2], tc1[0], tc1[1], tc1[2], &aWrapResult.c1[0], &aWrapResult.c1[1], &aWrapResult.c1[2]);
            }
        }
    }

    // use p1, p2, and c1 to create parameters for the wrapping plane
    p1c1 = p1 - aWrapResult.c1;
    vs = p1p2 % p1c1;
    WrapMath::NormalizeOrZero(vs, vs);

    vs4 = - (~vs*aWrapResult.c1);

    // find r1 & r2 by starting at c1 moving toward p1 & p2
    calcTangentPoint(p1e, aWrapResult.r1, p1, m, a, vs, vs4);
    calcTangentPoint(p2e, aWrapResult.r2, p2, m, a, vs, vs4);

    // create a series of line segments connecting r1 & r2 along the
    // surface of the ellipsoid.

calc_wrap_path:
    CalcDistanceOnEllipsoid(aWrapResult.r1, aWrapResult.r2, m, a, vs, vs4, far_side_wrap, aWrapResult);

    if (_wrapSign != 0 && aWrapResult.wrap_pts.getSize() > 2 && ! far_side_wrap)
    {
        SimTK::Vec3 r1p1, r2p2, r1w1, r2w2;

        SimTK::Vec3& w1 = aWrapResult.wrap_pts.updElt(1);
        SimTK::Vec3& w2 = aWrapResult.wrap_pts.updElt(aWrapResult.wrap_pts.getSize() - 2);

        // check for wrong-way wrap by testing angle of first and last
        // wrap path segments:
        r1p1 = p1 - aWrapResult.r1;
        r1w1 = w1 - aWrapResult.r1;
        r2p2 = p2 - aWrapResult.r2;
        r2w2 = w2 - aWrapResult.r2;

        WrapMath::NormalizeOrZero(r1p1, r1p1);
        WrapMath::NormalizeOrZero(r1w1, r1w1);
        WrapMath::NormalizeOrZero(r2p2, r2p2);
        WrapMath::NormalizeOrZero(r2w2, r2w2);

        if ((~r1p1*r1w1) > 0.0 || (~r2p2*r2w2) > 0.0)
        {
            // NOTE: I added the ability to call CalcDistanceOnEllipsoid() a 2nd time in this
            //  situation to force a far-side wrap instead of aborting the
            //  wrap.   -- KMS 9/3/99
            far_side_wrap = true;

            goto calc_wrap_path;
        }
    }

    // unfactor the output coordinates
    aWrapResult.wrap_path_length /= aWrapResult.factor;

    for (i = 0; i < aWrapResult.wrap_pts.getSize(); i++)
        aWrapResult.wrap_pts[i] *= (1.0 / aWrapResult.factor);

    // transform back to starting coordinate system
    // Note: c1 and sv do not get transformed
    for (i = 0; i < 3; i++)
    {
        aWrapResult.r1[i] /= aWrapResult.factor;
        aWrapResult.r2[i] /= aWrapResult.factor;
    }

    return mandatoryWrap;
}

//_____________________________________________________________________________
/**
 * Adjust a point (r1) such that the point remains in
 * a specified plane (vs, vs4), and creates a tangent line segment connecting
 * it to a specified point (p1) outside of the ellipsoid. All quantities
 * are normalized.
 *
 * @param p1e Ellipsoid parameter for 'p1'?
 * @param r1 Point to be adjusted to satisfy tangency-within-a-plane constraint
 * @param p1 Point outside of ellipsoid
 * @param m Ellipsoid origin
 * @param a Ellipsoid axis
 * @param vs Plane vector
 * @param vs4 Plane coefficient
 * @return '1' if the point was adjusted, '0' otherwise
 */
int WrapEllipsoid::calcTangentPoint(double p1e, SimTK::Vec3& r1, SimTK::Vec3& p1, SimTK::Vec3& m,
                                                SimTK::Vec3& a, SimTK::Vec3& vs, double vs4) const
{
    int i, j, k, nit, nit2, maxit=50, maxit2=1000;
    Vec3 nr1, p1r1, p1m;
    double d1, v[4], ee[4], ssqo, ssq, pcos;
    double fakt, alpha=0.01, diag[4], vt[4], dd;
    SimTK::Mat44 dedth, dedth2, ddinv2;

    if (fabs(p1e) < 0.0001)
    {
        for (i = 0; i < 3; i++)
            r1[i] = p1[i];
    }
    else
    {
        for (i = 0; i < 3; i++)
            nr1[i] = 2.0 * (r1[i] - m[i])/(SQR(a[i]));

        d1 = -(~nr1*r1);
        ee[0] = (~vs*r1) + vs4;
        ee[1] = -1.0;

        for (i = 0; i < 3; i++)
            ee[1] += SQR((r1[i] - m[i]) / a[i]);

        ee[2] = (~nr1*r1) + d1;
        ee[3] = (~nr1*p1) + d1;

        ssqo = SQR(ee[0]) + SQR(ee[1]) + SQR(ee[2]) + SQR(ee[3]);
        ssq = ssqo;

        nit = 0;

        while ((ssq > ELLIPSOID_TINY) && (nit < maxit))
        {
            nit++;

            for (i = 0; i < 3; i++)
            {
                dedth[i][0] = vs[i];
                dedth[i][1] = 2.0 * (r1[i] - m[i]) / SQR(a[i]);
                dedth[i][2] = 2.0 * (2.0 * r1[i] - m[i]) / SQR(a[i]);
                dedth[i][3] = 2.0 * p1[i] / SQR(a[i]);
            }

            dedth[3][0] = 0.0;
            dedth[3][1] = 0.0;
            dedth[3][2] = 1.0;
            dedth[3][3] = 1.0;

            p1r1 = p1 - r1;
            WrapMath::NormalizeOrZero(p1r1, p1r1);

            p1m = p1 - m;
            WrapMath::NormalizeOrZero(p1m, p1m);

            pcos = (~p1r1*p1m);

            if (pcos > 0.1)
                dd = 1.0 - pow(pcos, 100);
            else
                dd = 1.0;

            for (i = 0; i < 4; i++)
            {
                v[i] = 0.0;

                for (j = 0; j < 4; j++)
                    v[i] -= dedth[i][j] * ee[j];
            }

            for (i = 0; i < 4; i++)
            {
                for (j = 0; j < 4; j++)
                {
                    dedth2[i][j] = 0.0;

                    for (k = 0; k < 4; k++)
                        dedth2[i][j] += dedth[i][k] * dedth[j][k];
                }
            }

            for (i = 0; i < 4; i++)
                diag[i] = dedth2[i][i];

            nit2 = 0;

            while ((ssq >= ssqo) && (nit2 < maxit2))
            {
                for (i = 0; i < 4; i++)
                    dedth2[i][i] = diag[i] * (1.0 + alpha);

                ddinv2 = dedth2.invert();

                for (i = 0; i < 4; i++)
                {
                    vt[i] = 0.0;

                    for (j = 0; j < 4; j++)
                        vt[i] += dd * ddinv2[i][j] * v[j] / 16.0;
                }

                for (i = 0; i < 3; i++)
                    r1[i] += vt[i];

                d1 += vt[3];

                for (i = 0; i < 3; i++)
                    nr1[i] = 2.0 * (r1[i] - m[i])/SQR(a[i]);

                ee[0] = (~vs*r1) + vs4;
                ee[1] = -1.0;

                for (i = 0; i < 3; i++)
                    ee[1] += SQR((r1[i] - m[i])/a[i]);

                ee[2] = (~nr1*r1) + d1;
                ee[3] = (~nr1*p1) + d1;

                ssqo = ssq;

                ssq = SQR(ee[0]) + SQR(ee[1]) + SQR(ee[2]) + SQR(ee[3]);

                alpha *= 4.0;
                nit2++;
            }

            alpha /= 8.0;

            fakt = 0.5;

            nit2 = 0;

            while ((ssq <= ssqo) && (nit2 < maxit2))
            {
                fakt *= 2.0;

                for (i = 0; i < 3; i++)
                    r1[i] += vt[i] * fakt;

                d1 += vt[3] * fakt;

                for (i = 0; i < 3; i++)
                    nr1[i] = 2.0 * (r1[i] - m[i]) / SQR(a[i]);

                ee[0] = (~vs*r1) + vs4;
                ee[1] = -1.0;

                for (i=0; i<3; i++)
                    ee[1] += SQR((r1[i] - m[i]) / a[i]);

                ee[2] = (~nr1*r1) + d1;
                ee[3] = (~nr1*p1) + d1;

                ssqo = ssq;     

                ssq = SQR(ee[0]) + SQR(ee[1]) + SQR(ee[2]) + SQR(ee[3]);

                nit2++;
            }

            for (i = 0; i < 3; i++)
                r1[i] -= vt[i] * fakt;

            d1 -= vt[3] * fakt;

            for (i=0; i<3; i++)
                nr1[i] = 2.0 * (r1[i] - m[i]) / SQR(a[i]);

            ee[0] = (~vs*r1) + vs4;
            ee[1] = -1.0;

            for (i = 0; i < 3; i++)
                ee[1] += SQR((r1[i] - m[i]) / a[i]);

            ee[2] = (~nr1*r1) + d1;
            ee[3] = (~nr1*p1) + d1;

            ssq = SQR(ee[0]) + SQR(ee[1]) + SQR(ee[2]) + SQR(ee[3]);
            ssqo = ssq;     
        }
    }   
    return 1;

}

//_____________________________________________________________________________
/**
 * Calculate the distance over the surface between two points on an ellipsoid.
 * All quantities are normalized.
 *
 * @param r1 The first point on the surface
 * @param r2 The second point on the surface
 * @param m Center of the ellipsoid
 * @param a Axes of the ellipsoid
 * @param vs Orientation plane vector
 * @param vs4 Orientation plane coefficient
 * @param far_side_wrap Whether or not the wrapping is the long way around
 * @param aWrapResult The wrapping results (tangent points, etc.)
 */
void WrapEllipsoid::CalcDistanceOnEllipsoid(SimTK::Vec3& r1, SimTK::Vec3& r2, SimTK::Vec3& m, SimTK::Vec3& a, 
                                                          SimTK::Vec3& vs, double vs4, bool far_side_wrap,
                                                          WrapResult& aWrapResult) const
{
    int i, j, k, l, imax, numPathSegments;
    SimTK::Vec3 u, ux, a0, ar1, ar2, vsy, vsz, t, r, f1, f2, dr, dv;
    double phi, dphi, phi0, len, mu, aa, bb, cc, mu3, s[500][3], r0[3][3], rphi[3][3], desiredSegLength = 0.001;

    dr = r1 - r2;
    len = dr.norm() / aWrapResult.factor;

    if (len < desiredSegLength) {
        // If the distance between r1 and r2 is very small, then don't bother
        // calculating wrap points along the surface of the ellipsoid.
        // Just use r1 and r2 as the surface points and return the distance
        // between them as the distance along the ellipsoid.
        aWrapResult.wrap_pts.setSize(0);
        //SimmPoint p1(r1);
        aWrapResult.wrap_pts.append(r1);
        //SimmPoint p2(r2);
        aWrapResult.wrap_pts.append(r2);
        aWrapResult.wrap_path_length = len * aWrapResult.factor; // the length is unnormalized later
        return;
    } else {
        // You don't know the wrap length until you divide it
        // into N pieces and add up the lengths of each one.
        // So calculate N based on the distance between r1 and r2.
        // desiredSegLength should really depend on the units of
        // the model, but for now assume it's in meters and use 0.001.
        numPathSegments = (int) (len / desiredSegLength);
        if (numPathSegments <= 0)
        {
            aWrapResult.wrap_path_length = len;
            return;
        }
        else if (numPathSegments > 499)
            numPathSegments = 499;
    }

    int numPathPts = numPathSegments + 1;
    int numInteriorPts = numPathPts - 2;

    ux[0] = 1.0;
    ux[1] = 0.0;
    ux[2] = 0.0;

    imax = 0;

    for (i = 1; i < 3; i++)
        if (fabs(vs[i]) > fabs(vs[imax]))
            imax = i;

    u[0] = u[1] = u[2] = 0.0;
    u[imax] = 1.0;

    mu = (-(~vs*m) - vs4) / (~vs*u);

    for (i=0;i<3;i++)
        a0[i] = m[i] + mu * u[i];

    ar1 = r1 - a0;
    WrapMath::NormalizeOrZero(ar1, ar1);
    ar2 = r2 - a0;
    WrapMath::NormalizeOrZero(ar2, ar2);

    phi0 = acos((~ar1*ar2));

    if (far_side_wrap)
        dphi = - (2 * SimTK_PI - phi0) / (double) numPathSegments;
    else
        dphi = phi0 / (double) numPathSegments;

    vsz = ar1 % ar2;
    WrapMath::NormalizeOrZero(vsz, vsz);
    vsy = vsz % ar1;

    for (i = 0; i < 3; i++)
    {
        for (j = 0; j < 3; j++)
        {
            rphi[i][j] = 0.0;
            r0[i][j] = 0.0;
        }
    }

    for (i = 0; i < 3; i++)
    {
        r0[i][0] = ar1[i];
        r0[i][1] = vsy[i];
        r0[i][2] = vsz[i];
    }

    rphi[2][2] = 1;

    for (i = 0; i < numInteriorPts; i++)
    {
        phi = (i + 1) * dphi;
        rphi[0][0] = cos(phi);
        rphi[0][1] = -sin(phi);
        rphi[1][0] = sin(phi);
        rphi[1][1] = cos(phi);

        for (j = 0; j < 3; j++)
        {
            r[j] = 0.0;

            for (k = 0; k < 3; k++)
            {
                t[k] = 0.0;

                for (l = 0; l < 3; l++)
                    t[k] += rphi[k][l] * ux[l];

                r[j] += r0[j][k] * t[k];
            }
        }

        for (j = 0; j < 3; j++)
        {
            f1[j] = r[j]/a[j];
            f2[j] = (a0[j] - m[j])/a[j];
        }

        aa = (~f1*f1);
        bb = 2.0 * ((~f1*f2));
        cc = (~f2*f2) - 1.0;
        mu3 = (-bb + sqrt(SQR(bb) - 4.0 * aa * cc)) / (2.0 * aa);

        for (j = 0; j < 3; j++)
            s[i][j] = a0[j] + mu3 * r[j];
    }

    aWrapResult.wrap_pts.setSize(0);
    //SimmPoint p1(r1);
    aWrapResult.wrap_pts.append(r1);

    for (i = 0; i < numInteriorPts; i++)
    {
        Vec3 spt(Vec3::getAs(&s[i][0]));
        aWrapResult.wrap_pts.append(spt);
    }

    //SimmPoint p2(r2);
    aWrapResult.wrap_pts.append(r2);

    aWrapResult.wrap_path_length = 0.0;

    for (i = 0; i < numPathSegments; i++)
    {
        Vec3 p = aWrapResult.wrap_pts.get(i);
        Vec3 q = aWrapResult.wrap_pts.get(i+1);
        dv = q - p;
        aWrapResult.wrap_path_length += dv.norm(); //WrapMath::Magnitude(3, dv);
    }
}

//_____________________________________________________________________________
/**
 * Calculate the point on an ellipsoid that is closest to a point in 3D space.
 * This function is courtesy of Dave Eberly, Graphics Gems IV.
 *
 * Input:   Ellipsoid (x/a)^2+(y/b)^2+(z/c)^2 = 1, point (u,v,w).
 *
 * Output:  Closest point (x,y,z) on ellipsoid to (u,v,w), function returns
 *          the distance sqrt((x-u)^2+(y-v)^2+(z-w)^2).
 *
 * @param a X dimension of the ellipsoid
 * @param b Y dimension of the ellipsoid
 * @param c Z dimension of the ellipsoid
 * @param u X coordinate of the point in space
 * @param v Y coordinate of the point in space
 * @param w Z coordinate of the point in space
 * @param x X coordinate of the closest point
 * @param y Y coordinate of the closest point
 * @param z Z coordinate of the closest point
 * @param specialCaseAxis For dealing with uvw points near a major axis
 * @return The distance between the ellipsoid and the point in space
 */
double WrapEllipsoid::findClosestPoint(double a, double b, double c,
                                                    double u, double v, double w,
                                                    double* x, double* y, double* z,
                                                    int specialCaseAxis) const
{
    // Graphics Gems IV algorithm for computing distance from point to
    // ellipsoid (x/a)^2 + (y/b)^2 +(z/c)^2 = 1.  The algorithm as stated
    // is not stable for points near the coordinate planes.  The first part
    // of this code handles those points separately.
    int i,j;
    
    // handle points near the coordinate planes by reducing the problem
    // to a 2-dimensional pt-to-ellipse.
    //
    // if uvw is close to more than one coordinate plane,  pick the 2d
    // elliptical cross-section with the narrowest radius.
    if (specialCaseAxis < 0)
    {
         double uvw[3], minEllipseRadiiSum = SimTK::Infinity;
       
       uvw[0] = u; uvw[1] = v; uvw[2] = w;

       for (i = 0; i < 3; i++)
       {
           if (EQUAL_WITHIN_ERROR(0.0,uvw[i]))
           {
               double ellipseRadiiSum = 0;

               for (j = 0; j < 3; j++)
                   if (j != i)
                       ellipseRadiiSum += get_dimensions()[j];

               if (minEllipseRadiiSum > ellipseRadiiSum)
               {
                   specialCaseAxis = i;
                   minEllipseRadiiSum = ellipseRadiiSum;
               }
           }
       }
    }
    if (specialCaseAxis == 0) // use elliptical cross-section in yz plane
    {
       *x = u;
       return closestPointToEllipse(b, c, v, w, y, z);
    }
    if (specialCaseAxis == 1) // use elliptical cross-section in xz plane
    {
       *y = v;
       return closestPointToEllipse(c, a, w, u, z, x);
    }
    if (specialCaseAxis == 2) // use elliptical cross-section in xy plane
    {
       *z = w;
       return closestPointToEllipse(a, b, u, v, x, y);
    }

    // if we get to here, then the point is not close to any of the major planes,
    // so we can solve the general case.
    {
        double a2 = a*a, b2 = b*b, c2 = c*c;
        double u2 = u*u, v2 = v*v, w2 = w*w;
        double a2u2 = a2*u2, b2v2 = b2*v2, c2w2 = c2*w2;
        double dx, dy, dz, t, f;

        // initial guess
        if ( (u/a)*(u/a) + (v/b)*(v/b) + (w/c)*(w/c) < 1.0 )
        {
            t = 0.0;
        }
        else
        {
            double max = a;
        
            if ( b > max )
                max = b;
            if ( c > max )
                max = c;

            t = max*sqrt(u*u+v*v+w*w);
        }

        double P{ 0 }, P2{ 0 }, Q{ 0 }, Q2{ 0 }, R{ 0 }, _R2{ 0 };
        double PQ{ 0 }, PR{ 0 }, QR{ 0 }, PQR{ 0 }, fp{ 0 };

        for (i = 0; i < 64; i++)
        {
            P = t+a2, P2 = P*P;
            Q = t+b2, Q2 = Q*Q;
            R = t+c2, _R2 = R*R;

            f = P2*Q2*_R2 - a2u2*Q2*_R2 - b2v2*P2*_R2 - c2w2*P2*Q2;
        
            if ( fabs(f) < 1e-09 )
            {
                *x = a2 * u / P;
                *y = b2 * v / Q;
                *z = c2 * w / R;
            
                dx = *x - u;
                dy = *y - v;
                dz = *z - w;
            
                return sqrt(dx*dx+dy*dy+dz*dz);
            }

            PQ = P*Q;
            PR = P*R;
            QR = Q*R;
            PQR = P*Q*R;
            fp = 2.0*(PQR*(QR+PR+PQ)-a2u2*QR*(Q+R)-b2v2*PR*(P+R)-c2w2*PQ*(P+Q));
        
            t -= f/fp;
        }
    }

    return -1.0;
}

//_____________________________________________________________________________
/**
 * Calculate the point on an ellipse that is closest to a point in 2D space.
 * This function is courtesy of Dave Eberly, Graphics Gems IV.
 *
 * Input:   Ellipse (x/a)^2+(y/b)^2 = 1, point (u,v).
 *
 * Output:  Closest point (x,y) on ellipse to (u,v), function returns
 *          the distance sqrt((x-u)^2+(y-v)^2).
 *
 * @param a X dimension of the ellipse
 * @param b Y dimension of the ellipse
 * @param u X coordinate of point on ellipse
 * @param v Y coordinate of point on ellipse
 * @param x X coordinate of closest point
 * @param y Y coordinate of closest point
 * @return Distance between uv point and ellipse
 */
double WrapEllipsoid::closestPointToEllipse(double a, double b, double u,
                                                          double v, double* x, double* y) const
{
    // Graphics Gems IV algorithm for computing distance from point to
    // ellipse (x/a)^2 + (y/b)^2 = 1.  The algorithm as stated is not stable
    // for points near the coordinate axes.  The first part of this code
    // handles those points separately.
    double a2 = a*a, b2 = b*b;
    double u2 = u*u, v2 = v*v;
    double a2u2 = a2*u2, b2v2 = b2*v2;
    double dx, dy, xda, ydb;
    int i/*, which*/;
    double t, P, Q, P2, Q2, f, fp;

    bool nearXOrigin = (bool) EQUAL_WITHIN_ERROR(0.0,u);
    bool nearYOrigin = (bool) EQUAL_WITHIN_ERROR(0.0,v);

    // handle points near the coordinate axes
    if (nearXOrigin && nearYOrigin)
    {
        if (a < b)
        {
            *x = (u < 0.0 ? -a : a);
            *y = v;
            return a;
        } else {
            *x = u;
            *y = (v < 0.0 ? -b : b);
            return b;
        }
    }

    if (nearXOrigin)  // u == 0
    {
        if ( a >= b || fabs(v) >= b - a2/b )
        {
            *x = u;
            *y = ( v >= 0 ? b : -b );
            dy = *y - v;
            return fabs(dy);
        }
        else
        {
            *y = b2 * v / (b2-a2);
            dy = *y - v;
            ydb = *y / b;
            *x = a * sqrt(fabs(1 - ydb*ydb));
            return sqrt(*x * *x + dy*dy);
        }
    }

    if (nearYOrigin)  // v == 0
    {
        if ( b >= a || fabs(u) >= a - b2/a )
        {
            *x = ( u >= 0 ? a : -a );
            dx = *x - u;
            *y = v;
            return fabs(*x - u);
        }
        else
        {
            *x = a2 * u / (a2-b2);
            dx = *x - u;
            xda = *x / a;
            *y = b * sqrt(fabs(1 - xda*xda));
            return sqrt(dx*dx + *y * *y);
        }
    }

    // initial guess
    if ( (u/a)*(u/a) + (v/b)*(v/b) < 1.0 )
    {
        //which = 0;
        t = 0.0;
    }
    else
    {
        double max = a;

        //which = 1;
        
        if ( b > max )
            max = b;

        t = max * sqrt(u*u + v*v);
    }

    for (i = 0; i < 64; i++)
    {
        P = t+a2;
        P2 = P*P;
        Q = t+b2;
        Q2 = Q*Q;
        f = P2*Q2 - a2u2*Q2 - b2v2*P2;

        if ( fabs(f) < 1e-09 )
            break;

        fp = 2.0 * (P*Q*(P+Q) - a2u2*Q - b2v2*P);
        t -= f / fp;
    }

    *x = a2 * u / P;
    *y = b2 * v / Q;
    dx = *x - u;
    dy = *y - v;

    return sqrt(dx*dx + dy*dy);
}
// Implement generateDecorations by WrapEllipsoid to replace the previous out of place implementation 
// in ModelVisualizer
void WrapEllipsoid::generateDecorations(bool fixed, const ModelDisplayHints& hints, const SimTK::State& state,
    SimTK::Array_<SimTK::DecorativeGeometry>& appendToThis) const 
{

    Super::generateDecorations(fixed, hints, state, appendToThis);
    if (!fixed) return;

    if (hints.get_show_wrap_geometry()) {
        const Appearance& defaultAppearance = get_Appearance();
        if (!defaultAppearance.get_visible()) return;
        const Vec3 color = defaultAppearance.get_color();
        
        const auto X_BP = calcWrapGeometryTransformInBaseFrame();
        appendToThis.push_back(
            SimTK::DecorativeEllipsoid(getRadii())
            .setTransform(X_BP).setResolution(2.0)
            .setColor(color).setOpacity(defaultAppearance.get_opacity())
            .setScale(1).setRepresentation(defaultAppearance.get_representation())
            .setBodyId(getFrame().getMobilizedBodyIndex()));
    }

}
