/* -------------------------------------------------------------------------- *
 *                          OpenSim:  WrapSphere.cpp                          *
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
#include "WrapSphere.h"
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

static const char* wrapTypeName = "sphere";
//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */
WrapSphere::WrapSphere()
{
    constructProperties();
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
WrapSphere::~WrapSphere()
{
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void WrapSphere::constructProperties()
{
    constructProperty_radius(0.05);
}

void WrapSphere::extendScale(const SimTK::State& s, const ScaleSet& scaleSet)
{
    Super::extendScale(s, scaleSet);

    // Get scale factors (if an entry for the Frame's base Body exists).
    const Vec3& scaleFactors = getScaleFactors(scaleSet, getFrame());
    if (scaleFactors == ModelComponent::InvalidScaleFactors)
        return;

    upd_radius() *= (scaleFactors.sum() / 3.);
}

//_____________________________________________________________________________
/**
 * Perform some set up functions that happen after the
 * object has been deserialized or copied.
 *
 * @param aModel OpenSim model.
 */
void WrapSphere::extendFinalizeFromProperties()
{
    // Base class
    Super::extendFinalizeFromProperties();

    // maybe set a parent pointer, _body = aBody;
    OPENSIM_THROW_IF_FRMOBJ(
        get_radius() < 0,
        InvalidPropertyValue,
        getProperty_radius().getName(),
        "Radius cannot be less than zero");

/*
    Sphere* sphere = new Sphere(_radius);
    setGeometryQuadrants(sphere);
*/
}

//_____________________________________________________________________________
/**
 * Get the name of the type of wrap object ("sphere" in this case)
 *
 * @return A string representing the type of wrap object
 */
const char* WrapSphere::getWrapTypeName() const
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
string WrapSphere::getDimensionsString() const
{
    stringstream dimensions;
    dimensions << "radius " << get_radius();

    return dimensions.str();
}

//_____________________________________________________________________________
/**
 * Get the radius of the sphere.
 *
 * @return The radius of the sphere
 */
double WrapSphere::getRadius() const
{
    return get_radius();
}

//=============================================================================
// WRAPPING
//=============================================================================
//_____________________________________________________________________________
/**
 * Calculate the wrapping of one line segment over the sphere.
 *
 * @param aPoint1 One end of the line segment
 * @param aPoint2 The other end of the line segment
 * @param aPathWrap An object holding the parameters for this line/sphere pairing
 * @param aWrapResult The result of the wrapping (tangent points, etc.)
 * @param aFlag A flag for indicating errors, etc.
 * @return The status, as a WrapAction enum
 */
int WrapSphere::wrapLine(const SimTK::State& s, SimTK::Vec3& aPoint1, SimTK::Vec3& aPoint2,
                                 const PathWrap& aPathWrap, WrapResult& aWrapResult, bool& aFlag) const
{
   double l1, l2, disc, a, b, c, a1, a2, j1, j2, j3, j4, r1r2,
            axis[4], angle, *r11, *r22;
    Vec3 ri, p2m, p1m, mp, r1n, r2n,
            p1p2, np2, hp2, r1m, r2m, y, z, n, r1a, r2a,
            r1b, r2b, r1am, r2am, r1bm, r2bm;
    SimTK::Vec3 vec, rotvec;
    SimTK::Mat33 ra, aa;
    SimTK::Rotation rrx;

   int i, j,/* maxit, */ return_code = wrapped;
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

   //maxit = 50;
   aFlag = true;

    aWrapResult.wrap_pts.setSize(0);

    for (i = 0; i < 3; i++) {
        p1m[i] = aPoint1[i] - origin[i];
        p2m[i] = aPoint2[i] - origin[1];
        ri[i] = aPoint1[i] - aPoint2[i];
        mp[i] = origin[i] - aPoint2[i];
        p1p2[i] = aPoint1[i] - aPoint2[i];
    }

   // check that neither point is inside the radius of the sphere
    if (p1m.norm() < get_radius() || p2m.norm() < get_radius())
      return insideRadius;

   a = (~ri*ri);
   b = -2.0 * (~mp*ri);
   c = (~mp*mp) - get_radius() * get_radius();
   disc = b * b - 4.0 * a * c;

   // check if there is an intersection of p1p2 and the sphere
   if (disc < 0.0) 
   {
      aFlag = false;
        aWrapResult.wrap_path_length = 0.0;
      return noWrap;
   }

   l1 = (-b + sqrt(disc)) / (2.0 * a);
   l2 = (-b - sqrt(disc)) / (2.0 * a);

   // check if the intersection is between p1 and p2
   if ( ! (0.0 < l1 && l1 < 1.0) || ! (0.0 < l2 && l2 < 1.0))   
   {
      aFlag = false;
      aWrapResult.wrap_path_length = 0.0;
      return noWrap;
   }

   if (l1 < l2) 
   {
      aFlag = false;
      aWrapResult.wrap_path_length = 0.0;
      return noWrap;
   }

    WrapMath::NormalizeOrZero(p1p2, p1p2);
    WrapMath::NormalizeOrZero(p2m, np2);

    hp2 = p1p2 % np2;

   // if the muscle line passes too close to the center of the sphere
   // then give up
    if (hp2.norm() < 0.00001) {
        // JPL 12/28/06: r1 and r2 from the previous wrap have already
        // been copied into aWrapResult (and not yet overwritten). So
        // just go directly to calc_path.
        goto calc_path;
   }

   // calc tangent point candidates r1a, r1b
    WrapMath::NormalizeOrZero(hp2, n);
    for (i = 0; i < 3; i++)
        y[i] = origin[i] - aPoint1[i];
    WrapMath::NormalizeOrZero(y, y);
    z = n % y;
   
   for (i = 0; i < 3; i++)
   {
      ra[i][0] = n[i];
      ra[i][1] = y[i];
      ra[i][2] = z[i];
   }

    a1 = asin(get_radius() / p1m.norm());

    rrx.setRotationFromAngleAboutX(a1);
    aa = ra * ~rrx;

   for (i = 0; i < 3; i++)
      r1a[i] = aPoint1[i] + aa[i][1] * p1m.norm() * cos(a1);

   rrx.setRotationFromAngleAboutX(-a1);
   aa = ra * ~rrx;

   for (i = 0; i < 3; i++)
      r1b[i] = aPoint1[i] + aa[i][1] * p1m.norm() * cos(a1);

   // calc tangent point candidates r2a, r2b
    for (i = 0; i < 3; i++)
        y[i] = origin[i] - aPoint2[i];
    WrapMath::NormalizeOrZero(y, y);
    z = n % y;

   for (i = 0; i < 3; i++)
   {
      ra[i][0] = n[i];
      ra[i][1] = y[i];
      ra[i][2] = z[i];
   }

   a2 = asin(get_radius() / p2m.norm());

   rrx.setRotationFromAngleAboutX(a2);
   aa = ra * ~rrx;

   for (i = 0; i < 3; i++)
      r2a[i] = aPoint2[i] + aa[i][1] * p2m.norm() * cos(a2);

   rrx.setRotationFromAngleAboutX(-a2);
   aa = ra * ~rrx;

   for (i = 0; i < 3; i++)
      r2b[i] = aPoint2[i] + aa[i][1] * p2m.norm() * cos(a2);

   // determine wrapping tangent points r1 & r2
    r1am = r1a - origin;
    r1bm = r1b - origin;
    r2am = r2a - origin;
    r2bm = r2b - origin;

    WrapMath::NormalizeOrZero(r1am, r1am);
    WrapMath::NormalizeOrZero(r1bm, r1bm);
    WrapMath::NormalizeOrZero(r2am, r2am);
    WrapMath::NormalizeOrZero(r2bm, r2bm);
   
   {
      // check which of the tangential points results in the shortest distance
      j1 = (~r1am*r2am);
      j2 = (~r1am*r2bm);
      j3 = (~r1bm*r2am);
      j4 = (~r1bm*r2bm);
       
      if (j1 > j2 && j1 > j3 && j1 > j4)
      {
            for (i = 0; i < 3; i++) {
                aWrapResult.r1[i] = r1a[i];
                aWrapResult.r2[i] = r2a[i];
            }
         r11 = &r1b[0];
         r22 = &r2b[0];
      }
      else if (j2 > j3 && j2 > j4)
      {
            for (i = 0; i < 3; i++) {
                aWrapResult.r1[i] = r1a[i];
                aWrapResult.r2[i] = r2b[i];
            }
         r11 = &r1b[0];
         r22 = &r2a[0];
      }
      else if (j3 > j4)
      {
            for (i = 0; i < 3; i++) {
                aWrapResult.r1[i] = r1b[i];
                aWrapResult.r2[i] = r2a[i];
            }
         r11 = &r1a[0];
         r22 = &r2b[0];
      }
      else
      {
            for (i = 0; i < 3; i++) {
                aWrapResult.r1[i] = r1b[i];
                aWrapResult.r2[i] = r2b[i];
            }
         r11 = &r1a[0];
         r22 = &r2a[0];
      }
   }

   if (_wrapSign != 0)
   {
      if (DSIGN(aPoint1[_wrapAxis]) == _wrapSign || DSIGN(aPoint2[_wrapAxis]) == _wrapSign)
      {
         double tt, r_squared = get_radius() * get_radius();
            Vec3 mm;
         // If either muscle point is on the constrained side, then check for intersection
         // of the muscle line and the cylinder. If there is an intersection, then
         // you've found a mandatory wrap. If not, then if one point is not on the constrained
         // side and the closest point on the line is not on the constrained side, you've
         // found a potential wrap. Otherwise, there is no wrap.
         WrapMath::GetClosestPointOnLineToPoint(origin, aPoint1, p1p2, mm, tt);

         tt = -tt; // because p1p2 is actually aPoint2->aPoint1

         if (WrapMath::CalcDistanceSquaredBetweenPoints(origin, mm) < r_squared && tt > 0.0 && tt < 1.0)
         {
            return_code = mandatoryWrap;
         }
         else
         {
            if (DSIGN(aPoint1[_wrapAxis]) != DSIGN(aPoint2[_wrapAxis]) && DSIGN(mm[_wrapAxis]) != _wrapSign)
            {
               return_code = wrapped;
            }
            else
            {
               return noWrap;
            }
         }
      }

      if (DSIGN(aPoint1[_wrapAxis]) != _wrapSign || DSIGN(aPoint2[_wrapAxis]) != _wrapSign)
      {
         SimTK::Vec3 wrapaxis, sum_musc, sum_r;

         for (i = 0; i < 3; i++)
            wrapaxis[i] = (i == _wrapAxis) ? (double) _wrapSign : 0.0;

         // determine best constrained r1 & r2 tangent points:
         for (i = 0; i < 3; i++)
            sum_musc[i] = (origin[i] - aPoint1[i]) + (origin[i] - aPoint2[i]);

         WrapMath::NormalizeOrZero(sum_musc, sum_musc);

            if ((~r1am*sum_musc) > (~r1bm*sum_musc))
         {
                for (i = 0; i < 3; i++)
                    aWrapResult.r1[i] = r1a[i];
            r11 = &r1b[0];
         }
         else
         {
                for (i = 0; i < 3; i++)
                    aWrapResult.r1[i] = r1b[i];
            r11 = &r1a[0];
         }

            if ((~r2am*sum_musc) > (~r2bm*sum_musc))
         {
                for (i = 0; i < 3; i++)
                    aWrapResult.r2[i] = r2a[i];
            r22 = &r2b[0];
         }
         else
         {
                for (i = 0; i < 3; i++)
                    aWrapResult.r2[i] = r2b[i];
            r22 = &r2a[0];
         }

         // flip if necessary:
         for (i = 0; i < 3; i++)
            sum_musc[i] = (aWrapResult.r1[i] - aPoint1[i]) + (aWrapResult.r2[i] - aPoint2[i]);

         WrapMath::NormalizeOrZero(sum_musc, sum_musc);

            if ((~sum_musc*wrapaxis) < 0.0)
         {
                for (i = 0; i < 3; i++) {
                    aWrapResult.r1[i] = r11[i];
                    aWrapResult.r2[i] = r22[i];
                }
         }

         // determine if the resulting tangent points create a far side wrap
         for (i = 0; i < 3; i++) {
            sum_musc[i] = (aWrapResult.r1[i] - aPoint1[i]) + (aWrapResult.r2[i] - aPoint2[i]);
            sum_r[i] = (aWrapResult.r1[i] - origin[i]) + (aWrapResult.r2[i] - origin[i]);
         }

         if ((~sum_r*sum_musc) < 0.0)
            far_side_wrap = true;
      }
   }

 calc_path:
    for (i = 0; i < 3; i++) {
        r1m[i] = aWrapResult.r1[i] - origin[i];
        r2m[i] = aWrapResult.r2[i] - origin[i];
    }

    WrapMath::NormalizeOrZero(r1m, r1n);
    WrapMath::NormalizeOrZero(r2m, r2n);

    angle = acos((~r1n*r2n));
   
   if (far_side_wrap)
        angle = -(2 * SimTK_PI - angle);
   
   r1r2 = get_radius() * angle;
   aWrapResult.wrap_path_length = r1r2;

    Vec3 axis3 = r1n % r2n;
    WrapMath::NormalizeOrZero(axis3, axis3);

   for(int ii=0; ii<3; ii++) axis[ii]=axis3[ii];
   axis[3] = 1.0;

    aWrapResult.wrap_pts.setSize(0);

    // Each muscle segment on the surface of the sphere should be
    // 0.002 meters long. This assumes the model is in meters, of course.
    int numWrapSegments = (int) (aWrapResult.wrap_path_length / 0.002);
    if (numWrapSegments < 1)
        numWrapSegments = 1;

    //SimmPoint sp1(aWrapResult.r1);
    aWrapResult.wrap_pts.append(aWrapResult.r1);

   vec[0] = r1m[0];
   vec[1] = r1m[1];
   vec[2] = r1m[2];

    SimTK::Rotation R;
    for (i = 0; i < numWrapSegments - 2; i++) {
        double wangle = angle * (i+1) / (numWrapSegments - 1) * SimTK_DEGREE_TO_RADIAN;

        R.setRotationFromAngleAboutNonUnitVector(wangle, Vec3::getAs(axis));
        rotvec = ~R * vec;

        SimTK::Vec3 wp;
        for (j = 0; j < 3; j++)
            wp[j] = origin[j] + rotvec[j];
        //SimmPoint wppt(wp);
        aWrapResult.wrap_pts.append(wp);
   }

    //SimmPoint sp2(aWrapResult.r2);
    aWrapResult.wrap_pts.append(aWrapResult.r2);

   return return_code;
}

// Implement generateDecorations by WrapSphere to replace the previous out of place implementation 
// in ModelVisualizer
void WrapSphere::generateDecorations(bool fixed, const ModelDisplayHints& hints, const SimTK::State& state,
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
            SimTK::DecorativeSphere(getRadius())
            .setTransform(X_BP).setResolution(2.0)
            .setColor(color).setOpacity(defaultAppearance.get_opacity())
            .setScale(1).setRepresentation(defaultAppearance.get_representation())
            .setBodyId(getFrame().getMobilizedBodyIndex()));
    }


}
