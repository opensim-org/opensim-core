/* -------------------------------------------------------------------------- *
 *                          OpenSim:  WrapSphere.cpp                          *
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
#include "WrapSphere.h"
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

static const char* wrapTypeName = "sphere";
//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */
WrapSphere::WrapSphere() :
    WrapObject(),
   _radius(_radiusProp.getValueDbl())
{
    setNull();
    setupProperties();
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
WrapSphere::~WrapSphere()
{
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aWrapSphere WrapSphere to be copied.
 */
WrapSphere::WrapSphere(const WrapSphere& aWrapSphere) :
    WrapObject(aWrapSphere),
   _radius(_radiusProp.getValueDbl())
{
    setNull();
    setupProperties();
    copyData(aWrapSphere);
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the data members of this WrapSphere to their null values.
 */
void WrapSphere::setNull()
{
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void WrapSphere::setupProperties()
{
    // BASE CLASS
    //WrapObject::setupProperties();

    _radiusProp.setName("radius");
    _radiusProp.setValue(-1.0);
    _propertySet.append(&_radiusProp);
}

//_____________________________________________________________________________
/**
 * Perform some set up functions that happen after the
 * object has been deserialized or copied.
 *
 * @param aModel OpenSim model.
 */
void WrapSphere::connectToModelAndBody(Model& aModel, OpenSim::Body& aBody)
{
    // Base class
    Super::connectToModelAndBody(aModel, aBody);

   // maybe set a parent pointer, _body = aBody;

    if (_radius < 0.0)
    {
        string errorMessage = "Error: radius for wrapSphere " + getName() + " was either not specified, or is negative.";
        throw Exception(errorMessage);
    }

    _displayer.freeGeometry();
    AnalyticSphere* sphere = new AnalyticSphere(_radius);
    setGeometryQuadrants(sphere);
    _displayer.addGeometry(sphere);
}

//_____________________________________________________________________________
/**
 * Scale the sphere by the average of the three scale factors. The base class
 * scales the origin of the sphere in the body's reference frame.
 *
 * @param aScaleFactors The XYZ scale factors.
 */
void WrapSphere::scale(const SimTK::Vec3& aScaleFactors)
{
   WrapObject::scale(aScaleFactors);

   _radius *= (aScaleFactors.sum() / 3.0);
}

//_____________________________________________________________________________
/**
 * Copy data members from one WrapSphere to another.
 *
 * @param aWrapSphere WrapSphere to be copied.
 */
void WrapSphere::copyData(const WrapSphere& aWrapSphere)
{
    // BASE CLASS
    WrapObject::copyData(aWrapSphere);

    _radius = aWrapSphere._radius;
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
    dimensions << "radius " << _radius;

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
    return _radius;
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
WrapSphere& WrapSphere::operator=(const WrapSphere& aWrapSphere)
{
    // BASE CLASS
    WrapObject::operator=(aWrapSphere);

    return(*this);
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
   double l1, l2, disc, a, b, c, a1, a2, j1, j2, j3, j4, r1r2, ra[3][3], rrx[3][3], aa[3][3], mat[4][4], 
            axis[4], vec[4], rotvec[4], angle, *r11, *r22;
    Vec3 ri, p2m, p1m, mp, r1n, r2n,
            p1p2, np2, hp2, r1m, r2m, y, z, n, r1a, r2a,
            r1b, r2b, r1am, r2am, r1bm, r2bm;
            
   int i, j, maxit, return_code = wrapped;
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

   maxit = 50;
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
    if (Mtx::Magnitude(3, p1m) < _radius || Mtx::Magnitude(3, p2m) < _radius)
      return insideRadius;

    a = Mtx::DotProduct(3, ri, ri);
   b = -2.0 * Mtx::DotProduct(3, mp, ri);
   c = Mtx::DotProduct(3, mp, mp) - _radius * _radius;
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

    Mtx::Normalize(3, p1p2, p1p2);
    Mtx::Normalize(3, p2m, np2);

    Mtx::CrossProduct(p1p2, np2, hp2);

   // if the muscle line passes too close to the center of the sphere
   // then give up
    if (Mtx::Magnitude(3, hp2) < 0.00001) {
        // JPL 12/28/06: r1 and r2 from the previous wrap have already
        // been copied into aWrapResult (and not yet overwritten). So
        // just go directly to calc_path.
#if 0
      // no wait!  don't give up!  Instead use the previous r1 & r2:
      // -- added KMS 9/9/99
      //
        const WrapResult& previousWrap = aPathWrap.getPreviousWrap();
      for (i = 0; i < 3; i++) {
         aWrapResult.r1[i] = previousWrap.r1[i];
         aWrapResult.r2[i] = previousWrap.r2[i];
      }
#endif
      goto calc_path;
   }

   // calc tangent point candidates r1a, r1b
    Mtx::Normalize(3, hp2, n);
    for (i = 0; i < 3; i++)
        y[i] = origin[i] - aPoint1[i];
    Mtx::Normalize(3, y, y);
    Mtx::CrossProduct(n, y, z);
   
   for (i = 0; i < 3; i++)
   {
      ra[i][0] = n[i];
      ra[i][1] = y[i];
      ra[i][2] = z[i];
   }

    a1 = asin(_radius / Mtx::Magnitude(3, p1m));

    WrapMath::Make3x3DirCosMatrix(a1, rrx);
    Mtx::Multiply(3, 3, 3, (double*)ra, (double*)rrx, (double*)aa);
    // TODO: test that this gives same result as SIMM code

   for (i = 0; i < 3; i++)
      r1a[i] = aPoint1[i] + aa[i][1] * Mtx::Magnitude(3, p1m) * cos(a1);

   WrapMath::Make3x3DirCosMatrix(-a1, rrx);
    Mtx::Multiply(3, 3, 3, (double*)ra, (double*)rrx, (double*)aa);

   for (i = 0; i < 3; i++)
      r1b[i] = aPoint1[i] + aa[i][1] * Mtx::Magnitude(3, p1m) * cos(a1);

   // calc tangent point candidates r2a, r2b
    for (i = 0; i < 3; i++)
        y[i] = origin[i] - aPoint2[i];
    Mtx::Normalize(3, y, y);
    Mtx::CrossProduct(n, y, z);

   for (i = 0; i < 3; i++)
   {
      ra[i][0] = n[i];
      ra[i][1] = y[i];
      ra[i][2] = z[i];
   }

   a2 = asin(_radius / Mtx::Magnitude(3, p2m));
   
   WrapMath::Make3x3DirCosMatrix(a2, rrx);
    Mtx::Multiply(3, 3, 3, (double*)ra, (double*)rrx, (double*)aa);

   for (i = 0; i < 3; i++)
      r2a[i] = aPoint2[i] + aa[i][1] * Mtx::Magnitude(3, p2m) * cos(a2);

   WrapMath::Make3x3DirCosMatrix(-a2, rrx);
    Mtx::Multiply(3, 3, 3, (double*)ra, (double*)rrx, (double*)aa);

   for (i = 0; i < 3; i++)
      r2b[i] = aPoint2[i] + aa[i][1] * Mtx::Magnitude(3, p2m) * cos(a2);

   // determine wrapping tangent points r1 & r2
    for (i = 0; i < 3; i++) {
        r1am[i] = r1a[i] - origin[i];
        r1bm[i] = r1b[i] - origin[i];
        r2am[i] = r2a[i] - origin[i];
        r2bm[i] = r2b[i] - origin[i];
    }

    Mtx::Normalize(3, r1am, r1am);
    Mtx::Normalize(3, r1bm, r1bm);
    Mtx::Normalize(3, r2am, r2am);
    Mtx::Normalize(3, r2bm, r2bm);
   
   {
      // check which of the tangential points results in the shortest distance
        j1 = Mtx::DotProduct(3, r1am, r2am);
      j2 = Mtx::DotProduct(3, r1am, r2bm);
      j3 = Mtx::DotProduct(3, r1bm, r2am);
      j4 = Mtx::DotProduct(3, r1bm, r2bm);
       
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
         double tt, r_squared = _radius * _radius;
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

            Mtx::Normalize(3, sum_musc, sum_musc);

            if (Mtx::DotProduct(3, r1am, sum_musc) > Mtx::DotProduct(3, r1bm, sum_musc))
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

            if (Mtx::DotProduct(3, r2am, sum_musc) > Mtx::DotProduct(3, r2bm, sum_musc))
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

            Mtx::Normalize(3, sum_musc, sum_musc);

            if (Mtx::DotProduct(3, sum_musc, wrapaxis) < 0.0)
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

            if (Mtx::DotProduct(3, sum_r, sum_musc) < 0.0)
            far_side_wrap = true;
      }
   }

 calc_path:
    for (i = 0; i < 3; i++) {
        r1m[i] = aWrapResult.r1[i] - origin[i];
        r2m[i] = aWrapResult.r2[i] - origin[i];
    }

    Mtx::Normalize(3, r1m, r1n);
    Mtx::Normalize(3, r2m, r2n);

    angle = acos(Mtx::DotProduct(3, r1n, r2n));
   
   if (far_side_wrap)
        angle = -(2 * SimTK_PI - angle);
   
   r1r2 = _radius * angle;
   aWrapResult.wrap_path_length = r1r2;

    Vec3 axis3;
    Mtx::CrossProduct(r1n, r2n, axis3);
    Mtx::Normalize(3, axis3, axis3);

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
   vec[3] = 1.0;

   for (i = 0; i < numWrapSegments - 2; i++) {
        double wangle = angle * (i+1) / (numWrapSegments - 1) * SimTK_DEGREE_TO_RADIAN;

        WrapMath::ConvertAxisAngleTo4x4DirCosMatrix(Vec3::getAs(axis), wangle, mat);
        Mtx::Multiply(4, 4, 1, (double*)mat, (double*)vec, (double*)rotvec);

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
