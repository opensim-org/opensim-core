/* -------------------------------------------------------------------------- *
 *                          OpenSim:  WrapTorus.cpp                           *
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
#include "WrapTorus.h"
#include "WrapCylinder.h"
#include <OpenSim/Simulation/Model/PathPoint.h>
#include "PathWrap.h"
#include "WrapResult.h"
#include <OpenSim/Common/SimmMacros.h>
#include <OpenSim/Common/Lmdif.h>
#include <OpenSim/Common/Mtx.h>
#include <sstream>

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace OpenSim;
using SimTK::Vec3;

static const char* wrapTypeName = "torus";

#define CYL_LENGTH 10000.0

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */
WrapTorus::WrapTorus() :
    WrapObject(),
   _innerRadius(_innerRadiusProp.getValueDbl()),
   _outerRadius(_outerRadiusProp.getValueDbl())
{
    setNull();
    setupProperties();
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
WrapTorus::~WrapTorus()
{
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aWrapTorus WrapTorus to be copied.
 */
WrapTorus::WrapTorus(const WrapTorus& aWrapTorus) :
    WrapObject(aWrapTorus),
   _innerRadius(_innerRadiusProp.getValueDbl()),
   _outerRadius(_outerRadiusProp.getValueDbl())
{
    setNull();
    setupProperties();
    copyData(aWrapTorus);
}


//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the data members of this WrapTorus to their null values.
 */
void WrapTorus::setNull()
{
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void WrapTorus::setupProperties()
{
    // BASE CLASS
    //WrapObject::setupProperties();

    _innerRadiusProp.setName("inner_radius");
    _innerRadiusProp.setValue(-1.0);
    _propertySet.append(&_innerRadiusProp);

    _outerRadiusProp.setName("outer_radius");
    _outerRadiusProp.setValue(-1.0);
    _propertySet.append(&_outerRadiusProp);
}

//_____________________________________________________________________________
/**
 * Scale the torus's dimensions. The base class scales the origin
 * of the torus in the body's reference frame.
 *
 * @param aScaleFactors The XYZ scale factors.
 */
void WrapTorus::scale(const SimTK::Vec3& aScaleFactors)
{
   // Base class, to scale origin in body frame
   WrapObject::scale(aScaleFactors);

    SimTK::Vec3 localScaleVector[2]; // only need X and Y for torus

   // _pose.x() holds the torus's X axis expressed in the
   // body's reference frame, and _pose.y() holds the Y.
   // Multiplying these vectors by the scale factor vector gives
    // localScaleVector[]. The magnitudes of the localScaleVectors
    // gives the amount to scale the torus in the XYZ dimensions.
    // The wrap torus is oriented along the Z axis, so the inner and
    // outer radii are scaled by the average of the X and Y scale factors.
    for (int i=0; i<3; i++) {
        localScaleVector[0][i] = _pose.x()[i] * aScaleFactors[i];
        localScaleVector[1][i] = _pose.y()[i] * aScaleFactors[i];
    }

   double averageXYScale = (localScaleVector[0].norm() + localScaleVector[1].norm()) * 0.5;
   _innerRadius *= averageXYScale;
   _outerRadius *= averageXYScale;
}

//_____________________________________________________________________________
/**
 * Perform some set up functions that happen after the
 * object has been deserialized or copied.
 *
 * @param aModel pointer to OpenSim Model 
 */
void WrapTorus::connectToModelAndBody(Model& aModel, PhysicalFrame& aBody)
{
    // Base class
    Super::connectToModelAndBody(aModel, aBody);

   // maybe set a parent pointer, _body = aBody;

    if (_innerRadius < 0.0)
    {
        string errorMessage = "Error: inner_radius for WrapTorus " + getName() + " was either not specified, or is negative.";
        throw Exception(errorMessage);
    }

    if (_outerRadius <= _innerRadius)
    {
        string errorMessage = "Error: outer_radius for WrapTorus " + getName() + " is less than or equal to inner_radius.";
        throw Exception(errorMessage);
    }
/*  Torus* torus = new Torus(_innerRadius, (_outerRadius-_innerRadius));
    setGeometryQuadrants(torus);
*/
}

//_____________________________________________________________________________
/**
 * Copy data members from one WrapTorus to another.
 *
 * @param aWrapTorus WrapTorus to be copied.
 */
void WrapTorus::copyData(const WrapTorus& aWrapTorus)
{
    _innerRadius = aWrapTorus._innerRadius;
    _outerRadius = aWrapTorus._outerRadius;
}

//_____________________________________________________________________________
/**
 * Get the name of the type of wrap object ("torus" in this case)
 *
 * @return A string representing the type of wrap object
 */
const char* WrapTorus::getWrapTypeName() const
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
string WrapTorus::getDimensionsString() const
{
    stringstream dimensions;
    dimensions << "radius " << _innerRadius << " " << _outerRadius;

    return dimensions.str();
}
//_____________________________________________________________________________
/**
 * Get the inner radius of the torus
 *
 * @return A Read containing the inner radius of the torus
 */
SimTK::Real WrapTorus::getInnerRadius() const
{
    return SimTK::Real(_innerRadius);
}
//_____________________________________________________________________________
/**
 * Get the outer radius of the torus
 *
 * @return A Read containing the outer radius of the torus
 */
SimTK::Real WrapTorus::getOuterRadius() const
{
    return SimTK::Real(_outerRadius);
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
WrapTorus& WrapTorus::operator=(const WrapTorus& aWrapTorus)
{
    // BASE CLASS
    WrapObject::operator=(aWrapTorus);

    return(*this);
}

//=============================================================================
// WRAPPING
//=============================================================================
//_____________________________________________________________________________
/**
 * Calculate the wrapping of one line segment over the torus.
 *
 * @param aPoint1 One end of the line segment
 * @param aPoint2 The other end of the line segment
 * @param aPathWrap An object holding the parameters for this line/torus pairing
 * @param aWrapResult The result of the wrapping (tangent points, etc.)
 * @param aFlag A flag for indicating errors, etc.
 * @return The status, as a WrapAction enum
 */
int WrapTorus::wrapLine(const SimTK::State& s, SimTK::Vec3& aPoint1, SimTK::Vec3& aPoint2,
                                const PathWrap& aPathWrap, WrapResult& aWrapResult, bool& aFlag) const
{
    int i;
    SimTK::Vec3 closestPt;
    //bool constrained = (bool) (_wrapSign != 0);
    //bool far_side_wrap = false;
    aFlag = true;

    if (findClosestPoint(_outerRadius, &aPoint1[0], &aPoint2[0], &closestPt[0], &closestPt[1], &closestPt[2], _wrapSign, _wrapAxis) == 0)
        return noWrap;

    // Now put a cylinder at closestPt and call the cylinder wrap code.
    WrapCylinder cyl;//(rot, trans, quadrant, body, radius, length);
    SimTK::Vec3 cylXaxis, cylYaxis, cylZaxis; // cylinder axes in torus reference frame

    cyl.set_radius(_innerRadius);
    cyl.set_length(CYL_LENGTH);
    cyl.set_quadrant("+x");

    closestPt *= -1;

    cylXaxis = closestPt;
    Mtx::Normalize(3, cylXaxis, cylXaxis);
    cylYaxis[0] = 0.0;
    cylYaxis[1] = 0.0;
    cylYaxis[2] = -1.0;
    Mtx::CrossProduct(cylXaxis, cylYaxis, cylZaxis);
    // Note: you don't need to recalculate Y as Z x X because X and Z are always in the XY plane, so Y will remain 0 0 -1.

    // cylinderToTorus is the transform from the cylinder to the torus.
    // The origin of the cylinder frame in the torus frame is closestPt.
    // closestPtCyl is along the X axis of the cylinder since cylXaxis = closestPt.
    SimTK::Transform cylinderToTorus(SimTK::Rotation(SimTK::Mat33(cylXaxis[0], cylXaxis[1], cylXaxis[2],
        cylYaxis[0], cylYaxis[1], cylYaxis[2], cylZaxis[0], cylZaxis[1], cylZaxis[2])));
    SimTK::Vec3 closestPtCyl = cylinderToTorus.shiftFrameStationToBase(closestPt);
    cylinderToTorus.setP(closestPtCyl);
    Vec3 p1 = cylinderToTorus.shiftFrameStationToBase(aPoint1);
    Vec3 p2 = cylinderToTorus.shiftFrameStationToBase(aPoint2);
    int return_code = cyl.wrapLine(s, p1, p2, aPathWrap, aWrapResult, aFlag);
   if (aFlag == true && return_code > 0) {
        aWrapResult.r1 = cylinderToTorus.shiftBaseStationToFrame(aWrapResult.r1);
        aWrapResult.r2 = cylinderToTorus.shiftBaseStationToFrame(aWrapResult.r2);
        for (i = 0; i < aWrapResult.wrap_pts.getSize(); i++)
            aWrapResult.wrap_pts.updElt(i) = cylinderToTorus.shiftBaseStationToFrame(aWrapResult.wrap_pts.get(i));
    }

    return wrapped;
}

//_____________________________________________________________________________
/**
 * Calculate the closest point on an origin-centered circle on the Z=0 plane
 * to the line between p1 and p2. This circle represents the inner axis of
 * the torus.
 *
 * @param radius The radius of the circle
 * @param p1 One end of the line
 * @param p2 The other end of the line
 * @param xc The X coordinate of the closest point
 * @param yc The Y coordinate of the closest point
 * @param zc The Z coordinate of the closest point
 * @param wrap_sign If wrap is constrained to a quadrant, the sign of the relevant axis
 * @param wrap_axis If wrap is constrained to a quadrant, the relevant axis
 * @return '1' if a closest point was found, '0' if there was an error while trying to constrain the wrap
 */
int WrapTorus::findClosestPoint(double radius, double p1[], double p2[],
                                          double* xc, double* yc, double* zc,
                                          int wrap_sign, int wrap_axis) const
{
   int info;                  // output flag
   int num_func_calls;        // number of calls to func (nfev)
   int ldfjac = 1;            // leading dimension of fjac (nres)
   int numResid = 1;
   int numQs = 1;
   double q[2], resid[2], fjac[2];            // m X n array
   CircleCallback cb;
   bool constrained = (bool) (wrap_sign != 0);
   // solution parameters
   int mode = 1, nprint = 0, max_iter = 500;
   double ftol = 1e-4, xtol = 1e-4, gtol = 0.0;
   double epsfcn = 0.0, step_factor = 0.2;
   // work arrays
   int ipvt[2];  
   double diag[2], qtf[2], wa1[2], wa2[2], wa3[2], wa4[2];
   // Circle variables
   double u, mag, nx, ny, nz, x, y, z, a1[3], a2[3], distance1, distance2, betterPt = 0;

   cb.p1[0] = p1[0];
   cb.p1[1] = p1[1];
   cb.p1[2] = p1[2];
   cb.p2[0] = p2[0];
   cb.p2[1] = p2[1];
   cb.p2[2] = p2[2];
   cb.r = radius;

   q[0] = 0.0;

   lmdif_C(calcCircleResids, numResid, numQs, q, resid,
           ftol, xtol, gtol, max_iter, epsfcn, diag, mode, step_factor,
           nprint, &info, &num_func_calls, fjac, ldfjac, ipvt, qtf,
           wa1, wa2, wa3, wa4, (void*)&cb);

   u = q[0];

   mag = sqrt((p2[0]-p1[0])*(p2[0]-p1[0]) + (p2[1]-p1[1])*(p2[1]-p1[1]) + (p2[2]-p1[2])*(p2[2]-p1[2]));

   nx = (p2[0]-p1[0]) / mag;
   ny = (p2[1]-p1[1]) / mag;
   nz = (p2[2]-p1[2]) / mag;

   x = p1[0] + u * nx;
   y = p1[1] + u * ny;
   z = p1[2] + u * nz;

   // Store the result from the first pass.
   a1[0] = x;
   a1[1] = y;
   a1[2] = z;

   distance1 = sqrt(x*x + y*y + z*z + radius*radius - 2.0 * radius * sqrt(x*x + y*y));

   // Perform the second pass, switching the order of the two points.
   cb.p1[0] = p2[0];
   cb.p1[1] = p2[1];
   cb.p1[2] = p2[2];
   cb.p2[0] = p1[0];
   cb.p2[1] = p1[1];
   cb.p2[2] = p1[2];
   cb.r = radius;

   q[0] = 0.0;

   lmdif_C(calcCircleResids, numResid, numQs, q, resid,
           ftol, xtol, gtol, max_iter, epsfcn, diag, mode, step_factor,
           nprint, &info, &num_func_calls, fjac, ldfjac, ipvt, qtf,
           wa1, wa2, wa3, wa4, (void*)&cb);

   u = q[0];

   mag = sqrt((p2[0]-p1[0])*(p2[0]-p1[0]) + (p2[1]-p1[1])*(p2[1]-p1[1]) + (p2[2]-p1[2])*(p2[2]-p1[2]));

   nx = (p1[0]-p2[0]) / mag;
   ny = (p1[1]-p2[1]) / mag;
   nz = (p1[2]-p2[2]) / mag;

   x = p2[0] + u * nx;
   y = p2[1] + u * ny;
   z = p2[2] + u * nz;

   // Store the result from the second pass.
   a2[0] = x;
   a2[1] = y;
   a2[2] = z;

   distance2 = sqrt(x*x + y*y + z*z + radius*radius - 2.0 * radius * sqrt(x*x + y*y));

   // Now choose the better result from the two passes. If the circle is not
   // constrained, then just choose the one with the shortest distance. If the
   // circle is constrained, then choose the one that is on the correct half of
   // the circle. If both are on the correct half, choose the closest.
   if (constrained)
   {
      if (DSIGN(a1[wrap_axis]) == wrap_sign && DSIGN(a2[wrap_axis]) == wrap_sign)
      {
         if (distance1 < distance2)
            betterPt = 0;
         else
            betterPt = 1;
      }
      else if (DSIGN(a1[wrap_axis]) == wrap_sign)
      {
         betterPt = 0;
      }
      else if (DSIGN(a2[wrap_axis]) == wrap_sign)
      {
         betterPt = 1;
      }
      else
      {
         // no wrapping should occur
         return 0;
      }
   }
   else
   {
      if (distance1 < distance2)
         betterPt = 0;
      else
         betterPt = 1;
   }

   // a1 and a2 represent the points on the line that are closest to the circle.
   // What you need to find and return is the corresponding point on the circle.
   if (betterPt == 0)
   {
      mag = (sqrt(a1[0]*a1[0] + a1[1]*a1[1]));
      *xc = a1[0] * radius / mag;
      *yc = a1[1] * radius / mag;
      *zc = 0.0;
   }
   else
   {
      mag = (sqrt(a2[0]*a2[0] + a2[1]*a2[1]));
      *xc = a2[0] * radius / mag;
      *yc = a2[1] * radius / mag;
      *zc = 0.0;
   }

   return 1;
}

//_____________________________________________________________________________
/**
 * A utility function used by findClosestPoint. The single residual that it
 * calculates is the distance between the current point and the circle.
 *
 * @param numResid The number of residuals (1)
 * @param numQs The number of degrees of freedom (1)
 * @param q Array of values of the degrees of freedom
 * @param resid Array of residuals to be calculated
 * @param flag2 A status flag
 * @param ptr Pointer to data structure containing current point and circle radius
 */
void WrapTorus::calcCircleResids(int numResid, int numQs, double q[],
                                            double resid[], int *flag2, void *ptr)
{
   double mag, nx, ny, nz, u;
   double c2, c3, c4, c5/*, c6*/;
   CircleCallback *cb = (CircleCallback*)ptr;

   u = q[0];

   mag = sqrt((cb->p2[0]-cb->p1[0])*(cb->p2[0]-cb->p1[0]) + (cb->p2[1]-cb->p1[1])*(cb->p2[1]-cb->p1[1]) +
      (cb->p2[2]-cb->p1[2])*(cb->p2[2]-cb->p1[2]));

   nx = (cb->p2[0]-cb->p1[0]) / mag;
   ny = (cb->p2[1]-cb->p1[1]) / mag;
   nz = (cb->p2[2]-cb->p1[2]) / mag;

   c2 = 2.0 * (cb->p1[0]*nx + cb->p1[1]*ny + cb->p1[2]*nz);
   c3 = cb->p1[0]*nx + cb->p1[1]*ny;
   c4 = nx*nx + ny*ny;
   c5 = cb->p1[0]*cb->p1[0] + cb->p1[1]*cb->p1[1];
   //c6 = sqrt (u * u * c4 + 2.0 * c3 * u + c5);

   resid[0] = c2 + 2.0 * u - 2.0 * cb->r * (2.0 * c4 * u + 2.0 * c3) / sqrt (u * u * c4 + 2.0 * c3 * u + c5);
}


// Implement generateDecorations by WrapTorus to replace the previous out of place implementation 
// in ModelVisualizer
void WrapTorus::generateDecorations(bool fixed, const ModelDisplayHints& hints, const SimTK::State& state,
    SimTK::Array_<SimTK::DecorativeGeometry>& appendToThis) const {
    int x = 0;
}
