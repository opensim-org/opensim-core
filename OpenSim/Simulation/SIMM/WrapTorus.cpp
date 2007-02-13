// WrapTorus.cpp
// Author: Peter Loan
/*
 * Copyright (c) 2006, Stanford University. All rights reserved. 
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including 
 * without limitation the rights to use, copy, modify, merge, publish, 
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject
 * to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included 
 * in all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

//=============================================================================
// INCLUDES
//=============================================================================
#include "WrapTorus.h"
#include "WrapCylinder.h"
#include "SimmMusclePoint.h"
#include "MuscleWrap.h"
#include "WrapResult.h"
#include "SimmMacros.h"
#include <OpenSim/Tools/Lmdif.h>
#include <OpenSim/Tools/Mtx.h>
#include <sstream>

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace OpenSim;

static char* wrapTypeName = "torus";

#define CYL_LENGTH 10000.0

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */
WrapTorus::WrapTorus() :
	AbstractWrapObject(),
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
	AbstractWrapObject(aWrapTorus),
   _innerRadius(_innerRadiusProp.getValueDbl()),
   _outerRadius(_outerRadiusProp.getValueDbl())
{
	setNull();
	setupProperties();
	copyData(aWrapTorus);
}

//_____________________________________________________________________________
/**
 * Copy this WrapTorus and return a pointer to the copy.
 * The copy constructor for this class is used.
 *
 * @return Pointer to a copy of this WrapTorus.
 */
Object* WrapTorus::copy() const
{
	WrapTorus *wrapTorus = new WrapTorus(*this);
	return(wrapTorus);
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
	setType("WrapTorus");
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void WrapTorus::setupProperties()
{
	// BASE CLASS
	AbstractWrapObject::setupProperties();

	_innerRadiusProp.setName("inner_radius");
	_innerRadiusProp.setValue(-1.0);
	_propertySet.append(&_innerRadiusProp);

	_outerRadiusProp.setName("outer_radius");
	_outerRadiusProp.setValue(-1.0);
	_propertySet.append(&_outerRadiusProp);
}

//_____________________________________________________________________________
/**
 * Perform some set up functions that happen after the
 * object has been deserialized or copied.
 *
 * @param aEngine dynamics engine containing this SimmBody.
 */
void WrapTorus::setup(AbstractDynamicsEngine* aEngine, AbstractBody* aBody)
{
	// Base class
	AbstractWrapObject::setup(aEngine, aBody);

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
}

//_____________________________________________________________________________
/**
 * Copy data members from one WrapTorus to another.
 *
 * @param aWrapTorus WrapTorus to be copied.
 */
void WrapTorus::copyData(const WrapTorus& aWrapTorus)
{
	// BASE CLASS
	AbstractWrapObject::copyData(aWrapTorus);

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
	AbstractWrapObject::operator=(aWrapTorus);

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
 * @param aMuscleWrap An object holding the parameters for this line/torus pairing
 * @param aWrapResult The result of the wrapping (tangent points, etc.)
 * @param aFlag A flag for indicating errors, etc.
 * @return The status, as a WrapAction enum
 */
int WrapTorus::wrapLine(Array<double>& aPoint1, Array<double>& aPoint2,
								const MuscleWrap& aMuscleWrap, WrapResult& aWrapResult, bool& aFlag) const
{
	int i;
	double closestPt[3];
	bool constrained = (bool) (_wrapSign != 0);
	bool far_side_wrap = false;
	aFlag = true;

	if (findClosestPoint(_outerRadius, &aPoint1[0], &aPoint2[0], &closestPt[0], &closestPt[1], &closestPt[2], _wrapSign, _wrapAxis) == 0)
		return noWrap;

	// Now put a cylinder at closestPt and call the cylinder wrap code.
	WrapCylinder cyl;//(rot, trans, quadrant, body, radius, length);
	double cylXaxis[3], cylYaxis[3], cylZaxis[3]; // cylinder axes in torus reference frame

	cyl.setRadius(_innerRadius);
	cyl.setLength(CYL_LENGTH);
	cyl.setQuadrantName("+x");

	for (i = 0; i < 3; i++)
		closestPt[i] = -closestPt[i];

	cylXaxis[0] = closestPt[0];
	cylXaxis[1] = closestPt[1];
	cylXaxis[2] = closestPt[2];
	Mtx::Normalize(3, cylXaxis, cylXaxis);
	cylYaxis[0] = 0.0;
	cylYaxis[1] = 0.0;
	cylYaxis[2] = -1.0;
	Mtx::CrossProduct(cylXaxis, cylYaxis, cylZaxis);

	// torusToCylinder is the transform to convert points in the
	// torus frame into the cylinder frame.
	// The origin of the cylinder frame in the torus frame is closestPt.
	Transform torusToCylinder, cylinderToTorus;
	double* mat = torusToCylinder.getMatrix();
	for (i = 0; i < 3; i++) {
		mat[i*4 + 0] = cylXaxis[i];
		mat[i*4 + 1] = cylYaxis[i];
		mat[i*4 + 2] = cylZaxis[i];
	}

	torusToCylinder.transformPoint(closestPt);
	torusToCylinder.translate(closestPt);
	Mtx::Invert(4, torusToCylinder.getMatrix(), cylinderToTorus.getMatrix());

	mat = cylinderToTorus.getMatrix();
	Array<double> p1(aPoint1), p2(aPoint2);
	torusToCylinder.transformPoint(p1);
	torusToCylinder.transformPoint(p2);

	int return_code = cyl.wrapLine(p1, p2, aMuscleWrap, aWrapResult, aFlag);

   if (aFlag == true && return_code > 0) {
		cylinderToTorus.transformPoint(aWrapResult.r1);
		cylinderToTorus.transformPoint(aWrapResult.r2);
		for (i = 0; i < aWrapResult.wrap_pts.getSize(); i++)
			cylinderToTorus.transformPoint(aWrapResult.wrap_pts.get(i).get());
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
   double c2, c3, c4, c5, c6;
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
   c6 = sqrt (u * u * c4 + 2.0 * c3 * u + c5);

   resid[0] = c2 + 2.0 * u - 2.0 * cb->r * (2.0 * c4 * u + 2.0 * c3) / sqrt (u * u * c4 + 2.0 * c3 * u + c5);
}

//=============================================================================
// TEST
//=============================================================================
void WrapTorus::peteTest() const
{
	cout << "   Torus Wrap Object " << getName() << endl;

	AbstractWrapObject::peteTest();

	cout << "      inner_radius: " << _innerRadius << endl;
	cout << "      outer_radius: " << _outerRadius << endl;
}
