// rdMath.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c) 2005, Stanford University. All rights reserved. 
* Redistribution and use in source and binary forms, with or without 
* modification, are permitted provided that the following conditions
* are met: 
*  - Redistributions of source code must retain the above copyright 
*    notice, this list of conditions and the following disclaimer. 
*  - Redistributions in binary form must reproduce the above copyright 
*    notice, this list of conditions and the following disclaimer in the 
*    documentation and/or other materials provided with the distribution. 
*  - Neither the name of the Stanford University nor the names of its 
*    contributors may be used to endorse or promote products derived 
*    from this software without specific prior written permission. 
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, 
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN 
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
* POSSIBILITY OF SUCH DAMAGE. 
*/

/* Note: This code was originally developed by Realistic Dynamics Inc. 
 * Author: Frank C. Anderson 
 */

//=============================================================================
// INCLUDES
//=============================================================================
#include "rdMath.h"
#include "Mtx.h"
#include "Line.h"
#include "Plane.h"


using namespace OpenSim;
using namespace std;

//=============================================================================
// EXPORTED STATIC CONSTANTS
//=============================================================================
const double rdMath::PI = acos(-1.0);
const double rdMath::PI_2 = asin(1.0);
const double rdMath::DTR = PI/180.0;
const double rdMath::RTD = 180.0/PI;
const double rdMath::SMALL = 1.0e-8;
const double rdMath::ZERO = 1.0e-14;
const double rdMath::NAN = 1.357931415e-29;
const double rdMath::INFINITY = 1.0e30;
const double rdMath::MINUS_INFINITY = -INFINITY;
const double rdMath::PLUS_INFINITY = INFINITY;


//=============================================================================
// ARITHMATIC
//=============================================================================
//_____________________________________________________________________________
/**
  * Return the magnitude of aMag with the sign of aSign.
 *
  * Note that if aSign has a value of 0.0, or aMag has a value of 0.0,
 * the sign of aMag is not changed.
 *
  * @param aMag   Magnitude
 * @param aSign  Sign
 * @return       Value with size of aMag and sign of aSign.
 */
double rdMath::
CopySign(double aMag,double aSign)
{
	if(aMag==0.0) {
		return(0.0);
	} else if(aSign==0.0) {
		return(aMag);
	} else if((aMag<0.0)&&(aSign<0.0)) {
		return(aMag);
	} else if((aMag>0.0)&&(aSign>0.0)) {
		return(aMag);
	} else {
		return(-1.0*aMag);
	}
}
//_____________________________________________________________________________
/**
 * Return wheter or not two values are equal to within a specified
 * tolerance.
 *
 * @param aValue1 Value 1.
 * @param aValue2 Value 2.
 * @param aTol Equality tolerance.
 * @return True if the absolute value of the difference between value 1 and 2
 * is less then or equal to aTol, false if not.
 */
bool rdMath::
IsEqual(double aValue1,double aValue2,double aTol)
{
	double diff = fabs(aValue1 - aValue2);
	return(diff<=aTol);
}
//_____________________________________________________________________________
/**
 * Return wheter or not an argument is closer to zero than the constant
 * rdMath::ZERO.
 *
 * @param aMag   Magnitude
 * @param aSign  Sign
 * @return       Value with size of aMag and sign of aSign.
 */
bool rdMath::
IsZero(double aValue)
{
	return( (aValue > -rdMath::ZERO) && (aValue < rdMath::ZERO) );
}

//=============================================================================
// EXPONENTIALS
//=============================================================================
//_____________________________________________________________________________
/**
 * A smooth step-up function using exponentials.
 *
 * @param tau  Rise and fall time constant.
 * @param to   Location of the midpoint of the step.
 * @param t    Independent variable
 */
double rdMath::
SigmaUp(double tau,double to,double t)
{
	return(  1.0 / (1.0 + exp(-(t-to)/tau)) );
}
//_____________________________________________________________________________
/**
 * A smooth step-down function using exponentials.
 *
 * @param tau  Rise and fall time constant.
 * @param to   Location of the midpoint of the step.
 * @param t    Independent variable
 */
double rdMath::
SigmaDn(double tau,double to,double t)
{
	return(1.0 -  1.0 / (1.0 + exp(-(t-to)/tau)) );
}


//=============================================================================
// FITTING EQUATIONS
//=============================================================================
//_____________________________________________________________________________
/**
 * Fit a parabola of the form y = c0 + c1*x + c2*x^2 to 
 * three points (x1,y1), (x2,y2), (x3,y3).
 *
 * It is required that x3 > x2 > x1.  If this condition is not met,
 * -1 is returned.  Otherwise, 0 is returned.
 *
 * To imporove numerical accuracy, the independent variables are mapped to a
 * new set of variables such that x1 -> 1.0, x3 -> 2.0, and x2 is
 * mapped to some number in between.
 */
int rdMath::
FitParabola(double aX1,double aY1,double aX2,double aY2,double aX3,double aY3,
		double *rC0,double *rC1,double *rC2)
{
	// MAPPING CONSTANTS
	double u = 2.0*aX1-aX3;
	double v = aX3-aX1;  if(v==0.0) return(-1);
	double rv = 1.0/v;

	// MAP INTO NEW VARIABLE SPACE
	double x1 = 1.0;
	double x2 = rv*(aX2-u);
	double x3 = 2.0;

	// COMPUTE THE NEGATIVE OF THE DETERMINANT
	double det = -(x1-x2)*(x2-x3)*(x3-x1);
	if(det==0.0) return(-1);

	// C0
	double c0;
	c0 = (x3*aY1-x1*aY3)*x2*x2 + (x1*x1*aY3-x3*x3*aY1)*x2 + x1*x3*(x3-x1)*aY2;
	c0 /= det;

	// C1
	double c1;
	c1 = (aY2-aY3)*x1*x1 + x3*x3*(aY1-aY2) + x2*x2*(aY3-aY1);
	c1 /= det;

	// C2
	double c2;
	c2 = x3*(aY2-aY1) + x2*(aY1-aY3) + x1*(aY3-aY2);
	c2 /= det;

	// MAP COEFICENTS BACK TO ORIGINAL VARIABLE SPACE
	*rC0 = c0 - rv*u*c1 + rv*rv*u*u*c2;
	*rC1 = rv*c1 - 2.0*rv*rv*u*c2;
	*rC2 = rv*rv*c2;

	return(0);
}

//_____________________________________________________________________________
/**
 * Linearly interpolate or extrapolate given two points.
 *
 * @param aX1 X coordinate of point 1.
 * @param aY1 Y coordinate of point 1.
 * @param aX2 X coordinate of point 2.
 * @param aY2 Y coordinate of point 2.
 * @param aX X coordinate whose corresponding Y coordinate is desired.
 * @return Y value corresponding to aX.
 */
double rdMath::
Interpolate(double aX1,double aY1,double aX2,double aY2,double aX)
{
	double y;
	double dx = aX2 - aX1;
	if(fabs(dx)<rdMath::ZERO) {
		y = aY1;
	} else {
		double dy = aY2 - aY1;
		double m = dy / dx;
		y = aY1 + m*(aX-aX1);
	}
	return(y);
}


//=============================================================================
// GEOMETRY
//=============================================================================
//_____________________________________________________________________________
/**
 * Compute the intersection of a line with a plane.
 * The line and plane can either intersect at a point, not intersect at all,
 * or be coincident.
 *
 * @param aLine Line.
 * @param aPlane Plane.
 * @param rPoint Point of intersection.  rPoint is set to the "zero" point of
 * the line in the event that line and plane are coincident; it is unchanged
 * if the line and plane do not intersect.
 * @return -1 if the line and plane do not intersect; 0 if the line and plane
 * intersect at a point; 1 if the line is coincident with the plane.
 */
int rdMath::
ComputeIntersection(const Line *aLine,const Plane *aPlane,double rPoint[3])
{
	if(aLine==NULL) return(-1);
	if(aPlane==NULL) return(-1);

	// DENOMINATOR- DOT OF LINE DIRECTION AND PLANE NORMAL
	double direction[3],normal[3];
	aLine->getDirection(direction);
	aPlane->getNormal(normal);
	double denom = Mtx::DotProduct(3,direction,normal);

	// NUMERATOR- DOT OF PLANE NORMAL WITH DIFFERENCE OF PLANE AND LINE POINTS
	double lo[3],po[3],lopo[3];
	aLine->getPoint(lo);
	aPlane->getPoint(po);
	Mtx::Subtract(1,3,po,lo,lopo);
	double numer = Mtx::DotProduct(3,normal,lopo);

	// INTERSECT AT A POINT
	if(!rdMath::IsZero(denom)) {
		double a = numer / denom;
		aLine->evaluate(a,rPoint);
		return(0);

	// DO INTERSECT
	} else if(!rdMath::IsZero(numer)) {
		return(-1);

	// COINCIDENT
	} else {
		aLine->getPoint(rPoint);
		return(1);
	}
}

//_____________________________________________________________________________
/**
 * Compute a normal to the plane described by three points (P1, P2, P3).
 * The normal is computed by taking the cross product:
 *
 *    n = (p2-p1) x (p3-p2) / | (p2-p1) x (p3-p2) |
 *
 * The three points should be distinct.
 *
 * @param aP1X X component of point 1..
 * @param aP1Y Y component of point 1.
 * @param aP1Z Z component of point 1.
 * @param aP2X X component of point 2.
 * @param aP2Y Y component of point 2.
 * @param aP2Z Z component of point 2.
 * @param aP3X X component of point 3.
 * @param aP3Y Y component of point 3.
 * @param aP3Z Z component of point 3.
 * @param rNormal Unit vector normal to the plane defined by P1, P2, and P3.
 */
void rdMath::
ComputeNormal(double aP1X,double aP1Y,double aP1Z,
				double aP2X,double aP2Y,double aP2Z,
				double aP3X,double aP3Y,double aP3Z,
				double rNormal[3])
{
	if(rNormal==NULL) return;

	// VECTOR FROM 1 TO 2
	double p12[3];
	p12[0] = aP2X - aP1X;
	p12[1] = aP2Y - aP1Y;
	p12[2] = aP2Z - aP1Z;

	// VECTOR FROM 2 TO 3
	double p23[3];
	p23[0] = aP3X - aP2X;
	p23[1] = aP3Y - aP2Y;
	p23[2] = aP3Z - aP2Z;

	// CROSS PRODUCT
	Mtx::CrossProduct(p12,p23,rNormal);

	// POINT
	double mag = Mtx::Normalize(3,rNormal,rNormal);

	// ERROR CHECK
	if(mag<rdMath::ZERO) {
		printf("Plane.ComputeNormal: WARN- normal is zero or very small.\n");
		printf("\tPoints may not have been distinct.\n");
	}
}


