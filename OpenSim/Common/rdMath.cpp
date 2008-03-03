// rdMath.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
 * Copyright (c)  2005, Stanford University. All rights reserved. 
* Use of the OpenSim software in source form is permitted provided that the following
* conditions are met:
* 	1. The software is used only for non-commercial research and education. It may not
*     be used in relation to any commercial activity.
* 	2. The software is not distributed or redistributed.  Software distribution is allowed 
*     only through https://simtk.org/home/opensim.
* 	3. Use of the OpenSim software or derivatives must be acknowledged in all publications,
*      presentations, or documents describing work in which OpenSim or derivatives are used.
* 	4. Credits to developers may not be removed from executables
*     created from modifications of the source.
* 	5. Modifications of source code must retain the above copyright notice, this list of
*     conditions and the following disclaimer. 
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
*  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
*  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
*  SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
*  TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
*  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR BUSINESS INTERRUPTION) OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
*  WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/* Note: This code was originally developed by Realistic Dynamics Inc. 
 * Author: Frank C. Anderson 
 */

//=============================================================================
// INCLUDES
//=============================================================================
#include <math.h>
#include "rdMath.h"
#include "Mtx.h"
#include "SimmMacros.h"


using namespace OpenSim;
using namespace std;
using SimTK::Vec3;
using SimTK::Mat33;

//=============================================================================
// EXPORTED STATIC CONSTANTS
//=============================================================================
const double rdMath::PI_2 = asin(1.0);
const double rdMath::SMALL = 1.0e-8;
const double rdMath::ZERO = 1.0e-14;
const double rdMath::NAN = SimTK::CNT<SimTK::Real>::getNaN();
const double rdMath::INFINITY = 1.0e30;
const double rdMath::MINUS_INFINITY = -INFINITY;
const double rdMath::PLUS_INFINITY = INFINITY;

#define LINE_EPSILON 0.00001

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
// CUBIC STEP FUNCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * A smooth step-down function using cubic polynomial.
 * x=0 for t<t0, x=1 for t>t1, and x=smooth step in between t0 and t1.
 *
 * @param t    Parameter at which to evaluate step function
 * @param t0   Parameter value at which step starts (result=0 to the left)
 * @param t1   Parameter value at which step ends (result=1 to the right)
 */
double rdMath::
Step(double t, double t0, double t1)
{
	double tn=(t-t0)/(t1-t0);
	if(tn<0) return 0;
	else if(tn>1) return 1;
	else return pow(tn,2)*(3-2*tn);
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
 *
int rdMath::
ComputeIntersection(const Line *aLine,const Plane *aPlane,double rPoint[3])
{
	if(aLine==NULL) return(-1);
	if(aPlane==NULL) return(-1);

	// DENOMINATOR- DOT OF LINE DIRECTION AND PLANE NORMAL
	SimTK::Vec3 direction,normal;
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
*/
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
				  SimTK::Vec3& rNormal)
{
	// VECTOR FROM 1 TO 2
	SimTK::Vec3 p12;
	p12[0] = aP2X - aP1X;
	p12[1] = aP2Y - aP1Y;
	p12[2] = aP2Z - aP1Z;

	// VECTOR FROM 2 TO 3
	SimTK::Vec3 p23;
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
bool rdMath::
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
		s = t = rdMath::NAN;
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
bool rdMath::
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
void rdMath::
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
void rdMath::
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
void rdMath::
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
void rdMath::
ConvertAxisAngleTo4x4DirCosMatrix(const SimTK::Vec3& axis, double angle, double mat[][4])
{
	SimTK::Vec3 normAxis;

	//Mtx::Identity(4, (double*)mat);
	SimTK::Mat44 mat44((double*)mat);
	mat44 = 1.0;
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
double rdMath::
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
double rdMath::
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
void rdMath::
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
void rdMath::
ConvertQuaternionToMatrix(const double quat[4], double matrix[][4])
{
	double Nq = quat[0] * quat[0] + quat[1] * quat[1] + quat[2] * quat[2] + quat[3] * quat[3];
	double s = (Nq > 0.0) ? (2.0 / Nq) : 0.0;

	double xs = quat[0] * s,   ys = quat[1] * s,   zs = quat[2] * s;
	double wx = quat[3] * xs,  wy = quat[3] * ys,  wz = quat[3] * zs;
	double xx = quat[0] * xs,  xy = quat[0] * ys,  xz = quat[0] * zs;
	double yy = quat[1] * ys,  yz = quat[1] * zs,  zz = quat[2] * zs;

	matrix[0][0] = 1.0 - (yy + zz);  matrix[0][1] = xy + wz;          matrix[0][2] = xz - wy;
	matrix[1][0] = xy - wz;          matrix[1][1] = 1.0 - (xx + zz);  matrix[1][2] = yz + wx;
	matrix[2][0] = xz + wy;          matrix[2][1] = yz - wx;          matrix[2][2] = 1.0 - (xx + yy);

	matrix[0][3] = matrix[1][3] = matrix[2][3] = matrix[3][0] = matrix[3][1] = matrix[3][2] = 0.0;
	matrix[3][3] = 1.0;
}

/* Rotate a 4x4 transform matrix by a quaternion.
 * @param matrix The 4x4 transform matrix
 * @param quat The quaternion
 */
void rdMath::
RotateMatrixQuaternion(double matrix[][4], const double quat[4])
{
	// append a quaternion rotation to a matrix
	double n[4][4];

	ConvertQuaternionToMatrix(quat, n);

	Mtx::Multiply(4, 4, 4, (double*)matrix, (double*)n, (double*)matrix);
}

/* Rotate a 4x4 transform matrix by 'angle' radians about the local X axis.
 * @param matrix The 4x4 transform matrix
 * @param angle The amount in radians to rotate
 */
void rdMath::
RotateMatrixXBodyFixed(double matrix[][4], double angle)
{
   // append rotation about local x-axis to matrix 'matrix'
   double quat[4];
   
   ConvertAxisAngleToQuaternion(Vec3::getAs(matrix[0]), angle, quat);
   RotateMatrixQuaternion(matrix, quat);
}

/* Rotate a 4x4 transform matrix by 'angle' radians about the local Y axis.
 * @param matrix The 4x4 transform matrix
 * @param angle The amount in radians to rotate
 */
void rdMath::
RotateMatrixYBodyFixed(double matrix[][4], double angle)
{
   // append rotation about local y-axis to matrix 'matrix'
   double quat[4];
   
   ConvertAxisAngleToQuaternion(Vec3::getAs(matrix[1]), angle, quat);
   RotateMatrixQuaternion(matrix, quat);
}

/* Rotate a 4x4 transform matrix by 'angle' radians about the local Z axis.
 * @param matrix The 4x4 transform matrix
 * @param angle The amount in radians to rotate
 */
void rdMath::
RotateMatrixZBodyFixed(double matrix[][4], double angle)
{
   // append rotation about local z-axis to matrix 'matrix'
   double quat[4];
   
   ConvertAxisAngleToQuaternion(Vec3::getAs(matrix[2]), angle, quat);
   RotateMatrixQuaternion(matrix, quat);
}

