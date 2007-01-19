// AbstractWrapObject.cpp
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
#include "AbstractWrapObject.h"
#include "AbstractDynamicsEngine.h"
#include "AbstractBody.h"
#include "SimmMusclePoint.h"
#include "WrapResult.h"
#include "SimmMacros.h"
#include <OpenSim/Tools/rdMath.h>
#include <OpenSim/Tools/Mtx.h>

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace OpenSim;
#define LINE_EPSILON 0.00001

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */
AbstractWrapObject::AbstractWrapObject() :
	Object(),
   _xyzBodyRotation(_xyzBodyRotationProp.getValueDblArray()),
   _translation(_translationProp.getValueDblArray()),
	_active(_activeProp.getValueBool()),
	_quadrantName(_quadrantNameProp.getValueStr())
{
	setNull();
	setupProperties();
}

//_____________________________________________________________________________
/**
 * Constructor from an XML node
 */
AbstractWrapObject::AbstractWrapObject(DOMElement* aElement) :
	Object(aElement),
   _xyzBodyRotation(_xyzBodyRotationProp.getValueDblArray()),
   _translation(_translationProp.getValueDblArray()),
	_active(_activeProp.getValueBool()),
	_quadrantName(_quadrantNameProp.getValueStr())
{
	setNull();
	setupProperties();
	updateFromXMLNode();
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
AbstractWrapObject::~AbstractWrapObject()
{
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aWrapObject AbstractWrapObject to be copied.
 */
AbstractWrapObject::AbstractWrapObject(const AbstractWrapObject& aWrapObject) :
	Object(aWrapObject),
   _xyzBodyRotation(_xyzBodyRotationProp.getValueDblArray()),
   _translation(_translationProp.getValueDblArray()),
	_active(_activeProp.getValueBool()),
	_quadrantName(_quadrantNameProp.getValueStr())
{
	setNull();
	setupProperties();
	copyData(aWrapObject);
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the data members of this AbstractWrapObject to their null values.
 */
void AbstractWrapObject::setNull()
{
	setType("AbstractWrapObject");

	_quadrant = allQuadrants;
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void AbstractWrapObject::setupProperties()
{
	const double defaultRotations[] = {0.0, 0.0, 0.0};
	_xyzBodyRotationProp.setName("xyz_body_rotation");
	_xyzBodyRotationProp.setValue(3, defaultRotations);
	_propertySet.append(&_xyzBodyRotationProp);

	const double defaultTranslations[] = {0.0, 0.0, 0.0};
	_translationProp.setName("translation");
	_translationProp.setValue(3, defaultTranslations);
	_propertySet.append(&_translationProp);

	_activeProp.setName("active");
	_activeProp.setValue(true);
	_propertySet.append(&_activeProp);

	_quadrantNameProp.setName("quadrant");
	_quadrantNameProp.setValue("Unassigned");
	_propertySet.append(&_quadrantNameProp);
}

//_____________________________________________________________________________
/**
 * Perform some set up functions that happen after the
 * object has been deserialized or copied.
 *
 * @param aEngine dynamics engine containing this wrap object.
 * @param aBody body containing this wrap object.
 */
void AbstractWrapObject::setup(AbstractDynamicsEngine* aEngine, AbstractBody* aBody)
{
   _body = aBody;

	if (_quadrantName == "-x" || _quadrantName == "-X") {
		_quadrant = negativeX;
		_wrapAxis = 0;
		_wrapSign = -1;
	} else if (_quadrantName == "x" || _quadrantName == "+x" || _quadrantName == "X" || _quadrantName == "+X") {
		_quadrant = positiveX;
		_wrapAxis = 0;
		_wrapSign = 1;
	} else if (_quadrantName == "-y" || _quadrantName == "-Y") {
		_quadrant = negativeY;
		_wrapAxis = 1;
		_wrapSign = -1;
	} else if (_quadrantName == "y" || _quadrantName == "+y" || _quadrantName == "Y" || _quadrantName == "+Y") {
		_quadrant = positiveY;
		_wrapAxis = 1;
		_wrapSign = 1;
	} else if (_quadrantName == "-z" || _quadrantName == "-Z") {
		_quadrant = negativeZ;
		_wrapAxis = 2;
		_wrapSign = -1;
	} else if (_quadrantName == "z" || _quadrantName == "+z" || _quadrantName == "Z" || _quadrantName == "+Z") {
		_quadrant = positiveZ;
		_wrapAxis = 2;
		_wrapSign = 1;
	} else if (_quadrantName == "all" || _quadrantName == "ALL" || _quadrantName == "All") {
		_quadrant = allQuadrants;
		_wrapSign = 0;
	} else if (_quadrantName == "Unassigned") {  // quadrant was not specified in wrap object definition; use default
		_quadrant = allQuadrants;
		_quadrantName = "all";
		_wrapSign = 0;
	} else {  // quadrant was specified incorrectly in wrap object definition; throw an exception
		string errorMessage = "Error: quadrant for wrap object " + getName() + " was specified incorrectly.";
		throw Exception(errorMessage);
	}

	// Form a transform matrix containing _xyzBodyRotation and _translation
	_pose.setIdentity();
	_pose.rotateXBodyFixed(_xyzBodyRotation[0], Transform::Radians);
	_pose.rotateYBodyFixed(_xyzBodyRotation[1], Transform::Radians);
	_pose.rotateZBodyFixed(_xyzBodyRotation[2], Transform::Radians);
	_pose.translate(&_translation[0]);

	/* Invert the forward transform and store the inverse. */
	Mtx::Invert(4, _pose.getMatrix(), _inversePose.getMatrix());
}

//_____________________________________________________________________________
/**
 * Copy data members from one AbstractWrapObject to another.
 *
 * @param aWrapObject AbstractWrapObject to be copied.
 */
void AbstractWrapObject::copyData(const AbstractWrapObject& aWrapObject)
{
	_xyzBodyRotation = aWrapObject._xyzBodyRotation;
	_translation = aWrapObject._translation;
	_active = aWrapObject._active;
	_quadrantName = aWrapObject._quadrantName;
	_quadrant = aWrapObject._quadrant;
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
AbstractWrapObject& AbstractWrapObject::operator=(const AbstractWrapObject& aWrapObject)
{
	// BASE CLASS
	Object::operator=(aWrapObject);

	return(*this);
}

//=============================================================================
// WRAPPING
//=============================================================================
int AbstractWrapObject::wrapMuscleSegment(SimmMusclePoint& aPoint1, SimmMusclePoint& aPoint2,
														const MuscleWrap& aMuscleWrap, WrapResult& aWrapResult) const
{
   int return_code = noWrap;
	bool p_flag;
	Array<double> pt1(0.0, 3);
	Array<double> pt2(0.0, 3);

	//printf("original pt1 = %lf %lf %lf\n", aPoint1.getAttachment()[0], aPoint1.getAttachment()[1], aPoint1.getAttachment()[2]);
	//printf("original pt2 = %lf %lf %lf\n", aPoint2.getAttachment()[0], aPoint2.getAttachment()[1], aPoint2.getAttachment()[2]);

	// Convert the muscle points from the frames of the bodies they are attached
	// to to the frame of the wrap object's body
	getBody()->getDynamicsEngine()->transformPosition(*aPoint1.getBody(), aPoint1.getAttachment(), *getBody(), pt1);
	getBody()->getDynamicsEngine()->transformPosition(*aPoint2.getBody(), aPoint2.getAttachment(), *getBody(), pt2);

	//printf("pt1 in wrap seg frame = %lf %lf %lf\n", pt1[0], pt1[1], pt1[2]);
	//printf("pt2 in wrap seg frame = %lf %lf %lf\n", pt2[0], pt2[1], pt2[2]);

	// Convert the muscle points from the frame of the wrap object's body
	// into the frame of the wrap object
	_inversePose.transformPoint(pt1);
	_inversePose.transformPoint(pt2);

#if 0
	double mat[16];
	_inversePose.getMatrix(mat);
	printf("mat:\n");
	printf("%lf %lf %lf %lf\n", mat[0], mat[1], mat[2], mat[3]);
	printf("%lf %lf %lf %lf\n", mat[4], mat[5], mat[6], mat[7]);
	printf("%lf %lf %lf %lf\n", mat[8], mat[9], mat[10], mat[11]);
	printf("%lf %lf %lf %lf\n", mat[12], mat[13], mat[14], mat[15]);
	printf("pt1 in wrap frame = %lf %lf %lf\n", pt1[0], pt1[1], pt1[2]);
	printf("pt2 in wrap frame = %lf %lf %lf\n", pt2[0], pt2[1], pt2[2]);
#endif

	return_code = wrapLine(pt1, pt2, aMuscleWrap, aWrapResult, p_flag);

   if (p_flag == TRUE && return_code > 0)
   {
		//printf("r1 in wrap frame = %.6lf %.6lf %.6lf\n", aWrapResult.r1[0], aWrapResult.r1[1], aWrapResult.r1[2]);
		//printf("r2 in wrap frame = %.6lf %.6lf %.6lf\n", aWrapResult.r2[0], aWrapResult.r2[1], aWrapResult.r2[2]);

		// Convert the tangent points from the frame of the wrap object to the
		// frame of the wrap object's body
		_pose.transformPoint(aWrapResult.r1);
		_pose.transformPoint(aWrapResult.r2);

		//printf("r1 in wrap seg frame = %.6lf %.6lf %.6lf\n", aWrapResult.r1[0], aWrapResult.r1[1], aWrapResult.r1[2]);
		//printf("r2 in wrap seg frame = %.6lf %.6lf %.6lf\n", aWrapResult.r2[0], aWrapResult.r2[1], aWrapResult.r2[2]);
   }

   return return_code;
}

//=============================================================================
// UTILITY
//=============================================================================
/* to calculate the closest 3d point to given 3d line.
 * the line is defined as vector(vec) and a point(pt)
 * the line has not been normalized to a unit vector
 * the value hypo*(cosalpha) of a rt triangle is found out
 * to get the closest point
 */
void AbstractWrapObject::get_point_from_point_line2(double point[], double pt[], double vec[],
																	 double closest_pt[], double* t) const
{
   double v1[3], v2[3];
   double mag, mag2;

   v1[0] = point[0] - pt[0];
   v1[1] = point[1] - pt[1];
   v1[2] = point[2] - pt[2];

   v2[0] = vec[0];
   v2[1] = vec[1];
   v2[2] = vec[2];

	mag = Mtx::Normalize(3, v1, v1);
	mag2 = Mtx::Normalize(3, v2, v2);
	*t = Mtx::DotProduct(3, v1, v2) * mag;

   closest_pt[0] = pt[0] + *t * v2[0];
   closest_pt[1] = pt[1] + *t * v2[1];
   closest_pt[2] = pt[2] + *t * v2[2];
   
   *t = *t / mag2;
}

/* to calculate the closest 3d point to given 3d line.
 * the line is defined as vector(vec) and a point(pt)
 * the line has not been normalized to a unit vector
 * the value hypo*(cosalpha) of a rt triangle is found out
 * to get the closest point
 */
void AbstractWrapObject::get_point_from_point_line(double point[], double pt[],
																	double vec[], double closest_pt[]) const
{
   double v1[3], v2[3];
   double t, mag;

   v1[0] = point[0] - pt[0];
   v1[1] = point[1] - pt[1];
   v1[2] = point[2] - pt[2];

   v2[0] = vec[0];
   v2[1] = vec[1];
   v2[2] = vec[2];

   mag = Mtx::Normalize(3, v1,v1);
   Mtx::Normalize(3, v2, v2);
   t = Mtx::DotProduct(3, v1, v2) * mag;

   closest_pt[0] = pt[0] + t * v2[0];
   closest_pt[1] = pt[1] + t * v2[1];
   closest_pt[2] = pt[2] + t * v2[2];
}

void AbstractWrapObject::make_3x3_xrot_matrix(double a, double m[][3]) const
{
   m[0][0] = 1.0;
   m[0][1] = 0.0;
   m[0][2] = 0.0;

   m[1][0] = 0.0;
   m[1][1] = cos(a);
   m[1][2] = sin(a);
   
   m[2][0] = 0.0;
   m[2][1] = -sin(a);
   m[2][2] = cos(a);
}

void AbstractWrapObject::make_4x4dircos_matrix(double angle, double axis[], double mat[][4]) const
{
	Mtx::Identity(4, (double*)mat);
	Mtx::Normalize(3, axis, axis);

   double cl = cos(angle);
   double sl = sin(angle);
   double omc = 1.0 - cl;

   /* the following matrix is taken from Kane's 'Spacecraft Dynamics,' pp 6-7 */
   mat[0][0] = cl + axis[0]*axis[0]*omc;
   mat[1][0] = -axis[2]*sl + axis[0]*axis[1]*omc;
   mat[2][0] = axis[1]*sl + axis[2]*axis[0]*omc;
   mat[0][1] = axis[2]*sl + axis[0]*axis[1]*omc;
   mat[1][1] = cl + axis[1]*axis[1]*omc;
   mat[2][1] = -axis[0]*sl + axis[1]*axis[2]*omc;
   mat[0][2] = -axis[1]*sl + axis[2]*axis[0]*omc;
   mat[1][2] = axis[0]*sl + axis[1]*axis[2]*omc;
   mat[2][2] = cl + axis[2]*axis[2]*omc;
}

double AbstractWrapObject::distancesqr_between_vertices(double vertex1[], double vertex2[]) const
{
   double vec[3];

   vec[0] = vertex2[0] - vertex1[0];
   vec[1] = vertex2[1] - vertex1[1];
   vec[2] = vertex2[2] - vertex1[2];

   return vec[0]*vec[0]+vec[1]*vec[1]+vec[2]*vec[2];
}

/* Calculates the square of the shortest distance from a point (point)
 * to a line (vl, through pl).
 */
double AbstractWrapObject::get_distsqr_point_line(double point[], double pl[], double vl[]) const
{
   double ptemp[3];

   /* find the closest point on line */
   get_point_from_point_line(point,pl,vl,ptemp);

   return distancesqr_between_vertices(point,ptemp);
}

/* intersection between a line (pt1,pt2) and plane (plane,d)
** will give the intersection point (inter) and where on the line it is (t)
** ratio t should lie within 0 and 1 
*/
bool AbstractWrapObject::intersect_line_plane01(double pt1[], double pt2[], 
																double plane[], double d,
																double inter[], double* t) const
{
   double dotprod;
   double vec[3];

   MAKE_3DVECTOR(pt1,pt2,vec);
	dotprod = Mtx::DotProduct(3, vec,plane);

   if (DABS(dotprod) < LINE_EPSILON)
      return false;

   *t = (-d - plane[0]*pt1[0] - plane[1]*pt1[1] - plane[2]*pt1[2])/dotprod;

   if ((*t < -LINE_EPSILON) || (*t > 1.0 + LINE_EPSILON))
      return false;

   inter[0] = pt1[0] + ((*t) * vec[0]);
   inter[1] = pt1[1] + ((*t) * vec[1]);
   inter[2] = pt1[2] + ((*t) * vec[2]);

   return true;
}

bool AbstractWrapObject::intersect_lines(double p1[], double p2[], double p3[], double p4[],
													  double p_int1[], double* t, double p_int2[], double* s) const
{

   double mag1, mag2, cross_prod[3], denom, vec1[3], vec2[3], mat[3][3];

   vec1[0] = p2[0] - p1[0];
   vec1[1] = p2[1] - p1[1];
   vec1[2] = p2[2] - p1[2];
	mag1 = Mtx::Normalize(3, vec1, vec1);

   vec2[0] = p4[0] - p3[0];
   vec2[1] = p4[1] - p3[1];
   vec2[2] = p4[2] - p3[2];
   mag2 = Mtx::Normalize(3, vec2, vec2);

	Mtx::CrossProduct(vec1, vec2, cross_prod);

   denom = cross_prod[0]*cross_prod[0] + cross_prod[1]*cross_prod[1]
      + cross_prod[2]*cross_prod[2];

   if (EQUAL_WITHIN_ERROR(denom,0.0))
   {
		*s = *t = rdMath::NAN;
      return false;
   }

   mat[0][0] = p3[0] - p1[0];
   mat[0][1] = p3[1] - p1[1];
   mat[0][2] = p3[2] - p1[2];
   mat[1][0] = vec1[0];
   mat[1][1] = vec1[1];
   mat[1][2] = vec1[2];
   mat[2][0] = cross_prod[0];
   mat[2][1] = cross_prod[1];
   mat[2][2] = cross_prod[2];

   *s = CALC_DETERMINANT(mat) / denom;

   p_int2[0] = p3[0] + (*s) * (vec2[0]);
   p_int2[1] = p3[1] + (*s) * (vec2[1]);
   p_int2[2] = p3[2] + (*s) * (vec2[2]);

   mat[1][0] = vec2[0];
   mat[1][1] = vec2[1];
   mat[1][2] = vec2[2];

   *t = CALC_DETERMINANT(mat) / denom;

   p_int1[0] = p1[0] + (*t) * (vec1[0]);
   p_int1[1] = p1[1] + (*t) * (vec1[1]);
   p_int1[2] = p1[2] + (*t) * (vec1[2]);

   (*s) /= mag2;
   (*t) /= mag1;

   return true;

}

/* Same as intersect_lines(), but this routine does not scale the S and T
 * parameters to be between 0.0 and 1.0. S ranges from 0.0 to mag2,
 * and T ranges from 0.0 to mag1.
 */

bool AbstractWrapObject::intersect_lines_scaled(double p1[], double p2[], double p3[], double p4[],
																double p_int1[], double* t, double* mag1,
																double p_int2[], double* s, double* mag2) const
{
   double cross_prod[3], denom, vec1[3], vec2[3], mat[3][3];

   vec1[0] = p2[0] - p1[0];
   vec1[1] = p2[1] - p1[1];
   vec1[2] = p2[2] - p1[2];
   *mag1 = Mtx::Normalize(3, vec1, vec1);

   vec2[0] = p4[0] - p3[0];
   vec2[1] = p4[1] - p3[1];
   vec2[2] = p4[2] - p3[2];
   *mag2 = Mtx::Normalize(3, vec2, vec2);

	Mtx::CrossProduct(vec1, vec2, cross_prod);

   denom = cross_prod[0]*cross_prod[0] + cross_prod[1]*cross_prod[1]
      + cross_prod[2]*cross_prod[2];

   if (EQUAL_WITHIN_ERROR(denom,0.0))
   {
		*s = *t = rdMath::NAN;
      return false;
   }

   mat[0][0] = p3[0] - p1[0];
   mat[0][1] = p3[1] - p1[1];
   mat[0][2] = p3[2] - p1[2];
   mat[1][0] = vec1[0];
   mat[1][1] = vec1[1];
   mat[1][2] = vec1[2];
   mat[2][0] = cross_prod[0];
   mat[2][1] = cross_prod[1];
   mat[2][2] = cross_prod[2];

   *s = CALC_DETERMINANT(mat) / denom;

   p_int2[0] = p3[0] + (*s) * (vec2[0]);
   p_int2[1] = p3[1] + (*s) * (vec2[1]);
   p_int2[2] = p3[2] + (*s) * (vec2[2]);

   mat[1][0] = vec2[0];
   mat[1][1] = vec2[1];
   mat[1][2] = vec2[2];

   *t = CALC_DETERMINANT(mat) / denom;

   p_int1[0] = p1[0] + (*t) * (vec1[0]);
   p_int1[1] = p1[1] + (*t) * (vec1[1]);
   p_int1[2] = p1[2] + (*t) * (vec1[2]);

   return true;
}

void AbstractWrapObject::rotate_matrix_axis_angle(double m[][4], const double axis[3], double angle) const
{
    double q[4];

    make_quaternion(q, axis, angle);
    rotate_matrix_by_quat(m, q);
}

void AbstractWrapObject::make_quaternion(double q[4], const double axis[3], double angle) const
{
   /* make a quaternion given an axis-angle rotation
    * (from java-based vecmath package)
    */
   double n, halfAngle;
  
   q[0] = axis[0];
   q[1] = axis[1];
	q[2] = axis[2];

	n = sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2]);

	halfAngle = 0.5 * angle;

	if (NOT_EQUAL_WITHIN_ERROR(n,0.0))
	{
		double s = sin(halfAngle) / n;

		q[0] *= s;  q[1] *= s;  q[2] *= s;

		q[3] = cos(halfAngle);
	}
}

void AbstractWrapObject::quat_to_matrix(const double q[4], double m[][4]) const
{
	/* make a rotation matrix from a quaternion */

	double Nq = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
	double s = (Nq > 0.0) ? (2.0 / Nq) : 0.0;

	double xs = q[0] * s,   ys = q[1] * s,   zs = q[2] * s;
	double wx = q[3] * xs,  wy = q[3] * ys,  wz = q[3] * zs;
	double xx = q[0] * xs,  xy = q[0] * ys,  xz = q[0] * zs;
	double yy = q[1] * ys,  yz = q[1] * zs,  zz = q[2] * zs;

	m[0][0] = 1.0 - (yy + zz);  m[0][1] = xy + wz;          m[0][2] = xz - wy;
	m[1][0] = xy - wz;          m[1][1] = 1.0 - (xx + zz);  m[1][2] = yz + wx;
	m[2][0] = xz + wy;          m[2][1] = yz - wx;          m[2][2] = 1.0 - (xx + yy);

	m[0][3] = m[1][3] = m[2][3] = m[3][0] = m[3][1] = m[3][2] = 0.0;
	m[3][3] = 1.0;
}

void AbstractWrapObject::rotate_matrix_by_quat(double m[][4], const double q[4]) const
{
	/* append a quaternion rotation to a matrix */

	double n[4][4];

	quat_to_matrix(q, n);

	Mtx::Multiply(4, 4, 4, (double*)m, (double*)n, (double*)m); // TODO: make sure this gives same result as append_matrix()
}

void AbstractWrapObject::x_rotate_matrix_bodyfixed(double m[][4], double radians) const
{
   /* append rotation about local x-axis to matrix 'm' */
   double q[4];
   
   make_quaternion(q, m[0], radians);
   rotate_matrix_by_quat(m, q);
}

void AbstractWrapObject::y_rotate_matrix_bodyfixed(double m[][4], double radians) const
{
   /* append rotation about local y-axis to matrix 'm' */
   double q[4];
   
   make_quaternion(q, m[1], radians);
   rotate_matrix_by_quat(m, q);
}

void AbstractWrapObject::z_rotate_matrix_bodyfixed(double m[][4], double radians) const
{
   /* append rotation about local z-axis to matrix 'm' */
   double q[4];
   
   make_quaternion(q, m[2], radians);
   rotate_matrix_by_quat(m, q);
}

//=============================================================================
// TEST
//=============================================================================
void AbstractWrapObject::peteTest() const
{
	cout << "      xyz_body_rotation: " << _xyzBodyRotation[0] << " " << _xyzBodyRotation[1] << " " << _xyzBodyRotation[2] << endl;
	cout << "      translation: " << _translation[0] << " " << _translation[1] << " " << _translation[2] << endl;
	cout << "      active: " << _active << endl;
	cout << "      quadrant: " << _quadrantName << endl;
}
