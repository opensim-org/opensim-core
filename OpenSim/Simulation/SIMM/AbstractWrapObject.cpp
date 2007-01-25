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

	setupQuadrant();

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

//_____________________________________________________________________________
/**
 * Set the name of the quadrant, and call setupQuadrant() to determine the
 * appropriate values of _quadrant, _wrapAxis, and _wrapSign.
 *
 * @param aName The name of the quadrant (e.g., "+x", "-y").
 */
void AbstractWrapObject::setQuadrantName(string& aName)
{
	_quadrantName = aName;

	setupQuadrant();
}

//_____________________________________________________________________________
/**
 * Determine the appropriate values of _quadrant, _wrapAxis, and _wrapSign,
 * based on the name of the quadrant. This should be called in setup() and
 * whenever the quadrant name changes.
 */
void AbstractWrapObject::setupQuadrant()
{
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

	// Convert the muscle points from the frames of the bodies they are attached
	// to to the frame of the wrap object's body
	getBody()->getDynamicsEngine()->transformPosition(*aPoint1.getBody(), aPoint1.getAttachment(), *getBody(), pt1);
	getBody()->getDynamicsEngine()->transformPosition(*aPoint2.getBody(), aPoint2.getAttachment(), *getBody(), pt2);

	// Convert the muscle points from the frame of the wrap object's body
	// into the frame of the wrap object
	_inversePose.transformPoint(pt1);
	_inversePose.transformPoint(pt2);

	return_code = wrapLine(pt1, pt2, aMuscleWrap, aWrapResult, p_flag);

   if (p_flag == true && return_code > 0)
   {
		// Convert the tangent points from the frame of the wrap object to the
		// frame of the wrap object's body
		_pose.transformPoint(aWrapResult.r1);
		_pose.transformPoint(aWrapResult.r2);

		// Convert the surface points (between the tangent points) from the frame of
		// the wrap object to the frame of the wrap object's body
		int i;
		for (i = 0; i < aWrapResult.wrap_pts.getSize(); i++)
			_pose.transformPoint(aWrapResult.wrap_pts.get(i).get());
   }

   return return_code;
}

//=============================================================================
// UTILITY
//=============================================================================
/* Calculates the square of the shortest distance from a point (point)
 * to a line (vl, through pl).
 */
void AbstractWrapObject::rotate_matrix_axis_angle(double m[][4], const double axis[3], double angle) const
{
    double q[4];

	 rdMath::ConvertAxisAngleToQuaternion(axis, angle, q);
    rotate_matrix_by_quat(m, q);
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
   
   rdMath::ConvertAxisAngleToQuaternion(m[0], radians, q);
   rotate_matrix_by_quat(m, q);
}

void AbstractWrapObject::y_rotate_matrix_bodyfixed(double m[][4], double radians) const
{
   /* append rotation about local y-axis to matrix 'm' */
   double q[4];
   
   rdMath::ConvertAxisAngleToQuaternion(m[1], radians, q);
   rotate_matrix_by_quat(m, q);
}

void AbstractWrapObject::z_rotate_matrix_bodyfixed(double m[][4], double radians) const
{
   /* append rotation about local z-axis to matrix 'm' */
   double q[4];
   
   rdMath::ConvertAxisAngleToQuaternion(m[2], radians, q);
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
