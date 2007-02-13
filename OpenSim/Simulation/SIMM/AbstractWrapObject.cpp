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
void AbstractWrapObject::setQuadrantName(const string& aName)
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
//_____________________________________________________________________________
/**
 * Calculate the wrapping of one muscle segment over one wrap object.
 *
 * @param aPoint1 The first muscle attachment point
 * @param aPoint2 The second muscle attachment point
 * @param aMuscleWrap An object holding the parameters for this muscle/wrap-object pairing
 * @param aWrapResult The result of the wrapping (tangent points, etc.)
 * @return The status, as a WrapAction enum
 */
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
// TEST
//=============================================================================
void AbstractWrapObject::peteTest() const
{
	cout << "      xyz_body_rotation: " << _xyzBodyRotation[0] << " " << _xyzBodyRotation[1] << " " << _xyzBodyRotation[2] << endl;
	cout << "      translation: " << _translation[0] << " " << _translation[1] << " " << _translation[2] << endl;
	cout << "      active: " << _active << endl;
	cout << "      quadrant: " << _quadrantName << endl;
}
