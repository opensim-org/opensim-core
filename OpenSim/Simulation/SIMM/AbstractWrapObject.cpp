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
	_quadrantNameProp.setValue("unassigned");
	_propertySet.append(&_quadrantNameProp);
}

//_____________________________________________________________________________
/**
 * Perform some set up functions that happen after the
 * object has been deserialized or copied.
 *
 * @param aEngine dynamics engine containing this SimmBody.
 */
void AbstractWrapObject::setup(AbstractDynamicsEngine* aEngine, AbstractBody* aBody)
{
   // maybe set a parent pointer, _body = aBody;

	if (_quadrantName == "-x" || _quadrantName == "-X")
		_quadrant = negativeX;
	else if (_quadrantName == "x" || _quadrantName == "+x" || _quadrantName == "X" || _quadrantName == "+X")
		_quadrant = positiveX;
	else if (_quadrantName == "-y" || _quadrantName == "-Y")
		_quadrant = negativeY;
	else if (_quadrantName == "y" || _quadrantName == "+y" || _quadrantName == "Y" || _quadrantName == "+Y")
		_quadrant = positiveY;
	else if (_quadrantName == "-z" || _quadrantName == "-Z")
		_quadrant = negativeZ;
	else if (_quadrantName == "z" || _quadrantName == "+z" || _quadrantName == "Z" || _quadrantName == "+Z")
		_quadrant = positiveZ;
	else if (_quadrantName == "all" || _quadrantName == "ALL" || _quadrantName == "All")
		_quadrant = allQuadrants;
	else if (_quadrantName == "unassigned") {  // quadrant was not specified in wrap object definition; use default
		_quadrant = allQuadrants;
		_quadrantName = "all";
	} else {  // quadrant was specified incorrectly in wrap object definition; throw an exception
		string errorMessage = "Error: quadrant for wrap object " + getName() + " was specified incorrectly.";
		throw Exception(errorMessage);
	}
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
// TEST
//=============================================================================
void AbstractWrapObject::peteTest() const
{
	cout << "      xyz_body_rotation: " << _xyzBodyRotation[0] << " " << _xyzBodyRotation[1] << " " << _xyzBodyRotation[2] << endl;
	cout << "      translation: " << _translation[0] << " " << _translation[1] << " " << _translation[2] << endl;
	cout << "      active: " << _active << endl;
	cout << "      quadrant: " << _quadrantName << endl;
}
