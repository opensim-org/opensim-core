// WrapCylinder.cpp
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
#include "WrapCylinder.h"

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
WrapCylinder::WrapCylinder() :
	AbstractWrapObject(),
   _radius(_radiusProp.getValueDbl()),
   _height(_heightProp.getValueDbl())
{
	setNull();
	setupProperties();
}

//_____________________________________________________________________________
/**
 * Constructor from an XML node
 */
WrapCylinder::WrapCylinder(DOMElement* aElement) :
	AbstractWrapObject(aElement),
   _radius(_radiusProp.getValueDbl()),
   _height(_heightProp.getValueDbl())
{
	setNull();
	setupProperties();
	updateFromXMLNode();
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
WrapCylinder::~WrapCylinder()
{
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aWrapCylinder WrapCylinder to be copied.
 */
WrapCylinder::WrapCylinder(const WrapCylinder& aWrapCylinder) :
	AbstractWrapObject(aWrapCylinder),
   _radius(_radiusProp.getValueDbl()),
   _height(_heightProp.getValueDbl())
{
	setNull();
	setupProperties();
	copyData(aWrapCylinder);
}

//_____________________________________________________________________________
/**
 * Copy this WrapCylinder and return a pointer to the copy.
 * The copy constructor for this class is used.
 *
 * @return Pointer to a copy of this WrapCylinder.
 */
Object* WrapCylinder::copy() const
{
	WrapCylinder *wrapCylinder = new WrapCylinder(*this);
	return(wrapCylinder);
}

//_____________________________________________________________________________
/**
 * Copy this WrapCylinder and modify the copy so that it is consistent
 * with a specified XML element node.
 *
 * The copy is constructed by first using
 * WrapCylinder::WrapCylinder(DOMElement*) in order to establish the
 * relationship of the WrapCylinder object with the XML node. Then, the
 * assignment operator is used to set all data members of the copy to the
 * values of this WrapCylinder object. Finally, the data members of the copy are
 * updated using WrapCylinder::updateFromXMLNode().
 *
 * @param aElement XML element. 
 * @return Pointer to a copy of this WrapCylinder.
 */
Object* WrapCylinder::copy(DOMElement *aElement) const
{
	WrapCylinder *wrapCylinder = new WrapCylinder(aElement);
	*wrapCylinder = *this;
	wrapCylinder->updateFromXMLNode();
	return(wrapCylinder);
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the data members of this WrapCylinder to their null values.
 */
void WrapCylinder::setNull()
{
	setType("WrapCylinder");
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void WrapCylinder::setupProperties()
{
	// BASE CLASS
	AbstractWrapObject::setupProperties();

	_radiusProp.setName("radius");
	_radiusProp.setValue(-1.0);
	_propertySet.append(&_radiusProp);

	_heightProp.setName("height");
	_heightProp.setValue(1.0);
	_propertySet.append(&_heightProp);
}

//_____________________________________________________________________________
/**
 * Perform some set up functions that happen after the
 * object has been deserialized or copied.
 *
 * @param aEngine dynamics engine containing this SimmBody.
 */
void WrapCylinder::setup(AbstractDynamicsEngine* aEngine, AbstractBody* aBody)
{
	// Base class
	AbstractWrapObject::setup(aEngine, aBody);

   // maybe set a parent pointer, _body = aBody;

	if (_radius < 0.0)
	{
		string errorMessage = "Error: radius for WrapCylinder " + getName() + " was either not specified, or is negative.";
		throw Exception(errorMessage);
	}
}

//_____________________________________________________________________________
/**
 * Copy data members from one WrapCylinder to another.
 *
 * @param aWrapCylinder WrapCylinder to be copied.
 */
void WrapCylinder::copyData(const WrapCylinder& aWrapCylinder)
{
	// BASE CLASS
	AbstractWrapObject::copyData(aWrapCylinder);

	_radius = aWrapCylinder._radius;
	_height = aWrapCylinder._height;
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
WrapCylinder& WrapCylinder::operator=(const WrapCylinder& aWrapCylinder)
{
	// BASE CLASS
	AbstractWrapObject::operator=(aWrapCylinder);

	return(*this);
}

//=============================================================================
// TEST
//=============================================================================
void WrapCylinder::peteTest() const
{
	cout << "   Cylinder Wrap Object " << getName() << endl;

	AbstractWrapObject::peteTest();

	cout << "      radius: " << _radius << endl;
	cout << "      height: " << _height << endl;
}
