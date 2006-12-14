// WrapSphere.cpp
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
#include "WrapSphere.h"

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
WrapSphere::WrapSphere() :
	AbstractWrapObject(),
   _radius(_radiusProp.getValueDbl())
{
	setNull();
	setupProperties();
}

//_____________________________________________________________________________
/**
 * Constructor from an XML node
 */
WrapSphere::WrapSphere(DOMElement* aElement) :
	AbstractWrapObject(aElement),
   _radius(_radiusProp.getValueDbl())
{
	setNull();
	setupProperties();
	updateFromXMLNode();
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
	AbstractWrapObject(aWrapSphere),
   _radius(_radiusProp.getValueDbl())
{
	setNull();
	setupProperties();
	copyData(aWrapSphere);
}

//_____________________________________________________________________________
/**
 * Copy this WrapSphere and return a pointer to the copy.
 * The copy constructor for this class is used.
 *
 * @return Pointer to a copy of this WrapSphere.
 */
Object* WrapSphere::copy() const
{
	WrapSphere *wrapSPhere = new WrapSphere(*this);
	return(wrapSPhere);
}

//_____________________________________________________________________________
/**
 * Copy this WrapSphere and modify the copy so that it is consistent
 * with a specified XML element node.
 *
 * The copy is constructed by first using
 * WrapSphere::WrapSphere(DOMElement*) in order to establish the
 * relationship of the WrapSphere object with the XML node. Then, the
 * assignment operator is used to set all data members of the copy to the
 * values of this WrapSphere object. Finally, the data members of the copy are
 * updated using WrapSphere::updateFromXMLNode().
 *
 * @param aElement XML element. 
 * @return Pointer to a copy of this WrapSphere.
 */
Object* WrapSphere::copy(DOMElement *aElement) const
{
	WrapSphere *wrapSPhere = new WrapSphere(aElement);
	*wrapSPhere = *this;
	wrapSPhere->updateFromXMLNode();
	return(wrapSPhere);
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
	setType("WrapSphere");
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void WrapSphere::setupProperties()
{
	// BASE CLASS
	AbstractWrapObject::setupProperties();

	_radiusProp.setName("radius");
	_radiusProp.setValue(-1.0);
	_propertySet.append(&_radiusProp);
}

//_____________________________________________________________________________
/**
 * Perform some set up functions that happen after the
 * object has been deserialized or copied.
 *
 * @param aEngine dynamics engine containing this SimmBody.
 */
void WrapSphere::setup(AbstractDynamicsEngine* aEngine, AbstractBody* aBody)
{
	// Base class
	AbstractWrapObject::setup(aEngine, aBody);

   // maybe set a parent pointer, _body = aBody;

	if (_radius < 0.0)
	{
		string errorMessage = "Error: radius for wrapSphere " + getName() + " was either not specified, or is negative.";
		throw Exception(errorMessage);
	}
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
	AbstractWrapObject::copyData(aWrapSphere);

	_radius = aWrapSphere._radius;
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
	AbstractWrapObject::operator=(aWrapSphere);

	return(*this);
}

//=============================================================================
// TEST
//=============================================================================
void WrapSphere::peteTest() const
{
	cout << "   Sphere Wrap Object " << getName() << endl;

	AbstractWrapObject::peteTest();

	cout << "      radius: " << _radius << endl;
}
