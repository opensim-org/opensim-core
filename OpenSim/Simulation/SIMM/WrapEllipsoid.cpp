// WrapEllipsoid.cpp
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
#include "WrapEllipsoid.h"

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
WrapEllipsoid::WrapEllipsoid() :
	AbstractWrapObject(),
   _dimensions(_dimensionsProp.getValueDblArray()),
	_methodName(_methodNameProp.getValueStr())
{
	setNull();
	setupProperties();
}

//_____________________________________________________________________________
/**
 * Constructor from an XML node
 */
WrapEllipsoid::WrapEllipsoid(DOMElement* aElement) :
	AbstractWrapObject(aElement),
   _dimensions(_dimensionsProp.getValueDblArray()),
	_methodName(_methodNameProp.getValueStr())
{
	setNull();
	setupProperties();
	updateFromXMLNode();
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
WrapEllipsoid::~WrapEllipsoid()
{
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aWrapEllipsoid WrapEllipsoid to be copied.
 */
WrapEllipsoid::WrapEllipsoid(const WrapEllipsoid& aWrapEllipsoid) :
	AbstractWrapObject(aWrapEllipsoid),
   _dimensions(_dimensionsProp.getValueDblArray()),
	_methodName(_methodNameProp.getValueStr())
{
	setNull();
	setupProperties();
	copyData(aWrapEllipsoid);
}

//_____________________________________________________________________________
/**
 * Copy this WrapEllipsoid and return a pointer to the copy.
 * The copy constructor for this class is used.
 *
 * @return Pointer to a copy of this WrapEllipsoid.
 */
Object* WrapEllipsoid::copy() const
{
	WrapEllipsoid *wrapEllipsoid = new WrapEllipsoid(*this);
	return(wrapEllipsoid);
}

//_____________________________________________________________________________
/**
 * Copy this WrapEllipsoid and modify the copy so that it is consistent
 * with a specified XML element node.
 *
 * The copy is constructed by first using
 * WrapEllipsoid::WrapEllipsoid(DOMElement*) in order to establish the
 * relationship of the WrapEllipsoid object with the XML node. Then, the
 * assignment operator is used to set all data members of the copy to the
 * values of this WrapEllipsoid object. Finally, the data members of the copy are
 * updated using WrapEllipsoid::updateFromXMLNode().
 *
 * @param aElement XML element. 
 * @return Pointer to a copy of this WrapEllipsoid.
 */
Object* WrapEllipsoid::copy(DOMElement *aElement) const
{
	WrapEllipsoid *wrapEllipsoid = new WrapEllipsoid(aElement);
	*wrapEllipsoid = *this;
	wrapEllipsoid->updateFromXMLNode();
	return(wrapEllipsoid);
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the data members of this WrapEllipsoid to their null values.
 */
void WrapEllipsoid::setNull()
{
	setType("WrapEllipsoid");

	_method = hybrid;
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void WrapEllipsoid::setupProperties()
{
	// BASE CLASS
	AbstractWrapObject::setupProperties();

	const double defaultDimensions[] = {-1.0, -1.0, -1.0};
	_dimensionsProp.setName("dimensions");
	_dimensionsProp.setValue(3, defaultDimensions);
	_propertySet.append(&_dimensionsProp);

	_methodNameProp.setName("method");
	_methodNameProp.setValue("unassigned");
	_propertySet.append(&_methodNameProp);
}

//_____________________________________________________________________________
/**
 * Perform some set up functions that happen after the
 * object has been deserialized or copied.
 *
 * @param aEngine dynamics engine containing this SimmBody.
 */
void WrapEllipsoid::setup(AbstractDynamicsEngine* aEngine, AbstractBody* aBody)
{
	// Base class
	AbstractWrapObject::setup(aEngine, aBody);

   // maybe set a parent pointer, _body = aBody;

	if (_dimensions[0] < 0.0 || _dimensions[1] < 0.0 || _dimensions[2] < 0.0)
	{
		string errorMessage = "Error: dimensions for WrapEllipsoid " + getName() + " were either not specified, or are negative.";
		throw Exception(errorMessage);
	}

	if (_methodName == "hybrid" || _methodName == "Hybrid" || _methodName == "HYBRID")
		_method = hybrid;
	else if (_methodName == "midpoint" || _methodName == "Midpoint" || _methodName == "MIDPOINT")
		_method = midpoint;
	else if (_methodName == "axial" || _methodName == "Axial" || _methodName == "AXIAL")
		_method = axial;
	else if (_methodName == "unassigned") {  // method was not specified in wrap object definition; use default
		_method = hybrid;
		_methodName = "hybrid";
	} else {  // method was specified incorrectly in wrap object definition; throw an exception
		string errorMessage = "Error: wrapping method for ellipsoid wrap object " + getName() + " was either not specified, or specified incorrectly.";
		throw Exception(errorMessage);
	}
}

//_____________________________________________________________________________
/**
 * Copy data members from one WrapEllipsoid to another.
 *
 * @param aWrapEllipsoid WrapEllipsoid to be copied.
 */
void WrapEllipsoid::copyData(const WrapEllipsoid& aWrapEllipsoid)
{
	// BASE CLASS
	AbstractWrapObject::copyData(aWrapEllipsoid);

	_dimensions = aWrapEllipsoid._dimensions;
	_methodName = aWrapEllipsoid._methodName;
	_method = aWrapEllipsoid._method;
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
WrapEllipsoid& WrapEllipsoid::operator=(const WrapEllipsoid& aWrapEllipsoid)
{
	// BASE CLASS
	AbstractWrapObject::operator=(aWrapEllipsoid);

	return(*this);
}

//=============================================================================
// TEST
//=============================================================================
void WrapEllipsoid::peteTest() const
{
	cout << "   Ellipsoid Wrap Object " << getName() << endl;

	AbstractWrapObject::peteTest();

	cout << "      dimensions: " << _dimensions[0] << " " << _dimensions[1] << " " << _dimensions[2] << endl;
	cout << "      method: " << _methodName << endl;
}
