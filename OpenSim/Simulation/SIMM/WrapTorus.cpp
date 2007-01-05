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
#include "SimmMusclePoint.h"
#include "MuscleWrap.h"
#include "WrapResult.h"
#include <sstream>

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace OpenSim;

static char* wrapTypeName = "torus";
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
 * Constructor from an XML node
 */
WrapTorus::WrapTorus(DOMElement* aElement) :
	AbstractWrapObject(aElement),
   _innerRadius(_innerRadiusProp.getValueDbl()),
   _outerRadius(_outerRadiusProp.getValueDbl())
{
	setNull();
	setupProperties();
	updateFromXMLNode();
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

//_____________________________________________________________________________
/**
 * Copy this WrapTorus and modify the copy so that it is consistent
 * with a specified XML element node.
 *
 * The copy is constructed by first using
 * WrapTorus::WrapTorus(DOMElement*) in order to establish the
 * relationship of the WrapTorus object with the XML node. Then, the
 * assignment operator is used to set all data members of the copy to the
 * values of this WrapTorus object. Finally, the data members of the copy are
 * updated using WrapTorus::updateFromXMLNode().
 *
 * @param aElement XML element. 
 * @return Pointer to a copy of this WrapTorus.
 */
Object* WrapTorus::copy(DOMElement *aElement) const
{
	WrapTorus *wrapTorus = new WrapTorus(aElement);
	*wrapTorus = *this;
	wrapTorus->updateFromXMLNode();
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

const char* WrapTorus::getWrapTypeName() const
{
	return wrapTypeName;
}

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
int WrapTorus::wrapLine(Array<double>& aPoint1, Array<double>& aPoint2,
								const MuscleWrap& aMuscleWrap, WrapResult& aWrapResult, bool& aFlag) const
{
	return noWrap;
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
