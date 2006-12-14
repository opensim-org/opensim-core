// AbstractBody.cpp
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
#include "AbstractBody.h"
#include "AbstractDynamicsEngine.h"

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
AbstractBody::AbstractBody() :
	Object(),
	_wrapObjectSetProp(PropertyObj("", WrapObjectSet())),
	_wrapObjectSet((WrapObjectSet&)_wrapObjectSetProp.getValueObj())
{
	setNull();
	setupProperties();
}

//_____________________________________________________________________________
/**
 * Constructor from an XML node
 */
AbstractBody::AbstractBody(DOMElement *aElement) :
	Object(aElement),
	_wrapObjectSetProp(PropertyObj("", WrapObjectSet())),
	_wrapObjectSet((WrapObjectSet&)_wrapObjectSetProp.getValueObj())
{
	setNull();
	setupProperties();
	updateFromXMLNode();
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
AbstractBody::~AbstractBody()
{
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aBody AbstractBody to be copied.
 */
AbstractBody::AbstractBody(const AbstractBody &aBody) :
	Object(aBody),
	_wrapObjectSetProp(PropertyObj("", WrapObjectSet())),
	_wrapObjectSet((WrapObjectSet&)_wrapObjectSetProp.getValueObj())
{
	setNull();
	setupProperties();
	copyData(aBody);
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Copy data members from one AbstractBody to another.
 *
 * @param aBody AbstractBody to be copied.
 */
void AbstractBody::copyData(const AbstractBody &aBody)
{
	_dynamicsEngine = aBody._dynamicsEngine;
	_wrapObjectSet = aBody._wrapObjectSet;
}

/**
 * Set the data members of this AbstractBody to their null values.
 */
void AbstractBody::setNull()
{
	setType("AbstractBody");

	_dynamicsEngine = NULL;
}

//_____________________________________________________________________________
/**
 * Perform set up functions after model has been deserialized or copied.
 *
 * @param aEngine dynamics engine containing this body.
 */
void AbstractBody::setup(AbstractDynamicsEngine* aEngine)
{
	_dynamicsEngine = aEngine;

	int i;
	for (i = 0; i < _wrapObjectSet.getSize(); i++)
		_wrapObjectSet.get(i)->setup(aEngine, this);
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 *
 */
void AbstractBody::setupProperties()
{
	_wrapObjectSetProp.setName("WrapObjectSet");
	_propertySet.append(&_wrapObjectSetProp);
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
AbstractBody& AbstractBody::operator=(const AbstractBody &aBody)
{
	// BASE CLASS
	Object::operator=(aBody);

	copyData(aBody);

	return(*this);
}

//_____________________________________________________________________________
/**
 * Get the named wrap object, if it exists.
 *
 * @param aName Name of the wrap object.
 * @return Pointer to the wrap object.
 */
AbstractWrapObject* AbstractBody::getWrapObject(const string& aName)
{
	int i;

	for (i = 0; i < _wrapObjectSet.getSize(); i++)
	{
		if (aName == _wrapObjectSet.get(i)->getName())
			return _wrapObjectSet.get(i);
	}

	return NULL;
}

