// BodyScale.cpp
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
#include "BodyScale.h"

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
BodyScale::BodyScale() :
	_axisNames(_axisNamesProp.getValueStrArray())
{
	setNull();
	setupProperties();
}
//_____________________________________________________________________________
/**
 * Constructor from an XML node
 */
BodyScale::BodyScale(DOMElement *aElement) :
   Object(aElement),
	_axisNames(_axisNamesProp.getValueStrArray())
{
	setNull();
	setupProperties();
	updateFromXMLNode();
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
BodyScale::~BodyScale()
{
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aBodyScale BodyScale to be copied.
 */
BodyScale::BodyScale(const BodyScale &aBodyScale) :
   Object(aBodyScale),
	_axisNames(_axisNamesProp.getValueStrArray())
{
	setNull();
	setupProperties();
	copyData(aBodyScale);
}

//_____________________________________________________________________________
/**
 * Copy this BodyScale and return a pointer to the copy.
 * The copy constructor for this class is used.
 *
 * @return Pointer to a copy of this BodyScale.
 */
Object* BodyScale::copy() const
{
	BodyScale *bodyScale = new BodyScale(*this);
	return(bodyScale);
}

//_____________________________________________________________________________
/**
 * Copy this BodyScale and modify the copy so that it is consistent
 * with a specified XML element node.
 *
 * The copy is constructed by first using
 * BodyScale::BodyScale(DOMElement*) in order to establish the
 * relationship of the BodyScale object with the XML node. Then, the
 * assignment operator is used to set all data members of the copy to the
 * values of this BodyScale object. Finally, the data members of the copy are
 * updated using BodyScale::updateFromXMLNode().
 *
 * @param aElement XML element. 
 * @return Pointer to a copy of this BodyScale.
 */
Object* BodyScale::copy(DOMElement *aElement) const
{
	BodyScale *bodyScale = new BodyScale(aElement);
	*bodyScale = *this;
	bodyScale->updateFromXMLNode();
	return(bodyScale);
}

//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Copy data members from one BodyScale to another.
 *
 * @param aBodyScale BodyScale to be copied.
 */
void BodyScale::copyData(const BodyScale &aBodyScale)
{
	_axisNames = aBodyScale._axisNames;
}

//_____________________________________________________________________________
/**
 * Set the data members of this BodyScale to their null values.
 */
void BodyScale::setNull()
{
	setType("BodyScale");
}
//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void BodyScale::setupProperties()
{
	_axisNamesProp.setComment("Axes (X Y Z) along which to scale a body. "
		"For example, 'X Y Z' scales along all three axes, and 'Y' scales "
		"just along the Y axis.");
	_axisNamesProp.setName("axes");
	_propertySet.append(&_axisNamesProp);
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
BodyScale& BodyScale::operator=(const BodyScale &aBodyScale)
{
	// BASE CLASS
	Object::operator=(aBodyScale);

	copyData(aBodyScale);

	return(*this);
}

void BodyScale::peteTest() const
{
	cout << "         BodyScale: " << getName() << endl;
	cout << "            _axes: " << _axisNames << endl;
}

