// MuscleWrap.cpp
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
#include "MuscleWrap.h"

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
MuscleWrap::MuscleWrap() :
	Object(),
	_wrapObjectName(_wrapObjectNameProp.getValueStr()),
	_algorithmName(_algorithmNameProp.getValueStr()),
   _range(_rangeProp.getValueIntArray())
{
	setNull();
	setupProperties();
}

//_____________________________________________________________________________
/**
 * Constructor from an XML node
 */
MuscleWrap::MuscleWrap(DOMElement* aElement) :
	Object(aElement),
	_wrapObjectName(_wrapObjectNameProp.getValueStr()),
	_algorithmName(_algorithmNameProp.getValueStr()),
   _range(_rangeProp.getValueIntArray())
{
	setNull();
	setupProperties();
	updateFromXMLNode();
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
MuscleWrap::~MuscleWrap()
{
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aMuscleWrap MuscleWrap to be copied.
 */
MuscleWrap::MuscleWrap(const MuscleWrap& aMuscleWrap) :
	Object(aMuscleWrap),
	_wrapObjectName(_wrapObjectNameProp.getValueStr()),
	_algorithmName(_algorithmNameProp.getValueStr()),
   _range(_rangeProp.getValueIntArray())
{
	setNull();
	setupProperties();
	copyData(aMuscleWrap);
}

//_____________________________________________________________________________
/**
 * Copy this MuscleWrap and return a pointer to the copy.
 * The copy constructor for this class is used.
 *
 * @return Pointer to a copy of this MuscleWrap.
 */
Object* MuscleWrap::copy() const
{
	MuscleWrap *muscWrap = new MuscleWrap(*this);
	return(muscWrap);
}

//_____________________________________________________________________________
/**
 * Copy this MuscleWrap and modify the copy so that it is consistent
 * with a specified XML element node.
 *
 * The copy is constructed by first using
 * MuscleWrap::MuscleWrap(DOMElement*) in order to establish the
 * relationship of the MuscleWrap object with the XML node. Then, the
 * assignment operator is used to set all data members of the copy to the
 * values of this MuscleWrap object. Finally, the data members of the copy are
 * updated using MuscleWrap::updateFromXMLNode().
 *
 * @param aElement XML element. 
 * @return Pointer to a copy of this MuscleWrap.
 */
Object* MuscleWrap::copy(DOMElement *aElement) const
{
	MuscleWrap *muscWrap = new MuscleWrap(aElement);
	*muscWrap = *this;
	muscWrap->updateFromXMLNode();
	return(muscWrap);
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the data members of this MuscleWrap to their null values.
 */
void MuscleWrap::setNull()
{
	setType("MuscleWrap");
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void MuscleWrap::setupProperties()
{
	_wrapObjectNameProp.setName("wrap_object");
	_propertySet.append(&_wrapObjectNameProp);

	_algorithmNameProp.setName("algorithm");
	_propertySet.append(&_algorithmNameProp);

	const int defaultRange[] = {-1, -1};
	_rangeProp.setName("range");
	_rangeProp.setValue(2, defaultRange);
	_propertySet.append(&_rangeProp);
}

//_____________________________________________________________________________
/**
 * Perform some set up functions that happen after the
 * object has been deserialized or copied.
 *
 * @param aEngine dynamics engine containing this SimmBody.
 */
void MuscleWrap::setup(AbstractDynamicsEngine* aEngine)
{
	// get pointer to wrap object
}

//_____________________________________________________________________________
/**
 * Copy data members from one MuscleWrap to another.
 *
 * @param aMuscleWrap MuscleWrap to be copied.
 */
void MuscleWrap::copyData(const MuscleWrap& aMuscleWrap)
{
	_wrapObjectName = aMuscleWrap._wrapObjectName;
	_algorithmName = aMuscleWrap._algorithmName;
	_range = aMuscleWrap._range;
	_wrapObject = aMuscleWrap._wrapObject;

	int i;
	for (i = 0; i < 3; i++) {
		_c[i] = aMuscleWrap._c[i];
		_r1[i] = aMuscleWrap._r1[i];
		_r2[i] = aMuscleWrap._r2[i];
	}

	_wrapPoints[0] = aMuscleWrap._wrapPoints[0];
	_wrapPoints[1] = aMuscleWrap._wrapPoints[1];
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
MuscleWrap& MuscleWrap::operator=(const MuscleWrap& aMuscleWrap)
{
	// BASE CLASS
	Object::operator=(aMuscleWrap);

	return(*this);
}

//=============================================================================
// TEST
//=============================================================================
void MuscleWrap::peteTest() const
{
	cout << "   Muscle Wrap " << getName() << endl;

	cout << "      wrap_object: " << _wrapObjectName << endl;
	cout << "      algorithm: " << _algorithmName << endl;
	cout << "      range: " << _range[0] << " " << _range[1] << endl;
}
