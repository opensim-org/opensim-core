// SimmDof.cpp
// Author: Peter Loan
/* Copyright (c) 2005, Stanford University and Peter Loan.
 * 
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
#include "SimmDof.h"
#include "SimmKinematicsEngine.h"
#include "SimmJoint.h"

//=============================================================================
// STATICS
//=============================================================================


using namespace OpenSim;
using namespace std;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */
SimmDof::SimmDof(void) :
	_functions((ArrayPtrs<Function>&)_functionsProp.getValueObjArray()),
	_coordinateName((std::string&)_coordinateNameProp.getValueStr()),
	_coordinate(NULL)
{
	setNull();
}

//_____________________________________________________________________________
/**
 * Construct a SimmDof from an XML Element.
 */
SimmDof::SimmDof(DOMElement *aElement) :
	Object(aElement),
	_functions((ArrayPtrs<Function>&)_functionsProp.getValueObjArray()),
	_coordinateName((std::string&)_coordinateNameProp.getValueStr()),
	_coordinate(NULL)
{
	setNull();
	updateFromXMLNode();
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aDof SimmDof to be copied.
 */
SimmDof::SimmDof(const SimmDof &aDof) :
   Object(aDof),
	_functions((ArrayPtrs<Function>&)_functionsProp.getValueObjArray()),
	_coordinateName((std::string&)_coordinateNameProp.getValueStr()),
	_coordinate(NULL)
{
	setupProperties();
	copyData(aDof);
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
SimmDof::~SimmDof(void)
{
}

void SimmDof::copyData(const SimmDof &aDof)
{
	_functions = aDof._functions;
	_coordinateName = aDof._coordinateName;
	_coordinate = aDof._coordinate;
}


//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the data members of this SimmDof to their null values.
 */
void SimmDof::setNull(void)
{
	setupProperties();
	setType("SimmDof");
	setName("");
}
//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void SimmDof::setupProperties(void)
{
	_functionsProp.setName("Value");
	ArrayPtrs<Object> func;
	_functionsProp.setValue(func);
	_propertySet.append(&_functionsProp);

	_coordinateNameProp.setName("DOF_coordinate");
	_propertySet.append(&_coordinateNameProp);
}

/* Perform some set up functions that happen after the
 * object has been deserialized or copied.
 */
void SimmDof::setup(SimmKinematicsEngine* aEngine, SimmJoint* aJoint)
{
	string errorMessage;

	/* Look up the coordinate by name in the kinematics
	 * engine and store a pointer to it.
	 */
	_coordinate = dynamic_cast<SimmCoordinate *> (aEngine->getCoordinate(_coordinateName));

	/* _coordinate will be NULL if _coordinateName is not the
	 * name of a valid model coordinate. This is OK, unless
	 * the user specified the coordinate name in the model file.
	 */
	if (!_coordinateNameProp.getUseDefault() && !_coordinate)
	{
		errorMessage += "Invalid coordinate (" + _coordinateName + ") specified in dof " + getName() + " in joint " + aJoint->getName();
		throw (Exception(errorMessage.c_str()));
	}
}

SimmDof& SimmDof::operator=(const SimmDof &aDof)
{
	// BASE CLASS
	Object::operator=(aDof);

	copyData(aDof);

	return(*this);
}

Function* SimmDof::getFunction() const
{
	if (_functions.getSize() < 0)
		return NULL;

	return _functions[0];
}

void SimmDof::peteTest(void)
{
	cout << "DOF base class" << endl;
}
