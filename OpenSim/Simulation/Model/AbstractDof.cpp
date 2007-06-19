// AbstractDof.cpp
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
#include "AbstractDof.h"
#include "AbstractDynamicsEngine.h"
#include "AbstractJoint.h"
#include "CoordinateSet.h"

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
AbstractDof::AbstractDof(void) :
	_function(_functionProp.getValueObjPtrRef()),
	_coordinateName((std::string&)_coordinateNameProp.getValueStr()),
	_coordinate(NULL)
{
	setNull();
	setupProperties();
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aDof AbstractDof to be copied.
 */
AbstractDof::AbstractDof(const AbstractDof &aDof) :
   Object(aDof),
	_function(_functionProp.getValueObjPtrRef()),
	_coordinateName((std::string&)_coordinateNameProp.getValueStr()),
	_coordinate(NULL)
{
	setNull();
	setupProperties();
	copyData(aDof);
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
AbstractDof::~AbstractDof(void)
{
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Copy data members from one AbstractDof to another.
 *
 * @param aDof AbstractDof to be copied.
 */
void AbstractDof::copyData(const AbstractDof &aDof)
{
	_function = (Function*)Object::SafeCopy(aDof._function);
	_coordinateName = aDof._coordinateName;
	_coordinate = aDof._coordinate;
}

//_____________________________________________________________________________
/**
 * Set the data members of this AbstractDof to their null values.
 */
void AbstractDof::setNull(void)
{
	setType("AbstractDof");
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void AbstractDof::setupProperties(void)
{
	_functionProp.setName("Value");
	_propertySet.append(&_functionProp);

	_coordinateNameProp.setName("coordinate");
	_propertySet.append(&_coordinateNameProp);
}

//_____________________________________________________________________________
/**
 * Perform set up functions after model has been deserialized or copied.
 *
 * @param aEngine dynamics engine containing this dof.
 * @param aJoint joint containing this dof.
 */
void AbstractDof::setup(AbstractDynamicsEngine* aEngine, AbstractJoint* aJoint)
{
	string errorMessage;

	// Look up the coordinate by name in the kinematics
	// engine and store a pointer to it.
	_coordinate = aEngine->getCoordinateSet()->get(_coordinateName);

	// _coordinate will be NULL if _coordinateName is not the
	// name of a valid model coordinate. This is OK, unless
	// the DOF is a function.
	Function* f = getFunction();
	if (f && f->getNumberOfPoints() > 0 && !_coordinate){
		errorMessage += "Invalid coordinate (" + _coordinateName + ") specified in dof " + getName() + " in joint " + aJoint->getName();
		throw (Exception(errorMessage));
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
AbstractDof& AbstractDof::operator=(const AbstractDof &aDof)
{
	// BASE CLASS
	Object::operator=(aDof);

	copyData(aDof);

	return(*this);
}

//=============================================================================
// GET AND SET
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the name of the generalized coordinate that controls this degree
 * of freedom.
 *
 * @param aName Name of the generalized coordinate.
 */
void AbstractDof::
setCoordinateName(const string& aName)
{
	_coordinateName = aName;
}
//_____________________________________________________________________________
/**
 * Utility function to return the function that this dof uses.
 * NULL is returned if the dof is a constant.
 *
 * @return Function used by this dof.
 */
Function* AbstractDof::getFunction() const
{
	return _function;
}

void AbstractDof::peteTest(void)
{
	cout << "DOF base class" << endl;
}
