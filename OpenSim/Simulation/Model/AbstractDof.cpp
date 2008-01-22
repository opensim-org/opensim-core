// AbstractDof.cpp
// Author: Peter Loan
/*
 * Copyright (c)  2006, Stanford University. All rights reserved. 
* Use of the OpenSim software in source form is permitted provided that the following
* conditions are met:
* 	1. The software is used only for non-commercial research and education. It may not
*     be used in relation to any commercial activity.
* 	2. The software is not distributed or redistributed.  Software distribution is allowed 
*     only through https://simtk.org/home/opensim.
* 	3. Use of the OpenSim software or derivatives must be acknowledged in all publications,
*      presentations, or documents describing work in which OpenSim or derivatives are used.
* 	4. Credits to developers may not be removed from executables
*     created from modifications of the source.
* 	5. Modifications of source code must retain the above copyright notice, this list of
*     conditions and the following disclaimer. 
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
*  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
*  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
*  SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
*  TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
*  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR BUSINESS INTERRUPTION) OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
*  WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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
	_coordinateName((std::string&)_coordinateNameProp.getValueStr())
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
	_coordinateName((std::string&)_coordinateNameProp.getValueStr())
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
	_joint = aDof._joint;
}

//_____________________________________________________________________________
/**
 * Set the data members of this AbstractDof to their null values.
 */
void AbstractDof::setNull(void)
{
	_coordinate = NULL;
	_joint = NULL;
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

	_joint = aJoint;

	// Look up the coordinate by name in the kinematics
	// engine and store a pointer to it.
	if (aEngine != NULL)
	   _coordinate = aEngine->getCoordinateSet()->get(_coordinateName);
	else
		_coordinate = NULL;

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
 * Utility to return the function that this dof uses.
 *
 * @return Function used by this dof.
 */
Function* AbstractDof::getFunction() const
{
	return _function;
}

//_____________________________________________________________________________
/**
 * Utility to change the function that this dof uses.
 *
 * @param aFunction the function to change to.
 */
void AbstractDof::setFunction(Function* aFunction)
{
	_function = aFunction;
	if (_joint)
	   _joint->invalidate();
}
