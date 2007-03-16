// SdfastSpeed.cpp
// Author: Peter Loan, Frank C. Anderson
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
#include "SdfastSpeed.h"
#include "SdfastEngine.h"
#include <OpenSim/Simulation/SIMM/AbstractModel.h>
#include <OpenSim/Simulation/SIMM/SimmIO.h>
#include <OpenSim/Simulation/SIMM/SimmMacros.h>

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
SdfastSpeed::SdfastSpeed() :
   _defaultValue(_defaultValueProp.getValueDbl()),
	_index(_indexProp.getValueInt()),
	_coordinateName(_coordinateNameProp.getValueStr())
{
	setNull();
	setupProperties();
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
SdfastSpeed::~SdfastSpeed()
{
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aSpeed SdfastSpeed to be copied.
 */
SdfastSpeed::SdfastSpeed(const SdfastSpeed &aSpeed) :
   AbstractSpeed(aSpeed),
   _defaultValue(_defaultValueProp.getValueDbl()),
	_index(_indexProp.getValueInt()),
	_coordinateName(_coordinateNameProp.getValueStr())
{
	setNull();
	setupProperties();
	copyData(aSpeed);
}

//_____________________________________________________________________________
/**
 * Copy constructor from an AbstractSpeed.
 *
 * @param aSpeed SdfastSpeed to be copied.
 */
SdfastSpeed::SdfastSpeed(const AbstractSpeed &aSpeed) :
   AbstractSpeed(aSpeed),
   _defaultValue(_defaultValueProp.getValueDbl()),
	_index(_indexProp.getValueInt()),
	_coordinateName(_coordinateNameProp.getValueStr())
{
	setNull();
	setupProperties();
	copyData(aSpeed);
}

//_____________________________________________________________________________
/**
 * Copy this speed and return a pointer to the copy.
 * The copy constructor for this class is used.
 *
 * @return Pointer to a copy of this SdfastSpeed.
 */
Object* SdfastSpeed::copy() const
{
	SdfastSpeed *gc = new SdfastSpeed(*this);
	return(gc);
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Copy data members from one SdfastSpeed to another.
 *
 * @param aSpeed SdfastSpeed to be copied.
 */
void SdfastSpeed::copyData(const SdfastSpeed &aSpeed)
{
	_defaultValue = aSpeed.getDefaultValue();
	_index = aSpeed._index;
	_coordinateName = aSpeed._coordinateName;
	_SdfastEngine = aSpeed._SdfastEngine;
}

//_____________________________________________________________________________
/**
 * Copy data members from an AbstractSpeed to an SdfastSpeed.
 *
 * @param aSpeed AbstractSpeed to be copied.
 */
void SdfastSpeed::copyData(const AbstractSpeed &aSpeed)
{
	_defaultValue = aSpeed.getDefaultValue();
	_coordinate = aSpeed.getCoordinate();
	if (_coordinate)
		_coordinateName = _coordinate->getName();
	else
		_coordinateName = "Unassigned";
}

//_____________________________________________________________________________
/**
 * Set the data members of this SdfastSpeed to their null values.
 */
void SdfastSpeed::setNull(void)
{
	setType("SdfastSpeed");

	_coordinate = NULL;
	_SdfastEngine = NULL;
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void SdfastSpeed::setupProperties(void)
{
	_defaultValueProp.setName("default_value");
	_defaultValueProp.setValue(0.0);
	_propertySet.append(&_defaultValueProp);

	_indexProp.setName("index");
	_propertySet.append(&_indexProp);

	_coordinateNameProp.setName("coordinate");
	_propertySet.append(&_coordinateNameProp);
}

//_____________________________________________________________________________
/**
 * Perform some set up functions that happen after the
 * object has been deserialized or copied.
 *
 * @param aEngine dynamics engine containing this SdfastSpeed.
 */
void SdfastSpeed::setup(AbstractDynamicsEngine* aEngine)
{
	// Base class;
	AbstractSpeed::setup(aEngine);

	_SdfastEngine = dynamic_cast<SdfastEngine*>(aEngine);

	_coordinate = _SdfastEngine->getCoordinateSet()->get(_coordinateName);

	// If the user specified a default value, set the
	// current value to the default value.
	if (!_defaultValueProp.getUseDefault())
		setValue(_defaultValue);
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
SdfastSpeed& SdfastSpeed::operator=(const SdfastSpeed &aSpeed)
{
	// BASE CLASS
	AbstractSpeed::operator=(aSpeed);

	copyData(aSpeed);

	return(*this);
}


//=============================================================================
// COORDINATE
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the coordinate that this speed corresponds to.
 *
 * @param Pointer to the coordinate.
 * @return Whether or not the coordinate was set.
 */
bool SdfastSpeed::setCoordinate(AbstractCoordinate *aCoordinate)
{
	_coordinate = aCoordinate;
	return true;
}
//_____________________________________________________________________________
/**
 * Set the name of the coordinate that this speed corresponds to. When creating
 * a new SdfastSpeed object, this method should be used rather than setCoordinate,
 * because setup() will use _coordinateName to set _coordinate (which needs
 * to be done when deserializing SdfastSpeed objects).
 *
 * @param Name of the coordinate.
 * @return Whether or not the coordinate was set.
 */
bool SdfastSpeed::setCoordinateName(const string& aCoordName)
{
	_coordinateName = aCoordName;
	return true;
}


//=============================================================================
// DEFAULT VALUE
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the default value.
 *
 * @param aDefaultValue default value to change to.
 * @return Whether or not the default value was changed.
 */
bool SdfastSpeed::setDefaultValue(double aDefaultValue)
{
	_defaultValue = aDefaultValue;

	return true;
}


//=============================================================================
// VALUE
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the value.
 *
 * @param aValue value to change to.
 * @return Whether or not the value was changed.
 */
bool SdfastSpeed::setValue(double aValue)
{
	double *y = _SdfastEngine->getConfiguration();
	if(y) {
		y[_SdfastEngine->getNumCoordinates() + _index] = aValue;
		_SdfastEngine->setConfiguration(y);
	}
	return true;
}

//_____________________________________________________________________________
/**
 * Get the value.
 *
 * @return The current value of the speed.
 */
double SdfastSpeed::getValue() const
{
	double* y = _SdfastEngine->getConfiguration();
	return y[_SdfastEngine->getNumCoordinates() + _index];
}


//=============================================================================
// ACCELERATION
//=============================================================================
//_____________________________________________________________________________
/**
 * Get the acceleration of this speed.  Note that the accelerations must
 * have been computed by the dynamics engine by calling computeDerivatives()
 * for this method to return a valid result.
 *
 * @return The current value of the acceleration.
 */
double SdfastSpeed::getAcceleration() const
{
	double* dy = _SdfastEngine->getDerivatives();
	return dy[_SdfastEngine->getNumCoordinates() + _index];
}


//=============================================================================
// TESTING
//=============================================================================
void SdfastSpeed::peteTest(void) const
{
	cout << "Speed: " << getName() << endl;
	cout << "   default_value: " << _defaultValue << endl;
	cout << "   index: " << _index << endl;
}

