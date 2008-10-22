// SimbodySpeed01_05.cpp
// Author: Frank C. Anderson
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
#include "SimbodySpeed01_05.h"
#include "SimbodyEngine01_05.h"
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Common/SimmIO.h>
#include <OpenSim/Common/SimmMacros.h>

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace SimTK;
using namespace OpenSim;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */
SimbodySpeed01_05::SimbodySpeed01_05() :
   _defaultValue(_defaultValueProp.getValueDbl()),
	_coordinateName(_coordinateNameProp.getValueStr())
{
	setNull();
	setupProperties();
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
SimbodySpeed01_05::~SimbodySpeed01_05()
{
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aSpeed SimbodySpeed01_05 to be copied.
 */
SimbodySpeed01_05::SimbodySpeed01_05(const SimbodySpeed01_05 &aSpeed) :
   AbstractSpeed(aSpeed),
   _defaultValue(_defaultValueProp.getValueDbl()),
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
 * @param aSpeed SimbodySpeed01_05 to be copied.
 */
SimbodySpeed01_05::SimbodySpeed01_05(const AbstractSpeed &aSpeed) :
   AbstractSpeed(aSpeed),
   _defaultValue(_defaultValueProp.getValueDbl()),
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
 * @return Pointer to a copy of this SimbodySpeed01_05.
 */
Object* SimbodySpeed01_05::copy() const
{
	SimbodySpeed01_05 *gc = new SimbodySpeed01_05(*this);
	return(gc);
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Copy data members from one SimbodySpeed01_05 to another.
 *
 * @param aSpeed SimbodySpeed01_05 to be copied.
 */
void SimbodySpeed01_05::copyData(const SimbodySpeed01_05 &aSpeed)
{
	_defaultValue = aSpeed.getDefaultValue();
	_coordinateName = aSpeed._coordinateName;
	_engine = aSpeed._engine;
}

//_____________________________________________________________________________
/**
 * Copy data members from an AbstractSpeed to an SimbodySpeed01_05.
 *
 * @param aSpeed AbstractSpeed to be copied.
 */
void SimbodySpeed01_05::copyData(const AbstractSpeed &aSpeed)
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
 * Set the data members of this SimbodySpeed01_05 to their null values.
 */
void SimbodySpeed01_05::setNull(void)
{
	setType("SimbodySpeed");

	_coordinate = NULL;
	_engine = NULL;
	_mobilityIndex = -1;
	_engine = NULL;
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void SimbodySpeed01_05::setupProperties(void)
{
	_defaultValueProp.setName("default_value");
	_defaultValueProp.setValue(0.0);
	_propertySet.append(&_defaultValueProp);

	_coordinateNameProp.setName("coordinate");
	_propertySet.append(&_coordinateNameProp);
}

//_____________________________________________________________________________
/**
 * Perform some set up functions that happen after the
 * object has been deserialized or copied.
 *
 * @param aEngine dynamics engine containing this SimbodySpeed01_05.
 */
void SimbodySpeed01_05::setup(AbstractDynamicsEngine* aEngine)
{
	// Base class;
	AbstractSpeed::setup(aEngine);

	_engine = dynamic_cast<SimbodyEngine01_05*>(aEngine);

	_coordinate = _engine->getCoordinateSet()->get(_coordinateName);

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
SimbodySpeed01_05& SimbodySpeed01_05::operator=(const SimbodySpeed01_05 &aSpeed)
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
bool SimbodySpeed01_05::setCoordinate(AbstractCoordinate *aCoordinate)
{
	_coordinate = aCoordinate;
	return true;
}
//_____________________________________________________________________________
/**
 * Set the name of the coordinate that this speed corresponds to. When creating
 * a new SimbodySpeed01_05 object, this method should be used rather than setCoordinate,
 * because setup() will use _coordinateName to set _coordinate (which needs
 * to be done when deserializing SimbodySpeed01_05 objects).
 *
 * @param Name of the coordinate.
 * @return Whether or not the coordinate was set.
 */
bool SimbodySpeed01_05::setCoordinateName(const string& aCoordName)
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
bool SimbodySpeed01_05::setDefaultValue(double aDefaultValue)
{
	_defaultValue = aDefaultValue;
	return true;
}


//=============================================================================
// VALUE
//=============================================================================
//done_____________________________________________________________________________
/**
 * Set the value.
 *
 * @param aValue value to change to.
 * @return Whether or not the value was changed.
 */
bool SimbodySpeed01_05::setValue(double aValue)
{
	return true;
}

//done_____________________________________________________________________________
/**
 * Get the value.
 *
 * @return The current value of the speed.
 */
double SimbodySpeed01_05::getValue() const
{
	return 0.0;

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
double SimbodySpeed01_05::getAcceleration() const
{
	//_engine->_matter->getMobilizerUDot(*(_engine->_s),_bodyId,_mobilityIndex);
	Vector udot = _engine->_matter->getUDot(*(_engine->_s));
	//cout<<_coordinateName<<": udot("<<_bodyId-1<<")="<<udot[_bodyId-1]<<endl;
	return udot[_bodyId - 1];
}

