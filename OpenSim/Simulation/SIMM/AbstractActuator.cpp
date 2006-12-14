// AbstractActuator.cpp
// Author: Frank C. Anderson, Peter Loan
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
#include "AbstractActuator.h"
#include <OpenSim/Tools/Object.h>
#include <OpenSim/Tools/Memory.h>
#include <OpenSim/Tools/rdMath.h>
#include <OpenSim/Tools/DebugUtilities.h>
#include <sstream>

using namespace std;
using namespace OpenSim;
//=============================================================================
// STATICS
//=============================================================================
const double AbstractActuator::LARGE = 1.0e8;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Construct an actuator that has a specified number of controls, states,
 * and pseudostates.
 *
 * @param aNX Number of controls.
 * @param aNY Number of states.
 * @param aNYP Number of pseudostates.
 */
AbstractActuator::AbstractActuator() :
   Object(),
	_model(NULL),
	_appliesForce(true),
	_force(0.0),
	_speed(0.0),
	_x(NULL),
	_y(NULL),
	_yp(NULL),
	_controlSuffixes(""),
	_stateSuffixes(""),
	_pseudoStateSuffixes("")
{
	setNull();
}

//_____________________________________________________________________________
/**
 * Construct an actuator from an XML Element.
 *
 * @param aNX Number of controls.
 * @param aNY Number of states.
 * @param aNYP Number of pseudo-states.
 * @param aElement XML element.
 */
AbstractActuator::AbstractActuator(DOMElement *aElement) :
	Object(aElement),
	_model(NULL),
	_appliesForce(true),
	_force(0.0),
	_speed(0.0),
	_x(NULL),
	_y(NULL),
	_yp(NULL),
	_controlSuffixes(""),
	_stateSuffixes(""),
	_pseudoStateSuffixes("")
{
	setNull();
	updateFromXMLNode();
}
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aActuator Actuator to copy.
 */
AbstractActuator::AbstractActuator(const AbstractActuator &aAct) :
	Object(aAct),
	_model(NULL),
	_appliesForce(true),
	_force(0.0),
	_speed(0.0),
	_x(NULL),
	_y(NULL),
	_yp(NULL),
	_controlSuffixes(""),
	_stateSuffixes(""),
	_pseudoStateSuffixes("")
{
	setNull();

	// ASSIGN
	*this = aAct;
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
AbstractActuator::~AbstractActuator()
{
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the data members of this AbstractActuator to their null values.
 */
void AbstractActuator::
setNull()
{
	setType("AbstractActuator");
}

//_____________________________________________________________________________
/**
 */
void AbstractActuator::setNumControls(int aNumControls)
{
	_x.setSize(aNumControls);
	_controlSuffixes.setSize(aNumControls);
}
void AbstractActuator::setNumStates(int aNumStates)
{
	_y.setSize(aNumStates);
	_stateSuffixes.setSize(aNumStates);
}
void AbstractActuator::setNumPseudoStates(int aNumPseudoStates)
{
	_yp.setSize(aNumPseudoStates);
	_pseudoStateSuffixes.setSize(aNumPseudoStates);
}
//_____________________________________________________________________________
/**
 */
void AbstractActuator::bindControl(int aIndex, double &x, const std::string &aSuffix)
{
	_x[aIndex] = &x;
	_controlSuffixes.set(aIndex, aSuffix);
}
void AbstractActuator::bindState(int aIndex, double &y, const std::string &aSuffix)
{
	_y[aIndex] = &y;
	_stateSuffixes.set(aIndex, aSuffix);
}
void AbstractActuator::bindPseudoState(int aIndex, double &yp, const std::string &aSuffix)
{
	_yp[aIndex] = &yp;
	_pseudoStateSuffixes.set(aIndex, aSuffix);
}
//_____________________________________________________________________________
/**
 * Perform set up functions after model has been deserialized or copied.
 *
 * @param aModel model containing this actuator.
 */
void AbstractActuator::setup(AbstractModel* aModel)
{
	_model = aModel;
}
//_____________________________________________________________________________
/**
 * Update the geometric representation of the Actuator if any.
 * The resulting geometry is maintained at the VisibleObject layer
 * 
 */
void AbstractActuator::updateGeometry()
{
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
AbstractActuator& AbstractActuator::
operator=(const AbstractActuator &aAct)
{
	// BASE CLASS
	Object::operator=(aAct);

	// MODEL
	_model = aAct.getModel();

	// FORCE
	_appliesForce = aAct.getAppliesForce();
	setForce(aAct.getForce());

	// SPEED
	_speed = aAct.getSpeed();

	// NOTE: _x, _y, _yp are not copied because they contain pointers that reference within this object; it is safe to
	// copy the suffixes, though possibly unnecessary (since they will be set in the object's setNull() before we copy into it).
	_controlSuffixes = aAct._controlSuffixes;
	_stateSuffixes = aAct._stateSuffixes;
	_pseudoStateSuffixes = aAct._pseudoStateSuffixes;

	return(*this);
}
// CONTROLS
//_____________________________________________________________________________
/**
 * Get the name of a control, given its index.
 *
 * @param aIndex The index of the control to get.
 * @return The name of the control.
 */
const string AbstractActuator::getControlName(int aIndex) const
{
	if(0<=aIndex && aIndex<_x.getSize())
		return getName() + "." + _controlSuffixes[aIndex];
	else {
		std::stringstream msg;
		msg << "AbstractActuator::getControlName: ERR- index out of bounds.\nAbstractActuator " 
			 <<  getName() << "  of type " << getType() << " has " << getNumControls() << " controls.";
		throw( Exception(msg.str(),__FILE__,__LINE__) );
	}
}

//_____________________________________________________________________________
/**
 * Get the index of a control, given the name.
 *
 * @param aName The name of the control to get.
 * @return The index of the control.
 */
int AbstractActuator::getControlIndex(const string &aName) const
{
	for(int i=0;i<_controlSuffixes.getSize();i++)
		if(getName()+"."+_controlSuffixes[i]==aName)
			return i;
	return -1;
}

//_____________________________________________________________________________
/**
 * Set an actuator control, specified by index
 *
 * @param aIndex The index of the control to set.
 * @param aValue The value to set the control to.
 */
void AbstractActuator::setControl(int aIndex, double aValue)
{
	if(0<=aIndex && aIndex<_x.getSize()) {
		if(_x[aIndex]) *_x[aIndex] = aValue;
		else {
			std::stringstream msg;
			msg << "AbstractActuator::setControl: ERR- control " << aIndex << " (" << getControlName(aIndex) << ") is NULL.";
			throw( Exception(msg.str(),__FILE__,__LINE__) );
		}
	} else {
		std::stringstream msg;
		msg << "AbstractActuator::setControl: ERR- index out of bounds.\nActuator " 
			 << getName() << " of type " << getType() << " has " << getNumControls() << " controls.";
		throw( Exception(msg.str(),__FILE__,__LINE__) );
	}
}

//_____________________________________________________________________________
/**
 * Set all of the controls of an actuator.
 *
 * @param aX The array of controls to set.
 */
void AbstractActuator::setControls(const double aX[])
{
	for(int i=0;i<_x.getSize();i++) {
		if(_x[i]) *_x[i]=aX[i];
		else {
			std::stringstream msg;
			msg << "AbstractActuator::setControls: ERR- control " << i << " (" << getControlName(i) << ") is NULL.";
			throw( Exception(msg.str(),__FILE__,__LINE__) );
		}
	}
}

//_____________________________________________________________________________
/**
 * Get an actuator control, by index.
 *
 * @param aIndex the index of the control to get.
 * @return The value of the control.
 */
double AbstractActuator::getControl(int aIndex) const
{
	if(0<=aIndex && aIndex<_x.getSize()) {
		if(_x[aIndex]) return *_x[aIndex];
		else {
			std::stringstream msg;
			msg << "AbstractActuator::getControl: ERR- control " << aIndex << " (" << getControlName(aIndex) << ") is NULL.";
			throw( Exception(msg.str(),__FILE__,__LINE__) );
		}
	} else {
		std::stringstream msg;
		msg << "AbstractActuator::getControl: ERR- index out of bounds.\nActuator " 
			 << getName() << " of type " << getType() << " has " << getNumControls() << " controls.";
		throw( Exception(msg.str(),__FILE__,__LINE__) );
	}
}

//_____________________________________________________________________________
/**
 * Get all of the controls of the actuator.
 *
 * @param rX The array of controls is returned here.
 */
void AbstractActuator::getControls(double rX[]) const
{
	for(int i=0;i<_x.getSize();i++) {
		if(_x[i]) rX[i]=*_x[i];
		else {
			std::stringstream msg;
			msg << "AbstractActuator::getControls: ERR- control " << i << " (" << getControlName(i) << ") is NULL.";
			throw( Exception(msg.str(),__FILE__,__LINE__) );
		}
	}
}
// STATES
//_____________________________________________________________________________
/**
 * Get the name of a state, given its index.
 *
 * @param aIndex The index of the state to get.
 * @return The name of the state.
 */
const string AbstractActuator::getStateName(int aIndex) const
{
	if(0<=aIndex && aIndex<_y.getSize())
		return getName() + "." + _stateSuffixes[aIndex];
	else {
		std::stringstream msg;
		msg << "AbstractActuator::getStateName: ERR- index out of bounds.\nActuator " 
			 << getName() << " of type " << getType() << " has " << getNumStates() << " states.";
		throw( Exception(msg.str(),__FILE__,__LINE__) );
	}
}

//_____________________________________________________________________________
/**
 * Get the index of a state, given the name.
 *
 * @param aName The name of the state to get.
 * @return The index of the state.
 */
int AbstractActuator::getStateIndex(const string &aName) const
{
	for(int i=0;i<_stateSuffixes.getSize();i++)
		if(getName()+"."+_stateSuffixes[i]==aName)
			return i;
	return -1;
}

//_____________________________________________________________________________
/**
 * Set an actuator state, specified by index
 *
 * @param aIndex The index of the state to set.
 * @param aValue The value to set the state to.
 */
void AbstractActuator::setState(int aIndex, double aValue)
{
	if(0<=aIndex && aIndex<_y.getSize()) {
		if(_y[aIndex]) *_y[aIndex] = aValue;
		else {
			std::stringstream msg;
			msg << "AbstractActuator::setState: ERR- state " << aIndex << " (" << getStateName(aIndex) << ") is NULL.";
			throw( Exception(msg.str(),__FILE__,__LINE__) );
		}
	} else {
		std::stringstream msg;
		msg << "AbstractActuator::setState: ERR- index out of bounds.\nActuator " 
			 << getName() << " of type " << getType() << " has " << getNumStates() << " states.";
		throw( Exception(msg.str(),__FILE__,__LINE__) );
	}
}

//_____________________________________________________________________________
/**
 * Set all of the states of an actuator.
 *
 * @param aY The array of states to set.
 */
void AbstractActuator::setStates(const double aY[])
{
	for(int i=0;i<_y.getSize();i++) {
		if(_y[i]) *_y[i]=aY[i];
		else {
			std::stringstream msg;
			msg << "AbstractActuator::setStates: ERR- state " << i << " (" << getStateName(i) << ") is NULL.";
			throw( Exception(msg.str(),__FILE__,__LINE__) );
		}
	}
}

//_____________________________________________________________________________
/**
 * Get an actuator state, by index.
 *
 * @param aIndex the index of the state to get.
 * @return The value of the state.
 */
double AbstractActuator::getState(int aIndex) const
{
	if(0<=aIndex && aIndex<_y.getSize()) {
		if(_y[aIndex]) return *_y[aIndex];
		else {
			std::stringstream msg;
			msg << "AbstractActuator::getState: ERR- state " << aIndex << " (" << getStateName(aIndex) << ") is NULL.";
			throw( Exception(msg.str(),__FILE__,__LINE__) );
		}
	} else {
		std::stringstream msg;
		msg << "AbstractActuator::getState: ERR- index out of bounds.\nActuator " 
		    << getName() << " of type " << getType() << " has " << getNumStates() << " states.";
		throw( Exception(msg.str(),__FILE__,__LINE__) );
	}
}

//_____________________________________________________________________________
/**
 * Get all of the states of the actuator.
 *
 * @param rY The array of states is returned here.
 */
void AbstractActuator::getStates(double rY[]) const
{
	for(int i=0;i<_y.getSize();i++) {
		if(_y[i]) rY[i]=*_y[i];
		else {
			std::stringstream msg;
			msg << "AbstractActuator::getStates: ERR- state " << i << " (" << getStateName(i) << ") is NULL.";
			throw( Exception(msg.str(),__FILE__,__LINE__) );
		}
	}
}
// PSEUDO STATES
//_____________________________________________________________________________
/**
 * Get the name of a pseudoState, given its index.
 *
 * @param aIndex The index of the pseudoState to get.
 * @return The name of the pseudoState.
 */
const string AbstractActuator::getPseudoStateName(int aIndex) const
{
	if(0<=aIndex && aIndex<_yp.getSize())
		return getName() + "." + _pseudoStateSuffixes[aIndex];
	else {
		std::stringstream msg;
		msg << "AbstractActuator::getPseudoStateName: ERR- index out of bounds.\nAbstractActuator " 
			 << getName() << " of type " << getType() << " has " << getNumPseudoStates() << " pseudo states.";
		throw( Exception(msg.str(),__FILE__,__LINE__) );
	}
}

//_____________________________________________________________________________
/**
 * Get the index of a pseudoState, given the name.
 *
 * @param aName The name of the pseudoState to get.
 * @return The index of the pseudoState.
 */
int AbstractActuator::getPseudoStateIndex(const string &aName) const
{
	for(int i=0;i<_pseudoStateSuffixes.getSize();i++)
		if(getName()+"."+_pseudoStateSuffixes[i]==aName)
			return i;
	return -1;
}

//_____________________________________________________________________________
/**
 * Set an actuator pseudoState, specified by index
 *
 * @param aIndex The index of the pseudoState to set.
 * @param aValue The value to set the pseudoState to.
 */
void AbstractActuator::setPseudoState(int aIndex, double aValue)
{
	if(0<=aIndex && aIndex<_yp.getSize()) {
		if(_yp[aIndex]) *_yp[aIndex] = aValue;
		else {
			std::stringstream msg;
			msg << "AbstractActuator::setPseudoState: ERR- pseudo state " << aIndex << " (" << getPseudoStateName(aIndex) << ") is NULL.";
			throw( Exception(msg.str(),__FILE__,__LINE__) );
		}
	} else {
		std::stringstream msg;
		msg << "AbstractActuator::setPseudoState: ERR- index out of bounds.\nActuator " 
		    << getName() << " of type " << getType() << " has " << getNumPseudoStates() << " pseudo states.";
		throw( Exception(msg.str(),__FILE__,__LINE__) );
	}
}

//_____________________________________________________________________________
/**
 * Set all of the pseudoStates of an actuator.
 *
 * @param aY The array of pseudoStates to set.
 */
void AbstractActuator::setPseudoStates(const double aY[])
{
	for(int i=0;i<_yp.getSize();i++) {
		if(_yp[i]) *_yp[i]=aY[i];
		else {
			std::stringstream msg;
			msg << "AbstractActuator::setPseudoStates: ERR- pseudo state " << i << " (" << getPseudoStateName(i) << ") is NULL.";
			throw( Exception(msg.str(),__FILE__,__LINE__) );
		}
	}
}

//_____________________________________________________________________________
/**
 * Get an actuator pseudoState, by index.
 *
 * @param aIndex the index of the pseudoState to get.
 * @return The value of the pseudoState.
 */
double AbstractActuator::getPseudoState(int aIndex) const
{
	if(0<=aIndex && aIndex<_yp.getSize()) {
		if(_yp[aIndex]) return *_yp[aIndex];
		else {
			std::stringstream msg;
			msg << "AbstractActuator::getPseudoState: ERR- pseudo state " << aIndex << " (" << getPseudoStateName(aIndex) << ") is NULL.";
			throw( Exception(msg.str(),__FILE__,__LINE__) );
		}
	} else {
		std::stringstream msg;
		msg << "AbstractActuator::getPseudoState: ERR- index out of bounds.\nActuator " 
		    << getName() << " of type " << getType() << " has " << getNumPseudoStates() << " pseudo states.";
		throw( Exception(msg.str(),__FILE__,__LINE__) );
	}
}

//_____________________________________________________________________________
/**
 * Get all of the pseudoStates of the actuator.
 *
 * @param rY The array of pseudoStates is returned here.
 */
void AbstractActuator::getPseudoStates(double rY[]) const
{
	for(int i=0;i<_yp.getSize();i++) {
		if(_yp[i]) rY[i]=*_yp[i];
		else {
			std::stringstream msg;
			msg << "AbstractActuator::getPseudoStates: ERR- pseudo state " << i << " (" << getPseudoStateName(i) << ") is NULL.";
			throw( Exception(msg.str(),__FILE__,__LINE__) );
		}
	}
}

//_____________________________________________________________________________
/**
 * getStress needs to be overridden by derived classes to be usable
 */
double AbstractActuator::getStress() const
{
	OPENSIM_ERROR_IF_NOT_OVERRIDDEN();
}
