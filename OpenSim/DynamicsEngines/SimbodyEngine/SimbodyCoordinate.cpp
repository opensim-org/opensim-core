// SimbodyCoordinate.cpp
// Author: Frank C. Anderson
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
#include "SimbodyCoordinate.h"
#include "SimbodyEngine.h"
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/AbstractJoint.h>
#include <OpenSim/Simulation/Model/AbstractMuscle.h>
#include <OpenSim/Common/rdMath.h>
#include <OpenSim/Common/SimmMacros.h>
//#include <OpenSim/Common/SimmIO.h>

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
SimbodyCoordinate::SimbodyCoordinate() :
   _defaultValue(_defaultValueProp.getValueDbl()),
   _initialValue(_initialValueProp.getValueDbl()),
   _tolerance(_toleranceProp.getValueDbl()),
   _stiffness(_stiffnessProp.getValueDbl()),
	_range(_rangeProp.getValueDblArray()),
	_keys(_keysProp.getValueStrArray()),
	_clamped(_clampedProp.getValueBool()),
	_locked(_lockedProp.getValueBool()),
	_QType(_QTypeProp.getValueInt()),
	_restraintFunction(_restraintFunctionProp.getValueObjPtrRef()),
	_minRestraintFunction(_minRestraintFunctionProp.getValueObjPtrRef()),
	_maxRestraintFunction(_maxRestraintFunctionProp.getValueObjPtrRef()),
	_restraintActive(_restraintActiveProp.getValueBool()),
	_constraintFunction(_constraintFunctionProp.getValueObjPtrRef())
{
	setNull();
	setupProperties();
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
SimbodyCoordinate::~SimbodyCoordinate()
{
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aCoordinate SimbodyCoordinate to be copied.
 */
SimbodyCoordinate::SimbodyCoordinate(const SimbodyCoordinate &aCoordinate) :
   AbstractCoordinate(aCoordinate),
	_defaultValue(_defaultValueProp.getValueDbl()),
   _initialValue(_initialValueProp.getValueDbl()),
   _tolerance(_toleranceProp.getValueDbl()),
   _stiffness(_stiffnessProp.getValueDbl()),
	_range(_rangeProp.getValueDblArray()),
	_keys(_keysProp.getValueStrArray()),
	_clamped(_clampedProp.getValueBool()),
	_locked(_lockedProp.getValueBool()),
	_QType(_QTypeProp.getValueInt()),
	_restraintFunction(_restraintFunctionProp.getValueObjPtrRef()),
	_minRestraintFunction(_minRestraintFunctionProp.getValueObjPtrRef()),
	_maxRestraintFunction(_maxRestraintFunctionProp.getValueObjPtrRef()),
	_restraintActive(_restraintActiveProp.getValueBool()),
	_constraintFunction(_constraintFunctionProp.getValueObjPtrRef())
{
	setNull();
	setupProperties();
	copyData(aCoordinate);
}

//_____________________________________________________________________________
/**
 * Copy constructor from an AbstractCoordinate.
 *
 * @param aCoordinate SimbodyCoordinate to be copied.
 */
SimbodyCoordinate::SimbodyCoordinate(const AbstractCoordinate &aCoordinate) :
   AbstractCoordinate(aCoordinate),
	_defaultValue(_defaultValueProp.getValueDbl()),
   _initialValue(_initialValueProp.getValueDbl()),
   _tolerance(_toleranceProp.getValueDbl()),
   _stiffness(_stiffnessProp.getValueDbl()),
	_range(_rangeProp.getValueDblArray()),
	_keys(_keysProp.getValueStrArray()),
	_clamped(_clampedProp.getValueBool()),
	_locked(_lockedProp.getValueBool()),
	_QType(_QTypeProp.getValueInt()),
	_restraintFunction(_restraintFunctionProp.getValueObjPtrRef()),
	_minRestraintFunction(_minRestraintFunctionProp.getValueObjPtrRef()),
	_maxRestraintFunction(_maxRestraintFunctionProp.getValueObjPtrRef()),
	_restraintActive(_restraintActiveProp.getValueBool()),
	_constraintFunction(_constraintFunctionProp.getValueObjPtrRef())
{
	setNull();
	setupProperties();
	copyData(aCoordinate);
}

//_____________________________________________________________________________
/**
 * Copy this coordinate and return a pointer to the copy.
 * The copy constructor for this class is used.
 *
 * @return Pointer to a copy of this SimbodyCoordinate.
 */
Object* SimbodyCoordinate::copy() const
{
	SimbodyCoordinate *gc = new SimbodyCoordinate(*this);
	return(gc);
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Copy data members from one SimbodyCoordinate to another.
 *
 * @param aCoordinate SimbodyCoordinate to be copied.
 */
void SimbodyCoordinate::copyData(const SimbodyCoordinate &aCoordinate)
{
	_defaultValue = aCoordinate.getDefaultValue();
	_initialValue = aCoordinate.getDefaultValue();
	_tolerance = aCoordinate.getTolerance();
	_stiffness = aCoordinate._stiffness;
	_range = aCoordinate._range;
	_keys = aCoordinate._keys;
	_clamped = aCoordinate._clamped;
	_locked = aCoordinate._locked;
	_restraintFunction = (Function*)Object::SafeCopy(aCoordinate._restraintFunction);
	_minRestraintFunction = (Function*)Object::SafeCopy(aCoordinate._minRestraintFunction);
	_maxRestraintFunction = (Function*)Object::SafeCopy(aCoordinate._maxRestraintFunction);
	_restraintActive = aCoordinate._restraintActive;
	_constraintFunction = (Function*)Object::SafeCopy(aCoordinate._constraintFunction);
	_engine = aCoordinate._engine;
}

//_____________________________________________________________________________
/**
 * Copy data members from an AbstractCoordinate to an SimbodyCoordinate.
 *
 * @param aCoordinate AbstractCoordinate to be copied.
 */
void SimbodyCoordinate::copyData(const AbstractCoordinate &aCoordinate)
{
	_defaultValue = aCoordinate.getDefaultValue();
	_initialValue = aCoordinate.getDefaultValue();
	_tolerance = aCoordinate.getTolerance();
	_stiffness = aCoordinate.getStiffness();
	aCoordinate.getRange(&_range[0]);
	_clamped = aCoordinate.getClamped();
	_locked = aCoordinate.getLocked();
#if 0 // TODO
	_restraintFunction = aCoordinate.getRestraintFunction();
	_minRestraintFunction = aCoordinate._minRestraintFunction;
	_maxRestraintFunction = aCoordinate._maxRestraintFunction;
	_restraintActive = aCoordinate._restraintActive;
	_constraintFunction = aCoordinate.getConstraintFunction();
#endif
}

//_____________________________________________________________________________
/**
 * Set the data members of this SimbodyCoordinate to their null values.
 */
void SimbodyCoordinate::setNull(void)
{
	setType("SimbodyCoordinate");
	_motionType = AbstractDof::Rotational;
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void SimbodyCoordinate::setupProperties(void)
{
	_defaultValueProp.setName("default_value");
	_defaultValueProp.setValue(0.0);
	_propertySet.append(&_defaultValueProp);

	_initialValueProp.setName("initial_value");
	_initialValueProp.setValue(0.0);
	_propertySet.append(&_initialValueProp);

	_toleranceProp.setName("tolerance");
	_toleranceProp.setValue(rdMath::SMALL);
	_propertySet.append(&_toleranceProp);

	_stiffnessProp.setName("stiffness");
	_stiffnessProp.setValue(0.0);
	_propertySet.append(&_stiffnessProp);

	const double defaultRange[] = {-999999.9, 999999.9};
	_rangeProp.setName("range");
	_rangeProp.setValue(2, defaultRange);
	_propertySet.append(&_rangeProp);

	_keysProp.setName("keys");
	_propertySet.append(&_keysProp);

	_clampedProp.setName("clamped");
	_clampedProp.setValue(true);
	_propertySet.append(&_clampedProp);

	_lockedProp.setName("locked");
	_lockedProp.setValue(false);
	_propertySet.append(&_lockedProp);

	_restraintFunctionProp.setName("restraint_function");
	_propertySet.append(&_restraintFunctionProp);

	_minRestraintFunctionProp.setName("min_restraint_function");
	_propertySet.append(&_minRestraintFunctionProp);

	_maxRestraintFunctionProp.setName("max_restraint_function");
	_propertySet.append(&_maxRestraintFunctionProp);

	_restraintActiveProp.setName("restraint_active");
	_restraintActiveProp.setValue(true);
	_propertySet.append(&_restraintActiveProp);

	_constraintFunctionProp.setName("constraint_function");
	_propertySet.append(&_constraintFunctionProp);
}

//_____________________________________________________________________________
/**
 * Perform some set up functions that happen after the
 * object has been deserialized or copied.
 *
 * @param aEngine dynamics engine containing this SimbodyCoordinate.
 */
void SimbodyCoordinate::setup(AbstractDynamicsEngine* aEngine)
{
	// Base class
	AbstractCoordinate::setup(aEngine);

	_engine = dynamic_cast<SimbodyEngine*>(aEngine);

	// Make sure the range is min to max.
	if (_range[1] < _range[0]){
		double tmp = _range[0];
		_range[0] = _range[1];
		_range[1] = tmp;
	}

	// Make sure the default value is in the range
	if (_defaultValue < _range[0])
		_defaultValue = _range[0];
	else if (_defaultValue > _range[1])
		_defaultValue = _range[1];

	// If the user specified a default value but not a value, set the
	// current value to the default value
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
SimbodyCoordinate& SimbodyCoordinate::operator=(const SimbodyCoordinate &aCoordinate)
{
	// BASE CLASS
	AbstractCoordinate::operator=(aCoordinate);

	copyData(aCoordinate);

	return(*this);
}

//=============================================================================
// UTILITY
//=============================================================================
//_____________________________________________________________________________
/**
 * Update an existing coordinate with parameter values from a
 * new one, but only for the parameters that were explicitly
 * specified in the XML node.
 *
 * @param aCoordinate coordinate to update from
 */
void SimbodyCoordinate::updateFromCoordinate(const AbstractCoordinate &aCoordinate)
{
	if (!aCoordinate.getValueUseDefault())
		setValue(aCoordinate.getValue());

	if (!aCoordinate.getRangeUseDefault()) {
		setRangeMin(aCoordinate.getRangeMin());
		setRangeMax(aCoordinate.getRangeMax());
	}

	if (!aCoordinate.getToleranceUseDefault())
		setTolerance(aCoordinate.getTolerance());

	if (!aCoordinate.getStiffnessUseDefault())
		setStiffness(aCoordinate.getStiffness());

	if (!aCoordinate.getDefaultValueUseDefault())
		setDefaultValue(aCoordinate.getDefaultValue());

	if (!aCoordinate.getClampedUseDefault())
		setClamped(aCoordinate.getClamped());

	if (!aCoordinate.getLockedUseDefault())
		setLocked(aCoordinate.getLocked());

#if 0
	// TODO
	if (!aCoordinate._restraintFunctionProp.getUseDefault())
		_restraintFunction = aCoordinate._restraintFunction;

	if (!aCoordinate._maxRestraintFunctionProp.getUseDefault())
		_maxRestraintFunction = aCoordinate._maxRestraintFunction;

	if (!aCoordinate._restraintActiveProp.getUseDefault())
		_restraintActive = aCoordinate._restraintActive;

	if (!aCoordinate._constraintFunctionProp.getUseDefault())
		_constraintFunction = aCoordinate._constraintFunction;
#endif
}

//=============================================================================
// GET AND SET
//=============================================================================
//done_____________________________________________________________________________
/**
 * Get the value.
 *
 * @return The current value of the coordinate.
 */
double SimbodyCoordinate::getValue() const
{
	return _engine->_matter->getMobilizerQ(*(_engine->_s),_bodyId,_mobilityIndex);
}
//done_____________________________________________________________________________
/**
 * Set the value.
 *
 * @param aValue value to change to.
 * @return Whether or not the value was changed.
 */
bool SimbodyCoordinate::setValue(double aValue) { return setValue(aValue, true); }
bool SimbodyCoordinate::setValue(double aValue, bool aRealize)
{
	// pull value into range if it's off by tolerance due to roundoff
	if (_clamped) {
		if (aValue < _range[0] && (_range[0]-aValue < _tolerance))
			aValue = _range[0];
		else if (aValue > _range[1] && (aValue-_range[1] < _tolerance))
			aValue = _range[1];
	}

	if ((aValue >= _range[0] && aValue <= _range[1]) || !_clamped) {
		// Check if the value is sufficiently different
		// JPL 11/19/07: For some reason, when solving a frame of marker data,
		// Ipopt needs to be able to set a coordinate to its current value
		// or it will crash (so even a tolerance of 0.0 does not work). So
		// for now, do not check against tolerance.
		//	if (DABS(aValue - getValue()) > _tolerance)
		if (1)
		{
			if (_locked) {
				cout<<"SimbodyCoordinate.setValue: WARN- coordinate "<<getName();
				cout<<" is locked. Unable to change its value." << endl;
				if(aRealize) _engine->_matter->realize(*(_engine->_s),Stage::Velocity);
				return false;
			}
			_engine->resetBodyAndMobilityForceVectors();
			_engine->_matter->setMobilizerQ(*(_engine->_s),_bodyId,_mobilityIndex,aValue);
			if(aRealize) _engine->_matter->realize(*(_engine->_s),Stage::Velocity);

			// TODO: use Observer mechanism for _jointList, _pathList, and muscles
			ActuatorSet* act = _engine->getModel()->getActuatorSet();
			for(int i=0; i<act->getSize(); i++) {
				AbstractMuscle* sm = dynamic_cast<AbstractMuscle*>(act->get(i));
				if(sm) sm->invalidatePath();
			}
		}
	} else {
		cout << "SimbodyCoordinate.setValue: WARN- Attempting to set coordinate " << getName() << " to a value (" <<
			aValue << ") outside its range (" << _range[0] << " to " << _range[1] << ")" << endl;
		return false;
	}

	return true;
}

//_____________________________________________________________________________
/**
 * Set the range min and max.
 *
 * @param aRange range min and man to change to.
 * @return Whether or not the range was changed.
 */
bool SimbodyCoordinate::setRange(double aRange[2])
{
	if (aRange[1] >= aRange[0]) {
		_range[0] = aRange[0];
		_range[1] = aRange[1];
		return true;
	}

	return false;
}

//_____________________________________________________________________________
/**
 * Set the range min.
 *
 * @param aRange range min to change to.
 * @return Whether or not the range min was changed.
 */
bool SimbodyCoordinate::setRangeMin(double aMin)
{
	if (aMin <= _range[1]) {
		_range[0] = aMin;
		return true;
	}

	return false;
}

//_____________________________________________________________________________
/**
 * Set the range max.
 *
 * @param aRange range max to change to.
 * @return Whether or not the range max was changed.
 */
bool SimbodyCoordinate::setRangeMax(double aMax)
{
	if (aMax >= _range[0]) {
		_range[1] = aMax;
		return true;
	}

	return false;
}

//_____________________________________________________________________________
/**
 * Set the default value.
 *
 * @param aDefaultValue default value to change to.
 * @return Whether or not the default value was changed.
 */
bool SimbodyCoordinate::setDefaultValue(double aDefaultValue)
{
	if (aDefaultValue >= _range[0] && aDefaultValue <= _range[1]) {
		_defaultValue = aDefaultValue;
		return true;
	}

	return false;
}

//_____________________________________________________________________________
/**
 * Set the initial value.  Used to initialize the initial_value field in sdm.q
 */
void SimbodyCoordinate::setInitialValue(double aInitialValue)
{
	_initialValue = aInitialValue;
}

//_____________________________________________________________________________
/**
 * Set the tolerance.
 *
 * @param aTolerance tolerance to change to.
 * @return Whether or not the tolerance was changed.
 */
bool SimbodyCoordinate::setTolerance(double aTolerance)
{
	if (aTolerance >= 0.0) {
		_tolerance = aTolerance;
		return true;
	}

	return false;
}

//_____________________________________________________________________________
/**
 * Set the stiffness.
 *
 * @param aStiffness stiffness to change to.
 * @return Whether or not the stiffness was changed.
 */
bool SimbodyCoordinate::setStiffness(double aStiffness)
{
	if (aStiffness >= 0.0)
	{
		_stiffness = aStiffness;
		return true;
	}

	return false;
}

//_____________________________________________________________________________
/**
 * Get the names of the keys used in the GUI to control this coordinate.
 *
 * @param rKeys names of the keys are returned here.
 */
void SimbodyCoordinate::getKeys(string rKeys[]) const
{
	for (int i = 0; i < _keys.getSize(); i++)
		rKeys[i] = _keys[i];
}

//_____________________________________________________________________________
/**
 * Get the restraint function used to keep this coordinate inside its range.
 *
 * @return Pointer to the restraint function.
 */
Function* SimbodyCoordinate::getRestraintFunction() const
{
	return _restraintFunction;
}

//_____________________________________________________________________________
/**
 * Get the restraint function used to keep this coordinate from going below
 * its minimum.
 *
 * @return Pointer to the min restraint function.
 */
Function* SimbodyCoordinate::getMinRestraintFunction(void) const
{
	return _minRestraintFunction;
}

//_____________________________________________________________________________
/**
 * Get the restraint function used to keep this coordinate from going above
 * its maximum.
 *
 * @return Pointer to the max restraint function.
 */
Function* SimbodyCoordinate::getMaxRestraintFunction(void) const
{
	return _maxRestraintFunction;
}

//_____________________________________________________________________________
/**
 * Get the constraint function.
 *
 * @return Pointer to the constraint function.
 */
Function* SimbodyCoordinate::getConstraintFunction() const
{
	return _constraintFunction;
}

//_____________________________________________________________________________
/**
 * Set the constraint function.
 */
void SimbodyCoordinate::setConstraintFunction(const Function *function)
{
	_constraintFunction = (Function*)function->copy();
}


//-----------------------------------------------------------------------------
// LOCK
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set whether or not this coordinate is locked.
 * A distance constraint is used to lock the joint a the dynamic level,
 * so the Simbody multibody system is reconstructed every time the lock
 * flag is changed.
 *
 * @param aLocked If true the joint is locked; if false the joint is unlocked.
 */
bool SimbodyCoordinate::setLocked(bool aLocked)
{
	if(aLocked == _locked) return true;

	_locked = aLocked;
	_engine->constructMultibodySystem();
	return true;
}



//=============================================================================
// UTILITY
//=============================================================================
//_____________________________________________________________________________
/**
 * This function tries to determine whether the coordinate is primarily rotational
 * or translational. This information is needed for two reasons:
 * 1. to know how big to set the step size for the arrow buttons on the slider for
 *    that gencoord in the Model Viewer (0.001 for translations, 1.0 for rotations).
 * 2. to know how to interpret the moment arm values w.r.t. that coordinate. If the
 *    coordinate maps directly to a rotational dof, then the coordinate's units need to
 *    be treated as degrees, and thus converted to radians when calculating moment arms.
 *    If the coordinate maps directly to a translational dof, then its units are meters
 *    and no conversion is needed. If the gencoord maps directly to a dof of each type,
 *    then it really is an abstract quantity, and interpreting the moment arm in
 *    real-world units is problematic. So just find the first dof that maps directly
 *    to it and go with that one.
 * To determine whether a coordinate is rotational or translational, find the first
 * dof that maps directly to the coordinate (has a function with two points and slope
 * equal to 1.0 or -1.0 and passes through zero). If there is no such function, then
 * just look at the dofs that the coordinate is used in. If they are all translational,
 * then the gencoord is translational. If one or more is rotational, then the coordinate
 * is rotational.
 */
void SimbodyCoordinate::determineType()
{
	// TODO: For now, I'm just setting the type to rotational.  However, when
	// the Simbody classes are really finished, this will have to return
	// something that makes sense.
	_motionType = AbstractDof::Translational;
}

