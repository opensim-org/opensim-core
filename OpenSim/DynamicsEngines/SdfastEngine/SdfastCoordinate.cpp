// SdfastCoordinate.cpp
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
#include "SdfastCoordinate.h"
#include "SdfastEngine.h"
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/AbstractJoint.h>
#include <OpenSim/Common/SimmIO.h>
#include <OpenSim/Common/SimmMacros.h>

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
SdfastCoordinate::SdfastCoordinate() :
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
	_constraintFunction(_constraintFunctionProp.getValueObjPtrRef()),
	_motionType(AbstractDof::Rotational),
	_index(_indexProp.getValueInt()),
	_joint(_jointProp.getValueInt()),
	_axis(_axisProp.getValueInt()),
	_SdfastEngine(NULL)
{
	setNull();
	setupProperties();
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
SdfastCoordinate::~SdfastCoordinate()
{
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aCoordinate SdfastCoordinate to be copied.
 */
SdfastCoordinate::SdfastCoordinate(const SdfastCoordinate &aCoordinate) :
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
	_constraintFunction(_constraintFunctionProp.getValueObjPtrRef()),
	_motionType(AbstractDof::Rotational),
	_index(_indexProp.getValueInt()),
	_joint(_jointProp.getValueInt()),
	_axis(_axisProp.getValueInt()),
	_SdfastEngine(NULL)
{
	setNull();
	setupProperties();
	copyData(aCoordinate);
}

//_____________________________________________________________________________
/**
 * Copy constructor from an AbstractCoordinate.
 *
 * @param aCoordinate SdfastCoordinate to be copied.
 */
SdfastCoordinate::SdfastCoordinate(const AbstractCoordinate &aCoordinate) :
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
	_constraintFunction(_constraintFunctionProp.getValueObjPtrRef()),
	_motionType(AbstractDof::Rotational),
	_index(_indexProp.getValueInt()),
	_joint(_jointProp.getValueInt()),
	_axis(_axisProp.getValueInt()),
	_SdfastEngine(NULL)
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
 * @return Pointer to a copy of this SdfastCoordinate.
 */
Object* SdfastCoordinate::copy() const
{
	SdfastCoordinate *gc = new SdfastCoordinate(*this);
	return(gc);
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Copy data members from one SdfastCoordinate to another.
 *
 * @param aCoordinate SdfastCoordinate to be copied.
 */
void SdfastCoordinate::copyData(const SdfastCoordinate &aCoordinate)
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
	_index = aCoordinate._index;
	_joint = aCoordinate._joint;
	_axis = aCoordinate._axis;
	_SdfastEngine = aCoordinate._SdfastEngine;
	_motionType = aCoordinate._motionType;
}

//_____________________________________________________________________________
/**
 * Copy data members from an AbstractCoordinate to an SdfastCoordinate.
 *
 * @param aCoordinate AbstractCoordinate to be copied.
 */
void SdfastCoordinate::copyData(const AbstractCoordinate &aCoordinate)
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
 * Set the data members of this SdfastCoordinate to their null values.
 */
void SdfastCoordinate::setNull(void)
{
	setType("SdfastCoordinate");
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void SdfastCoordinate::setupProperties(void)
{
	_defaultValueProp.setName("default_value");
	_defaultValueProp.setValue(0.0);
	_propertySet.append(&_defaultValueProp);

	_initialValueProp.setName("initial_value");
	_initialValueProp.setValue(0.0);
	_propertySet.append(&_initialValueProp);

	_toleranceProp.setName("tolerance");
	_toleranceProp.setValue(0.0);
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

	_QTypeProp.setName("q_type");
	_QTypeProp.setValue(dpUnconstrained);
	_propertySet.append(&_QTypeProp);

	_indexProp.setName("index");
	_propertySet.append(&_indexProp);

	_jointProp.setName("joint");
	_propertySet.append(&_jointProp);

	_axisProp.setName("axis");
	_propertySet.append(&_axisProp);

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
 * @param aEngine dynamics engine containing this SdfastCoordinate.
 */
void SdfastCoordinate::setup(AbstractDynamicsEngine* aEngine)
{
	// Base class
	AbstractCoordinate::setup(aEngine);

	_SdfastEngine = dynamic_cast<SdfastEngine*>(aEngine);

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

#if 0
	// This code no longer works, now that the coordinate set
	// has been moved into AbstractDynamicsEngine. This is because
	// SdfastCoordinate::setup() is called before the state
	// vector has been created, so setValue() will do nothing.
	// I think this is OK, though, because the states are
	// initialized later by SdfastEngine::initializeState(),
	// after the state vector has been created.

	// If the user specified a default value, set the
	// current value to the default value, whether or not
	// the coordinate is locked.
	if (!_defaultValueProp.getUseDefault()) {
		bool lockedState = getLocked();
		setLocked(false);
		setValue(_defaultValue);
		setLocked(lockedState);
	}
#endif

	determineType();
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
SdfastCoordinate& SdfastCoordinate::operator=(const SdfastCoordinate &aCoordinate)
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
 * Determine whether a coordinate is rotational or translational.
 *
 */
void SdfastCoordinate::determineType()
{
	int info[50], slider[6];

	_SdfastEngine->_sdjnt(getJointIndex(), info, slider);

	if (slider[getAxisIndex()] == 0)
		_motionType = AbstractDof::Rotational;
	else
		_motionType = AbstractDof::Translational;
}

//_____________________________________________________________________________
/**
 * Update an existing coordinate with parameter values from a
 * new one, but only for the parameters that were explicitly
 * specified in the XML node.
 *
 * @param aCoordinate coordinate to update from
 */
void SdfastCoordinate::updateFromCoordinate(const AbstractCoordinate &aCoordinate)
{
	if (!aCoordinate.getValueUseDefault())
		setValue(aCoordinate.getValue());

	if (!aCoordinate.getRangeUseDefault())
	{
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
//_____________________________________________________________________________
/**
 * Get the value.
 *
 * @return The current value of the coordinate.
 */
double SdfastCoordinate::getValue() const
{
	double* y = _SdfastEngine->getConfiguration();
	return y[_index];
}

//_____________________________________________________________________________
/**
 * Set the value.
 *
 * @param aValue value to change to.
 * @return Whether or not the value was changed.
 */
bool SdfastCoordinate::setValue(double aValue)
{
	if (_locked) {
		cout << "___WARNING___: Coordinate " << getName() << " is locked. Unable to change its value." << endl;
		return false;
	}

	double* y = _SdfastEngine->getConfiguration();
	if(y) {
		if (DABS(aValue - y[_index]) > _tolerance) {
			y[_index] = aValue;
		}
		_SdfastEngine->setConfiguration(y);
	}

#if 1
	// When interacting with the model via the GUI (or calling a function like
	// kinTest(), you need to execute the following code after changing a
	// coordinate value. But you don't want to call this code if you're
	// running a dynamic simulation. Maybe the solution is to leave this
	// code active, and only call setValue() when you're not running a
	// dynamic simulation (the simulation would call setConfiguration()).
	_locked = true;
	_SdfastEngine->assemble();
	_locked = false;
	_SdfastEngine->_sdstate(_SdfastEngine->getModel()->getTime(), y, &y[_SdfastEngine->getNumCoordinates()]);
#endif

	return true;
}

//_____________________________________________________________________________
/**
 * Set the range min and max.
 *
 * @param aRange range min and man to change to.
 * @return Whether or not the range was changed.
 */
bool SdfastCoordinate::setRange(double aRange[2])
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
bool SdfastCoordinate::setRangeMin(double aMin)
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
bool SdfastCoordinate::setRangeMax(double aMax)
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
bool SdfastCoordinate::setDefaultValue(double aDefaultValue)
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
void SdfastCoordinate::setInitialValue(double aInitialValue)
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
bool SdfastCoordinate::setTolerance(double aTolerance)
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
bool SdfastCoordinate::setStiffness(double aStiffness)
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
void SdfastCoordinate::getKeys(string rKeys[]) const
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
Function* SdfastCoordinate::getRestraintFunction() const
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
Function* SdfastCoordinate::getMinRestraintFunction(void) const
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
Function* SdfastCoordinate::getMaxRestraintFunction(void) const
{
	return _maxRestraintFunction;
}

//_____________________________________________________________________________
/**
 * Get the constraint function.
 *
 * @return Pointer to the constraint function.
 */
Function* SdfastCoordinate::getConstraintFunction() const
{
	return _constraintFunction;
}

//_____________________________________________________________________________
/**
 * Set the constraint function.
 */
void SdfastCoordinate::setConstraintFunction(const Function *function)
{
	_constraintFunction = (Function*)function->copy();
}

void SdfastCoordinate::peteTest(void) const
{
	cout << "Coordinate: " << getName() << endl;
	cout << "   default_value: " << _defaultValue << endl;
	cout << "   tolerance: " << _tolerance << endl;
	cout << "   stiffness: " << _stiffness << endl;
	cout << "   range: " << _range << endl;
	cout << "   keys: " << _keys << endl;
	cout << "   clamped: " << ((_clamped) ? ("true") : ("false")) << endl;
	cout << "   locked: " << ((_locked) ? ("true") : ("false")) << endl;
	cout << "   index: " << _index << endl;
	cout << "   joint: " << _joint << endl;
	cout << "   axis: " << _axis << endl;
	if (_restraintFunction) cout << "   restraintFunction: " << *_restraintFunction << endl;
	if (_minRestraintFunction) cout << "   minRestraintFunction: " << *_minRestraintFunction << endl;
	if (_maxRestraintFunction) cout << "   maxRestraintFunction: " << *_maxRestraintFunction << endl;
	cout << "   restraintActive: " << ((_restraintActive) ? ("true") : ("false")) << endl;
	if (_constraintFunction) cout << "   constraintFunction: " << *_constraintFunction << endl;
}

