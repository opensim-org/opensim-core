// SdfastCoordinate.cpp
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
#include "SdfastCoordinate.h"
#include "SdfastEngine.h"
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/AbstractJoint.h>
#include <OpenSim/Simulation/Model/CoordinateSet.h>
#include <OpenSim/Common/rdMath.h>
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
	_constraintIndependentCoordinateName(_constraintIndependentCoordinateNameProp.getValueStr()),
	_constraintNumber(_constraintNumberProp.getValueInt()),
	_constraintFunction(_constraintFunctionProp.getValueObjPtrRef()),
	_motionType(AbstractTransformAxis::Rotational),
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
	_constraintIndependentCoordinateName(_constraintIndependentCoordinateNameProp.getValueStr()),
	_constraintNumber(_constraintNumberProp.getValueInt()),
	_constraintFunction(_constraintFunctionProp.getValueObjPtrRef()),
	_motionType(AbstractTransformAxis::Rotational),
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
	_constraintIndependentCoordinateName(_constraintIndependentCoordinateNameProp.getValueStr()),
	_constraintNumber(_constraintNumberProp.getValueInt()),
	_constraintFunction(_constraintFunctionProp.getValueObjPtrRef()),
	_motionType(AbstractTransformAxis::Rotational),
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
	_QType = aCoordinate._QType;
	_restraintFunction = (Function*)Object::SafeCopy(aCoordinate._restraintFunction);
	_minRestraintFunction = (Function*)Object::SafeCopy(aCoordinate._minRestraintFunction);
	_maxRestraintFunction = (Function*)Object::SafeCopy(aCoordinate._maxRestraintFunction);
	_restraintActive = aCoordinate._restraintActive;
	_constraintIndependentCoordinateName = aCoordinate._constraintIndependentCoordinateName;
	_constraintIndependentCoordinate = aCoordinate._constraintIndependentCoordinate; // TODO: should we be copying pointers?
	_constraintNumber = aCoordinate._constraintNumber;
	_constraintFunction = (Function*)Object::SafeCopy(aCoordinate._constraintFunction);
	_motionType = aCoordinate._motionType;
	_index = aCoordinate._index;
	_joint = aCoordinate._joint;
	_axis = aCoordinate._axis;
	_SdfastEngine = aCoordinate._SdfastEngine; // TODO: should we be copying pointers?
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

	_constraintIndependentCoordinateName = "";
	_constraintIndependentCoordinate = NULL;
	_constraintNumber = -1;
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
	_toleranceProp.setValue(rdMath::SMALL);
	_propertySet.append(&_toleranceProp);

	_stiffnessProp.setName("stiffness");
	_stiffnessProp.setValue(0.0);
	_propertySet.append(&_stiffnessProp);

	const double defaultRange[] = {-999999.9, 999999.9};
	_rangeProp.setName("range");
	_rangeProp.setValue(2, defaultRange);
	_rangeProp.setAllowableArraySize(2);
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

	_constraintIndependentCoordinateNameProp.setName("constraint_independent_coordinate");
	_propertySet.append(&_constraintIndependentCoordinateNameProp);

	_constraintNumberProp.setName("constraint_number");
	_propertySet.append(&_constraintNumberProp);

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

	if(getSdfastType() == dpConstrained) {
		if(!_constraintFunction)
			throw Exception("SdfastCoordinate.setup: ERR- Constrained coordinate '"+getName()+"' does not have valid constraint function",
								 __FILE__,__LINE__);

		if(_constraintNumber < 0)
			throw Exception("SdfastCoordinate.setup: ERR- Constraineed coordinate '"+getName()+
					"' does not have a valid constraint number",__FILE__,__LINE__);

		if(_constraintIndependentCoordinateName == "")
			throw Exception("SdfastCoordinate.setup: ERR- Constrained coordinate '"+getName()+
								 "' needs to have an independent coordinate set for its constraint function",__FILE__,__LINE__);

		AbstractCoordinate *coord = aEngine->getCoordinateSet()->get(_constraintIndependentCoordinateName);
		if(!coord) 
			throw Exception("SdfastCoordinate.setup: ERR- Did not find independent coordinate '"+_constraintIndependentCoordinateName+
								 "' needed for constraint function in coordinate '"+getName()+"'",__FILE__,__LINE__);

		_constraintIndependentCoordinate = dynamic_cast<SdfastCoordinate*>(coord);
		if(!_constraintIndependentCoordinate) 
			throw Exception("SdfastCoordinate.setup: ERR- Independent coordinate '"+_constraintIndependentCoordinateName+
								 "' referenced by constrained coordinate '"+getName()+"' is not an SdfastCoordinate",__FILE__,__LINE__);
	}

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
		_motionType = AbstractTransformAxis::Rotational;
	else
		_motionType = AbstractTransformAxis::Translational;
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
	// pull value into range if it's off by tolerance due to roundoff
	if (_clamped) {
		if (aValue < _range[0] && (_range[0]-aValue < _tolerance))
			aValue = _range[0];
		else if (aValue > _range[1] && (aValue-_range[1] < _tolerance))
			aValue = _range[1];
	}

	if ((aValue >= _range[0] && aValue <= _range[1]) || !_clamped) {
		// Check if the value is sufficiently different
		// if (DABS(aValue - getValue()) > _tolerance)
		if (1)
		{
			if (_locked) {
				cout << "SdfastCoordinate.setValue: WARN- Coordinate " << getName() << " is locked. Unable to change its value." << endl;
				return false;
			}

			double* y = _SdfastEngine->getConfiguration();
			if(y) {
				y[_index] = aValue;
				_SdfastEngine->setConfiguration(y);
			}
		} else {
			cout << "SdfastCoordinate.setValue: WARN- Attempting to set coordinate " << getName() << " to a value (" <<
				aValue << ") outside its range (" << _range[0] << " to " << _range[1] << ")" << endl;
			return false;
		}
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
