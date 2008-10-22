// SimbodyCoordinate01_05.cpp
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
#include "SimbodyCoordinate01_05.h"
#include "SimbodyEngine01_05.h"
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
SimbodyCoordinate01_05::SimbodyCoordinate01_05() :
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
SimbodyCoordinate01_05::~SimbodyCoordinate01_05()
{
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aCoordinate SimbodyCoordinate01_05 to be copied.
 */
SimbodyCoordinate01_05::SimbodyCoordinate01_05(const SimbodyCoordinate01_05 &aCoordinate) :
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
 * @param aCoordinate SimbodyCoordinate01_05 to be copied.
 */
SimbodyCoordinate01_05::SimbodyCoordinate01_05(const AbstractCoordinate &aCoordinate) :
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
 * @return Pointer to a copy of this SimbodyCoordinate01_05.
 */
Object* SimbodyCoordinate01_05::copy() const
{
	SimbodyCoordinate01_05 *gc = new SimbodyCoordinate01_05(*this);
	return(gc);
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Copy data members from one SimbodyCoordinate01_05 to another.
 *
 * @param aCoordinate SimbodyCoordinate01_05 to be copied.
 */
void SimbodyCoordinate01_05::copyData(const SimbodyCoordinate01_05 &aCoordinate)
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
 * Copy data members from an AbstractCoordinate to an SimbodyCoordinate01_05.
 *
 * @param aCoordinate AbstractCoordinate to be copied.
 */
void SimbodyCoordinate01_05::copyData(const AbstractCoordinate &aCoordinate)
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
 * Set the data members of this SimbodyCoordinate01_05 to their null values.
 */
void SimbodyCoordinate01_05::setNull(void)
{
	setType("SimbodyCoordinate");
	_motionType = AbstractTransformAxis::Rotational;
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void SimbodyCoordinate01_05::setupProperties(void)
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
 * @param aEngine dynamics engine containing this SimbodyCoordinate01_05.
 */
void SimbodyCoordinate01_05::setup(AbstractDynamicsEngine* aEngine)
{
	// Base class
	AbstractCoordinate::setup(aEngine);

	_engine = dynamic_cast<SimbodyEngine01_05*>(aEngine);

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
SimbodyCoordinate01_05& SimbodyCoordinate01_05::operator=(const SimbodyCoordinate01_05 &aCoordinate)
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
void SimbodyCoordinate01_05::updateFromCoordinate(const AbstractCoordinate &aCoordinate)
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
double SimbodyCoordinate01_05::getValue() const
{
	return 0.0;
}
//done_____________________________________________________________________________
/**
 * Set the value.
 *
 * @param aValue value to change to.
 * @return Whether or not the value was changed.
 */
bool SimbodyCoordinate01_05::setValue(double aValue) { return setValue(aValue, true); }
bool SimbodyCoordinate01_05::setValue(double aValue, bool aRealize)
{
	return true;
}

//_____________________________________________________________________________
/**
 * Set the range min and max.
 *
 * @param aRange range min and man to change to.
 * @return Whether or not the range was changed.
 */
bool SimbodyCoordinate01_05::setRange(double aRange[2])
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
bool SimbodyCoordinate01_05::setRangeMin(double aMin)
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
bool SimbodyCoordinate01_05::setRangeMax(double aMax)
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
bool SimbodyCoordinate01_05::setDefaultValue(double aDefaultValue)
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
void SimbodyCoordinate01_05::setInitialValue(double aInitialValue)
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
bool SimbodyCoordinate01_05::setTolerance(double aTolerance)
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
bool SimbodyCoordinate01_05::setStiffness(double aStiffness)
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
void SimbodyCoordinate01_05::getKeys(string rKeys[]) const
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
OpenSim::Function* SimbodyCoordinate01_05::getRestraintFunction() const
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
OpenSim::Function* SimbodyCoordinate01_05::getMinRestraintFunction(void) const
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
OpenSim::Function* SimbodyCoordinate01_05::getMaxRestraintFunction(void) const
{
	return _maxRestraintFunction;
}

//_____________________________________________________________________________
/**
 * Get the constraint function.
 *
 * @return Pointer to the constraint function.
 */
OpenSim::Function* SimbodyCoordinate01_05::getConstraintFunction() const
{
	return _constraintFunction;
}

//_____________________________________________________________________________
/**
 * Set the constraint function.
 */
void SimbodyCoordinate01_05::setConstraintFunction(const OpenSim::Function *function)
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
bool SimbodyCoordinate01_05::setLocked(bool aLocked)
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
void SimbodyCoordinate01_05::determineType()
{
	// TODO: For now, I'm just setting the type to rotational.  However, when
	// the Simbody classes are really finished, this will have to return
	// something that makes sense.
	_motionType = AbstractTransformAxis::Translational;
}

