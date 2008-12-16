// SimmCoordinate.cpp
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
#include "SimmCoordinate.h"
#include <OpenSim/Simulation/Model/AbstractTransformAxis.h>
#include <OpenSim/Simulation/Model/AbstractDof01_05.h>
#include <OpenSim/Simulation/Model/AbstractJoint.h>
#include "SimmKinematicsEngine.h"
#include "SimmJoint.h"
#include <OpenSim/Simulation/Model/AbstractDynamicsEngine.h>
#include <OpenSim/Simulation/Model/ActuatorSet.h>
#include <OpenSim/Simulation/Model/AbstractMuscle.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Common/rdMath.h>
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
SimmCoordinate::SimmCoordinate() :
   _defaultValue(_defaultValueProp.getValueDbl()),
   _value(_valueProp.getValueDbl()),
   _tolerance(_toleranceProp.getValueDbl()),
   _stiffness(_stiffnessProp.getValueDbl()),
	_range(_rangeProp.getValueDblArray()),
	_keys(_keysProp.getValueStrArray()),
	_clamped(_clampedProp.getValueBool()),
	_locked(_lockedProp.getValueBool()),
	_restraintFunction(_restraintFunctionProp.getValueObjPtrRef()),
	_minRestraintFunction(_minRestraintFunctionProp.getValueObjPtrRef()),
	_maxRestraintFunction(_maxRestraintFunctionProp.getValueObjPtrRef()),
	_restraintActive(_restraintActiveProp.getValueBool()),
	_jointList(0),
	_pathList(0),
	_motionType(AbstractTransformAxis::Rotational)
{
	setNull();
	setupProperties();
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
SimmCoordinate::~SimmCoordinate()
{
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aCoordinate SimmCoordinate to be copied.
 */
SimmCoordinate::SimmCoordinate(const SimmCoordinate &aCoordinate) :
   AbstractCoordinate(aCoordinate),
	_defaultValue(_defaultValueProp.getValueDbl()),
   _value(_valueProp.getValueDbl()),
   _tolerance(_toleranceProp.getValueDbl()),
   _stiffness(_stiffnessProp.getValueDbl()),
	_range(_rangeProp.getValueDblArray()),
	_keys(_keysProp.getValueStrArray()),
	_clamped(_clampedProp.getValueBool()),
	_locked(_lockedProp.getValueBool()),
	_restraintFunction(_restraintFunctionProp.getValueObjPtrRef()),
	_minRestraintFunction(_minRestraintFunctionProp.getValueObjPtrRef()),
	_maxRestraintFunction(_maxRestraintFunctionProp.getValueObjPtrRef()),
	_restraintActive(_restraintActiveProp.getValueBool()),
	_jointList(0),
	_pathList(0),
	_motionType(AbstractTransformAxis::Rotational)
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
 * @return Pointer to a copy of this SimmCoordinate.
 */
Object* SimmCoordinate::copy() const
{
	SimmCoordinate *gc = new SimmCoordinate(*this);
	return(gc);
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Copy data members from one SimmCoordinate to another.
 *
 * @param aCoordinate SimmCoordinate to be copied.
 */
void SimmCoordinate::copyData(const SimmCoordinate &aCoordinate)
{
	_defaultValue = aCoordinate.getDefaultValue();
	_value = aCoordinate.getValue();
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
	_jointList = aCoordinate._jointList;
	_pathList = aCoordinate._pathList;
	_motionType = aCoordinate._motionType;
}

//_____________________________________________________________________________
/**
 * Set the data members of this SimmCoordinate to their null values.
 */
void SimmCoordinate::setNull(void)
{
	setType("SimmCoordinate");
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void SimmCoordinate::setupProperties(void)
{
	_defaultValueProp.setComment("Default value for a coordinate.");
	_defaultValueProp.setName("default_value");
	_defaultValueProp.setValue(0.0);
	_propertySet.append(&_defaultValueProp);

	_valueProp.setComment("Value of the coordinate.");
	_valueProp.setName("value");
	_valueProp.setValue(0.0);
	_propertySet.append(&_valueProp);

	_toleranceProp.setComment("Tolerance for a coordinate.");
	_toleranceProp.setName("tolerance");
	_toleranceProp.setValue(rdMath::SMALL);
	_propertySet.append(&_toleranceProp);

	_stiffnessProp.setComment("Stiffness of a coordinate.");
	_stiffnessProp.setName("stiffness");
	_stiffnessProp.setValue(0.0);
	_propertySet.append(&_stiffnessProp);

	_rangeProp.setComment("Allowd range for a coordinate.");
	const double defaultRange[] = {-999999.9, 999999.9};
	_rangeProp.setName("range");
	_rangeProp.setValue(2, defaultRange);
	_rangeProp.setAllowableArraySize(2);
	_propertySet.append(&_rangeProp);

	_keysProp.setComment("Computer keyboard keys that can be used to manipulate "
		"in a graphical application.");
	_keysProp.setName("keys");
	_propertySet.append(&_keysProp);

	_clampedProp.setComment("Flag (true or false) indicating whether a coordinate "
		"is not permitted outside its range.");
	_clampedProp.setName("clamped");
	_clampedProp.setValue(true);
	_propertySet.append(&_clampedProp);

	_lockedProp.setComment("Flag (true or false) indicating whether a coordinate "
		"is fixed or locked at its current value.");
	_lockedProp.setName("locked");
	_lockedProp.setValue(false);
	_propertySet.append(&_lockedProp);

	_restraintFunctionProp.setComment("Pointer to a restraint function that applies generalized forces "
		"at a joint to keep the coordinate from moving outside its range. ");
	_restraintFunctionProp.setName("restraint_function");
	_propertySet.append(&_restraintFunctionProp);

	_minRestraintFunctionProp.setComment("Minimum restraint function.");
	_minRestraintFunctionProp.setName("min_restraint_function");
	_propertySet.append(&_minRestraintFunctionProp);

	_maxRestraintFunctionProp.setComment("Maximum restraint function.");
	_maxRestraintFunctionProp.setName("max_restraint_function");
	_propertySet.append(&_maxRestraintFunctionProp);

	_restraintActiveProp.setComment("Flag (true or false) indicating whether "
		"or not the restraint function is active.");
	_restraintActiveProp.setName("restraint_active");
	_restraintActiveProp.setValue(true);
	_propertySet.append(&_restraintActiveProp);
}

//_____________________________________________________________________________
/**
 * Perform some set up functions that happen after the
 * object has been deserialized or copied.
 *
 * @param aEngine dynamics engine containing this SimmCoordinate.
 */
void SimmCoordinate::setup(AbstractDynamicsEngine* aEngine)
{
	// Base class;
	AbstractCoordinate::setup(aEngine);

	// Make sure the range is min to max.
	if (_range[1] < _range[0])
	{
		double tmp = _range[0];
		_range[0] = _range[1];
		_range[1] = tmp;
	}

	// Make sure the default value is in the range
	if (_defaultValue < _range[0])
		_defaultValue = _range[0];
	else if (_defaultValue > _range[1])
		_defaultValue = _range[1];

	// Make sure the starting value is in the range
	if (_value < _range[0])
		_value = _range[0];
	else if (_value > _range[1])
		_value = _range[1];

	// If the user specified a default value but not a value, set the
	// current value to the default value, whether or not the coordinate
	// is locked.
	if (!_defaultValueProp.getUseDefault() && _valueProp.getUseDefault())
	{
		bool lockedState = getLocked();
		setLocked(false);
		setValue(_defaultValue);
		setLocked(lockedState);
	}

	_tolerance=1e-6;	// This should be set during construction but doesn't work due to useDefault.
	// can't call this here now that coordinates are stored in AbstractDynamicsEngine,
	// because this function is called before SimmKinematicsEngine::createCoordinateJointLists()
	//determineType();
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
SimmCoordinate& SimmCoordinate::operator=(const SimmCoordinate &aCoordinate)
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
void SimmCoordinate::determineType()
{
   AbstractDof01_05* dof;
	AbstractJoint* jnt;

   if ((dof = (dynamic_cast<SimmKinematicsEngine*>(_dynamicsEngine))->findUnconstrainedSimmDof(*this, jnt)))
   {
		if (dof->getMotionType() == AbstractDof01_05::Rotational)
		   _motionType = AbstractTransformAxis::Rotational;
		else
		   _motionType = AbstractTransformAxis::Translational;
   }
   else
   {
		_motionType = AbstractTransformAxis::Translational;

      for (int i = 0; i < _jointList.getSize(); i++)
      {
			SimmJoint* joint = dynamic_cast<SimmJoint*>(_jointList[i]);
			DofSet01_05* dofs = joint->getDofSet();

			for (int j = 0; j < dofs->getSize(); j++)
         {
				if (dofs->get(j)->getCoordinate() == this && dofs->get(j)->getMotionType() == AbstractDof01_05::Rotational)
            {
					/* You've found one rotational DOF: set the coordinate's type
					 * to rotational and you're done.
					 */
               _motionType = AbstractTransformAxis::Rotational;
               return;
            }
         }
      }
   }
}

//_____________________________________________________________________________
/**
 * Update an existing coordinate with parameter values from a
 * new one, but only for the parameters that were explicitly
 * specified in the XML node.
 *
 * @param aCoordinate coordinate to update from
 */
void SimmCoordinate::updateFromCoordinate(const AbstractCoordinate &aCoordinate)
{
	if (!aCoordinate.getDefaultValueUseDefault())
		setDefaultValue(aCoordinate.getDefaultValue());

	if (!aCoordinate.getValueUseDefault())
	{
		setValue(aCoordinate.getValue());
		_valueProp.setUseDefault(false);
	}

	if (!aCoordinate.getRangeUseDefault())
	{
		setRangeMin(aCoordinate.getRangeMin());
		setRangeMax(aCoordinate.getRangeMax());
		_rangeProp.setUseDefault(false);
	}

	if (!aCoordinate.getToleranceUseDefault())
	{
		setTolerance(aCoordinate.getTolerance());
		_toleranceProp.setUseDefault(false);
	}

	if (!aCoordinate.getStiffnessUseDefault())
	{
		setStiffness(aCoordinate.getStiffness());
		_stiffnessProp.setUseDefault(false);
	}

	if (!aCoordinate.getClampedUseDefault())
	{
		setClamped(aCoordinate.getClamped());
		_clampedProp.setUseDefault(false);
	}

	if (!aCoordinate.getLockedUseDefault())
	{
		setLocked(aCoordinate.getLocked());
		_lockedProp.setUseDefault(false);
	}

#if 0
	// TODO
	if (!aCoordinate._restraintFunctionProp.getUseDefault())
	{
		_restraintFunction = aCoordinate._restraintFunction;
		_restraintFunctionProp.setUseDefault(false);
	}

	if (!aCoordinate._maxRestraintFunctionProp.getUseDefault())
	{
		_maxRestraintFunction = aCoordinate._maxRestraintFunction;
		_maxRestraintFunctionProp.setUseDefault(false);
	}

	if (!aCoordinate._restraintActiveProp.getUseDefault())
	{
		_restraintActive = aCoordinate._restraintActive;
		_restraintActiveProp.setUseDefault(false);
	}
#endif
}

//=============================================================================
// GET AND SET
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the value.
 *
 * @param aValue value to change to.
 * @return Whether or not the value was changed.
 */
bool SimmCoordinate::setValue(double aValue)
{
	//// pull value into range if it's off by tolerance due to roundoff
	//if (_clamped) {
	//	if (aValue < _range[0] && (_range[0]-aValue < _tolerance))
	//		aValue = _range[0];
	//	else if (aValue > _range[1] && (aValue-_range[1] < _tolerance))
	//		aValue = _range[1];
	//}
	// pull value into range if it's clamped
	if (_clamped) {
		if (aValue < _range[0])
			aValue = _range[0];
		else if (aValue > _range[1])
			aValue = _range[1];
	}

	if ((aValue >= _range[0] && aValue <= _range[1]) || !_clamped) {
		// Check if the value is sufficiently different
		// if (DABS(aValue - _value) > _tolerance)
		if (1)
		{
			if (_locked) {
				//cout << "SimmCoordinate.setValue: WARN- Coordinate " << getName() << " is locked. Unable to change its value." << endl;
				return false;
			}

			_value = aValue;

			for (int i = 0; i < _jointList.getSize(); i++)
				_jointList[i]->invalidate();

			// JPL 1/11/08: this should no longer be necessary.
			// When a joint is invalidated it invalidates all of the paths that use it, and all of the muscles.
			//int pListSize = _pathList.getSize();
			//for (int i = 0; i < pListSize; i++)
			//	_pathList[i]->invalidate();
			//ActuatorSet* act = getDynamicsEngine()->getModel()->getActuatorSet();
			//for (int i = 0; i < act->getSize(); i++) {
			//	AbstractMuscle* sm = dynamic_cast<AbstractMuscle*>(act->get(i));
			//	if (sm)
			//		sm->invalidatePath();
			//}
		}
	} else {
		//cout << "SimmCoordinate.setValue: WARN- Attempting to set coordinate " << getName() << " to a value (" <<
		//	aValue << ") outside its range (" << _range[0] << " to " << _range[1] << ")" << endl;
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
bool SimmCoordinate::setRange(double aRange[2])
{
	if (aRange[1] >= aRange[0])
	{
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
bool SimmCoordinate::setRangeMin(double aMin)
{
	if (aMin <= _range[1])
	{
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
bool SimmCoordinate::setRangeMax(double aMax)
{
	if (aMax >= _range[0])
	{
		_range[1] = aMax;
		return true;
	}

	return false;
}

//_____________________________________________________________________________
/**
 * Set the default value.
 *
 * @param aRange default value to change to.
 * @return Whether or not the default value was changed.
 */
bool SimmCoordinate::setDefaultValue(double aDefaultValue)
{
	if (aDefaultValue >= _range[0] && aDefaultValue <= _range[1])
	{
		_defaultValue = aDefaultValue;
		return true;
	}

	return false;
}

//_____________________________________________________________________________
/**
 * Set the tolerance.
 *
 * @param aTolerance tolerance to change to.
 * @return Whether or not the tolerance was changed.
 */
bool SimmCoordinate::setTolerance(double aTolerance)
{
	if (aTolerance >= 0.0)
	{
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
bool SimmCoordinate::setStiffness(double aStiffness)
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
void SimmCoordinate::getKeys(string rKeys[]) const
{
	for (int i = 0; i < _keys.getSize(); i++)
		rKeys[i] = _keys[i];
}

void SimmCoordinate::setKeys(const OpenSim::Array<std::string>& aKeys)
{
	_keys.setSize(0);
	for (int i = 0; i < aKeys.getSize(); i++)
		_keys.append(aKeys[i]);
}

void SimmCoordinate::setRestraintActive(bool aActive)
{
	_restraintActive = aActive;
}

//_____________________________________________________________________________
/**
 * Get the restraint function used to keep this coordinate inside its range.
 *
 * @return Pointer to the restraint function.
 */
Function* SimmCoordinate::getRestraintFunction() const
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
Function* SimmCoordinate::getMinRestraintFunction(void) const
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
Function* SimmCoordinate::getMaxRestraintFunction(void) const
{
	return _maxRestraintFunction;
}
