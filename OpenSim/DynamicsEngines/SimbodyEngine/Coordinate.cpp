// Coordinate.cpp
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
#include "Coordinate.h"
#include "CoordinateCouplerConstraint.h"
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


// Helper class to construct bounds functions, which returns non-zero values for values
// outside the min and max range
class RangeFunction : public SimTK::Function<1> {
// returns q-qmax for q > qmax and q-qmin for q < qmin
private:
	const double _qmin;
	const double _qmax;

public:
	
	RangeFunction(const double qmin, const double qmax) : _qmin(qmin), _qmax(qmax){
	}

    Vec<1> calcValue(const Vector& x) const {
		double q = x[0];
		Vec1 val(((q-_qmin)-abs(q-_qmin))/2 + ((q-_qmax)+abs(q-_qmax))/2);
		return val;
    }
    Vec<1> calcDerivative(const std::vector<int>& derivComponents, const Vector& x) const {
		if (derivComponents.size() == 1){
			return calcValue(x)/x.norm();
		}
        return Vec1(0);
    }

    int getArgumentSize() const {
        return 1;
    }
    int getMaxDerivativeOrder() const {
        return 2;
    }
};


//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */
Coordinate::Coordinate() :
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
	_isPrescribed(_isPrescribedProp.getValueBool()),
	_prescribedFunction(_prescribedFunctionProp.getValueObjPtrRef())
{
	setNull();
	setupProperties();
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
Coordinate::~Coordinate()
{
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aCoordinate Coordinate to be copied.
 */
Coordinate::Coordinate(const Coordinate &aCoordinate) :
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
	_isPrescribed(_isPrescribedProp.getValueBool()),
	_prescribedFunction(_prescribedFunctionProp.getValueObjPtrRef())
{
	setNull();
	setupProperties();
	copyData(aCoordinate);
}

//_____________________________________________________________________________
/**
 * Copy constructor from an AbstractCoordinate.
 *
 * @param aCoordinate Coordinate to be copied.
 */
Coordinate::Coordinate(const AbstractCoordinate &aCoordinate) :
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
	_isPrescribed(_isPrescribedProp.getValueBool()),
	_prescribedFunction(_prescribedFunctionProp.getValueObjPtrRef())
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
 * @return Pointer to a copy of this Coordinate.
 */
Object* Coordinate::copy() const
{
	Coordinate *gc = new Coordinate(*this);
	return(gc);
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Copy data members from one Coordinate to another.
 *
 * @param aCoordinate Coordinate to be copied.
 */
void Coordinate::copyData(const Coordinate &aCoordinate)
{
	_defaultValue = aCoordinate.getDefaultValue();
	_initialValue = aCoordinate.getDefaultValue();
	_tolerance = aCoordinate.getTolerance();
	_stiffness = aCoordinate._stiffness;
	_range = aCoordinate._range;
	_keys = aCoordinate._keys;
	_clamped = aCoordinate._clamped;
	_locked = aCoordinate._locked;
	_isPrescribed = aCoordinate._isPrescribed;
	_restraintFunction = (Function*)Object::SafeCopy(aCoordinate._restraintFunction);
	_minRestraintFunction = (Function*)Object::SafeCopy(aCoordinate._minRestraintFunction);
	_maxRestraintFunction = (Function*)Object::SafeCopy(aCoordinate._maxRestraintFunction);
	_restraintActive = aCoordinate._restraintActive;
	_prescribedFunction = (Function*)Object::SafeCopy(aCoordinate._prescribedFunction);
	_dynamicsEngine = aCoordinate._dynamicsEngine;
	_motionType = aCoordinate._motionType;
}

//_____________________________________________________________________________
/**
 * Copy data members from an AbstractCoordinate to a Coordinate.
 *
 * @param aCoordinate AbstractCoordinate to be copied.
 */
void Coordinate::copyData(const AbstractCoordinate &aCoordinate)
{
	_defaultValue = aCoordinate.getDefaultValue();
	_initialValue = aCoordinate.getDefaultValue();
	_tolerance = aCoordinate.getTolerance();
	_stiffness = aCoordinate.getStiffness();
	aCoordinate.getRange(&_range[0]);
	//TODO _keys = aCoordinate._keys;
	_clamped = aCoordinate.getClamped();
	_locked = aCoordinate.getLocked();
	_isPrescribed = aCoordinate.isPrescribed();
	_restraintFunction = (Function*)Object::SafeCopy(aCoordinate.getRestraintFunction());
	_minRestraintFunction = (Function*)Object::SafeCopy(aCoordinate.getMinRestraintFunction());
	_maxRestraintFunction = (Function*)Object::SafeCopy(aCoordinate.getMaxRestraintFunction());
	_restraintActive = aCoordinate.isRestraintActive();
	_prescribedFunction = (Function*)Object::SafeCopy(aCoordinate.getPrescribedFunction());
	_dynamicsEngine = aCoordinate.getDynamicsEngine();
	_motionType = aCoordinate.getMotionType();
}

//_____________________________________________________________________________
/**
 * Set the data members of this Coordinate to their null values.
 */
void Coordinate::setNull(void)
{
	setType("Coordinate");

	_dynamicsEngine = NULL;
	_joint = NULL;

	// By default, the motion type is Translational.
	// If the q is used for a rotation, it is set to Rotational.
	// This occurs in SimbodyEngine::connectBodies() as each joint and
	// body are added to the Simbody matter subsystem.
	_motionType = AbstractTransformAxis::Translational;
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void Coordinate::setupProperties(void)
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

	_prescribedFunctionProp.setName("prescribed_function");
	_propertySet.append(&_prescribedFunctionProp);
}

void Coordinate::createConstraintsForLockClampPrescribed()
{
	//create lock constraint automatically
	// Define the locked value for the constraint as a function
	SimTK::Function<1>::Constant *lockFunction = new SimTK::Function<1>::Constant(Vec1(getValue()), 1); 
	// The underlying SimTK constraint
	SimTK::Constraint *lock;
	lock = new SimTK::Constraint::PrescribedMotion( 
			getEngine()->_system->updMatterSubsystem(), 
			lockFunction, 
			_bodyIndex, 
			SimTK::MobilizerQIndex(_mobilityIndex));
	_lockedConstraintIndex = lock->getConstraintIndex();

			
	SimTK::Constraint *prescribe = NULL;
	if(_prescribedFunction != NULL){
		//create prescribed motion constraint automatically
		prescribe = new SimTK::Constraint::PrescribedMotion( 
				getEngine()->_system->updMatterSubsystem(), 
				_prescribedFunction->createSimTKFunction(), 
				_bodyIndex, 
				SimTK::MobilizerQIndex(_mobilityIndex));
		_prescribedConstraintIndex = prescribe->getConstraintIndex();
	}
	else{
		_isPrescribed = false;
	}

	//TODO add clamping

	//Invalidate the state since it needs to include new constraints
	getEngine()->setInvalid();
}

//_____________________________________________________________________________
/**
 * Perform some set up functions that happen after the
 * object has been deserialized or copied.
 *
 * @param aEngine dynamics engine containing this Coordinate.
 */
void Coordinate::setup(AbstractDynamicsEngine* aEngine)
{
	// Base class
	AbstractCoordinate::setup(aEngine);

	//_engine = dynamic_cast<SimbodyEngine*>(aEngine);

	// Make sure the range is min to max.
	if (_range[1] < _range[0]){
		throw(Exception("Maximum coordinate range less than minimum."));
	}

	// Make sure the default value is in the range
	if (_defaultValue < _range[0])
		_defaultValue = _range[0];
	else if (_defaultValue > _range[1])
		_defaultValue = _range[1];

	createConstraintsForLockClampPrescribed();
}

void Coordinate::initializeState(State& completeState)
{
	// If the user specified a default value but not a value, set the
	// current value to the default value

	// Cannot enforce the constraint, since state of constraints may still be undefined
	setValue(_defaultValue, completeState, false);
	setIsPrescribed(_isPrescribed, completeState);
	setClamped(_clamped, completeState);
	// Locking takes precedence if all joint constraints are ON
	setLocked(_locked, completeState);
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
Coordinate& Coordinate::operator=(const Coordinate &aCoordinate)
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
void Coordinate::updateFromCoordinate(const AbstractCoordinate &aCoordinate)
{
	if (!aCoordinate.getValueUseDefault())
		setValue(aCoordinate.getValue(), false);

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
//-----------------------------------------------------------------------------
// JOINT
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the joint to which this coordinate belongs.
 *
 * @param aowningJoint Joint to which this coordinate belongs.
 */
void Coordinate::setJoint(Joint *aOwningJoint)
{
	_joint = aOwningJoint;
}
//_____________________________________________________________________________
/**
 * Get the joint to which this coordinate belongs.
 *
 * @return Joint to which this coordinate belongs.
 */
Joint* Coordinate::getJoint() const
{
	return(_joint);
}

//-----------------------------------------------------------------------------
// VALUE
//-----------------------------------------------------------------------------
//done_____________________________________________________________________________
/**
 * Get the value.
 *
 * @return The current value of the coordinate.
 */
double Coordinate::getValue() const
{
	if (getEngine()->isInvalid())
		return _defaultValue;
	else
		return getEngine()->_system->getMatterSubsystem().getMobilizedBody(_bodyIndex).getOneQ(getEngine()->_s,_mobilityIndex);
}
//done_____________________________________________________________________________
/**
 * Set the value.
 *
 * @param aValue value to change to.
 * @return Whether or not the value was changed.
 */
bool Coordinate::setValue(double aValue) { return setValue(aValue, getEngine()->_s); }
bool Coordinate::setValue(double aValue, bool aRealize){ 
		return setValue(aValue,  getEngine()->_s, aRealize); 
}
bool Coordinate::setValue(double aValue, State& theState, bool aRealize)
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
		// JPL 11/19/07: For some reason, when solving a frame of marker data,
		// Ipopt needs to be able to set a coordinate to its current value
		// or it will crash (so even a tolerance of 0.0 does not work). So
		// for now, do not check against tolerance.
		//	if (DABS(aValue - getValue()) > _tolerance)
		if (1)
		{
			if (_locked) {
				cout<<"Coordinate.setValue: WARN- coordinate "<<getName();
				cout<<" is locked. Unable to change its value." << endl;
				return false;
			}
			getEngine()->resetBodyAndMobilityForceVectors();
			getEngine()->_system->getMatterSubsystem().getMobilizedBody(_bodyIndex).setOneQ(theState,_mobilityIndex,aValue);

			// The Q that was set might not satisfy constraints, so if enforceConstraints then project configuration
			if (aRealize) {
				getEngine()->_system->realize(theState, Stage::Position);

				if (isConstrained()){
					double* coordinatesAndSpeeds = new double[getEngine()->getNumCoordinates() + getEngine()->getNumSpeeds()];
					getEngine()->getConfiguration(coordinatesAndSpeeds);

					// Update dependent cooordinates in case we are coupled.
					getEngine()->computeConstrainedCoordinates(coordinatesAndSpeeds);

					// Set the configuration that satisfies the coupled coordinates
					getEngine()->setConfiguration(coordinatesAndSpeeds);

					//project states onto to constraint manifold to make sure states satisfy ALL constraints
					getEngine()->projectConfigurationToSatisfyConstraints(coordinatesAndSpeeds, _tolerance);

					delete[] coordinatesAndSpeeds;
				}
			}

			// TODO: use Observer mechanism for _jointList, _pathList, and muscles
			ActuatorSet* act = getEngine()->getModel()->getActuatorSet();
			for(int i=0; i<act->getSize(); i++) {
				AbstractMuscle* sm = dynamic_cast<AbstractMuscle*>(act->get(i));
				if(sm) sm->invalidatePath();
			}
		}
	} else {
		cout << "Coordinate.setValue: WARN- Attempting to set coordinate " << getName() << " to a value (" <<
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
bool Coordinate::setRange(double aRange[2])
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
bool Coordinate::setRangeMin(double aMin)
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
bool Coordinate::setRangeMax(double aMax)
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
bool Coordinate::setDefaultValue(double aDefaultValue)
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
void Coordinate::setInitialValue(double aInitialValue)
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
bool Coordinate::setTolerance(double aTolerance)
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
bool Coordinate::setStiffness(double aStiffness)
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
void Coordinate::getKeys(string rKeys[]) const
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
OpenSim::Function* Coordinate::getRestraintFunction() const
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
OpenSim::Function* Coordinate::getMinRestraintFunction(void) const
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
OpenSim::Function* Coordinate::getMaxRestraintFunction(void) const
{
	return _maxRestraintFunction;
}

//_____________________________________________________________________________
/**
 * Get the prescribed motion function.
 *
 * @return Pointer to the constraint function.
 */
OpenSim::Function* Coordinate::getPrescribedFunction() const
{
	return _prescribedFunction;
}

//_____________________________________________________________________________
/**
 * Set the prescribed motion function.
 */
void Coordinate::setPrescribedFunction(const OpenSim::Function *function)
{
	_prescribedFunction = (Function*)function->copy();
}

//_____________________________________________________________________________
/**
 *  Determine if the the coordinate is constrained or not.
 *  Specifically, is it a dependent coordinate in any of the constraints?\
 *  If so return true, false otherwise.
 */
bool Coordinate::isConstrained() const
{
	for(int i=0; i<getEngine()->getConstraintSet()->getSize(); i++){
		AbstractConstraint *aConstraint = getEngine()->getConstraintSet()->get(i);
		if(aConstraint->getType() == "CoordinateCouplerConstraint"){
			CoordinateCouplerConstraint *coupler = dynamic_cast<CoordinateCouplerConstraint *>(aConstraint);
			if (coupler->getDependentCoordinateName() == this->_name)
				return true;
		}
	}
	return false;
}


//-----------------------------------------------------------------------------
// LOCK
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set whether or not this coordinate is locked.
 * A prescribed constraint is used to lock the joint at the dynamics level.
 * Creates the constraint the first time the lock is applied and require
 * update to Simbody engine. Lock can be turned on/off during the simulation.
 * If lock is applied after clamping or prescribed motion it takes precedence.
 *
 * @param aLocked If true the joint is locked; if false the joint is unlocked.
 */

bool Coordinate::setLocked(bool aLocked) 
	{return setLocked(aLocked, getEngine()->_s);}
bool Coordinate::setLocked(bool aLocked, State& theState)
{
	// Do nothing if the same
	if(aLocked == getLocked()) return true;
	
	SimTK::Constraint *lock = NULL;

	// Get constraint
	if(int(_lockedConstraintIndex) != SimTK::InvalidIndex){
		lock = &getEngine()->_system->updMatterSubsystem().updConstraint(_lockedConstraintIndex);
	}
	else{
		string msg = "Lock constraint for coordinate could not be found.";
		throw Exception(msg,__FILE__,__LINE__);
	}

	// Now enable if locked otherwise disable
	if(aLocked){	
		lock->enable(theState);
		//Cannot be locked and have prescribed motion and/or clamping
		setIsPrescribed(false, theState);
		setClamped(false, theState);
	}
	else
		lock->disable(theState);

	_locked = aLocked;

	return true;
}

/**
 * Get whether or not this coordinate is locked.
 * Calls the underlying constraint at the dynamics level.
 *
 * @return true if the coordinate is locked and false if unlocked.
 */
bool Coordinate::getLocked() const
{
	if(int(_lockedConstraintIndex) != SimTK::InvalidIndex){
		bool disabled = getEngine()->_system->updMatterSubsystem().getConstraint(_lockedConstraintIndex).isDisabled(getEngine()->_s);
		return !disabled;
	}
	else{
		return false;
	}
}

//-----------------------------------------------------------------------------
// PRESCRIBED MOTION
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set whether or not this coordinate is being prescribed.
 * A prescribed constraint is used specify motion at the dynamics level.
 * Creates the constraint the first time prescribed motion is applied and requires
 * an update to Simbody engine. Prescribed motion can be turned on/off during the
 * simulation. If isPrescribed is set after clamping or locking it takes precedence-
 * and clamped and locked are false.
 *
 * @param isPrescribed If true the coordinate is prescribed; if false not prescribed.
 */
void Coordinate::setIsPrescribed(bool isPrescribed) 
	{ setIsPrescribed(isPrescribed, getEngine()->_s);}
void Coordinate::setIsPrescribed(bool isPrescribed, State& theState)
{
	// Do nothing if the same
	if(isPrescribed == _isPrescribed) return;
	
	// The underlying SimTK constraint
	SimTK::Constraint *prescribe = NULL;

	// Get constraint
	if(int(_prescribedConstraintIndex) != SimTK::InvalidIndex){
		//get constraint
		prescribe = &getEngine()->_system->updMatterSubsystem().updConstraint(_prescribedConstraintIndex);
	}
	else{
		string msg = "Preecribed motion for coordinate not found.";
		throw Exception(msg,__FILE__,__LINE__);
	}


	// Now enable if prescribed motion constraint otherwise disable
	if(isPrescribed){	
		prescribe->enable(theState);
		setLocked(false, theState);
		setClamped(false, theState);
	}
	else
		prescribe->disable(theState);

	_isPrescribed = isPrescribed;
}

//-----------------------------------------------------------------------------
// CLAMP
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set whether or not this coordinate is clamped within a range of values.
 * A coupler constraint is used to stop the coordinate at the dynamics level.
 * Creates the constraint the first time the lock is applied and require
 * update to Simbody engine. Clamp can be turned on/off during the simulation.
 * If clamp is set after locking or prescribed motion it takes precedence.
 *
 * @param aLocked If true the joint is locked; if false the joint is unlocked.
 */

bool Coordinate::setClamped(bool aClamped)
{	return setClamped(aClamped, getEngine()->_s);}
bool Coordinate::setClamped(bool aClamped, State& theState)
{
	return false;
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
void Coordinate::determineType()
{
	// TODO: For now, I'm just setting the type to rotational.  However, when
	// the Simbody classes are really finished, this will have to return
	// something that makes sense.

	//Note!
	//This functionality is now taken care of by SimbodyEngine::connectBodies.
	//As each joint is encountered, the motion type is set on each q.
}

