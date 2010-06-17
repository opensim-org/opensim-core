// Coordinate.cpp
// Author: Frank C. Anderson, Ajay Seth, Jeffrey A. Reinbolt
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
#include <OpenSim/Common/IO.h>
#include <OpenSim/Common/Function.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/SimbodyEngine/Joint.h>


//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace SimTK;
using namespace OpenSim;

/**
 * A Function that takes a value and returns it.
 * The value is modifiable.
 */
template <class T>
class ModifiableConstant : public SimTK::Function_<T> {
public:
    ModifiableConstant(T value, int argumentSize) : value(value), argumentSize(argumentSize) {
    }
    T calcValue(const Vector& x) const {
        assert(x.size() == argumentSize);
        return value;
    }

	T calcDerivative(const std::vector<int>& derivComponents, const Vector& x) const {
        return calcDerivative(SimTK::ArrayViewConst_<int>(derivComponents),x);
    }
	T calcDerivative(const SimTK::Array_<int>& derivComponents, const Vector& x) const {
        return static_cast<T>(0);
    }

    virtual int getArgumentSize() const {
        return argumentSize;
    }
    int getMaxDerivativeOrder() const {
        return std::numeric_limits<int>::max();
    }

	void setValue(T newValue){
		value = newValue;
	}
private:
    const int argumentSize;
    T value;
};

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */
Coordinate::Coordinate() :
	_motionTypeName(_motionTypeNameProp.getValueStr()),
   _defaultValue(_defaultValueProp.getValueDbl()),
   _defaultSpeedValue(_defaultSpeedValueProp.getValueDbl()),
   _initialValue(_initialValueProp.getValueDbl()),
	_range(_rangeProp.getValueDblArray()),
	_clamped(_clampedProp.getValueBool()),
	_locked(_lockedProp.getValueBool()),
	_isPrescribed(_isPrescribedProp.getValueBool()),
	_prescribedFunction(_prescribedFunctionProp.getValueObjPtrRef())
{
	setNull();
	setupProperties();
}

//_____________________________________________________________________________
/**
 * Constructor.
 */
Coordinate::Coordinate(const std::string &aName, MotionType aMotionType,double aRangeMin,double aRangeMax) :
	_motionTypeName(_motionTypeNameProp.getValueStr()),
	_defaultValue(_defaultValueProp.getValueDbl()),
	_initialValue(_initialValueProp.getValueDbl()),
	_defaultSpeedValue(_defaultSpeedValueProp.getValueDbl()),
	_range(_rangeProp.getValueDblArray()),
	_clamped(_clampedProp.getValueBool()),
	_locked(_lockedProp.getValueBool()),
	_isPrescribed(_isPrescribedProp.getValueBool()),
	_prescribedFunction(_prescribedFunctionProp.getValueObjPtrRef())
{
	setNull();
	setupProperties();
	setName(aName);
	setMotionType(aMotionType);
	setRangeMin(aRangeMin);
	setRangeMax(aRangeMax);
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
     ModelComponent(aCoordinate),
	_motionTypeName(_motionTypeNameProp.getValueStr()),
	_defaultValue(_defaultValueProp.getValueDbl()),   
	_defaultSpeedValue(_defaultSpeedValueProp.getValueDbl()),
    _initialValue(_initialValueProp.getValueDbl()),
	_range(_rangeProp.getValueDblArray()),
	_clamped(_clampedProp.getValueBool()),
	_locked(_lockedProp.getValueBool()),
	_isPrescribed(_isPrescribedProp.getValueBool()),
	_prescribedFunction(_prescribedFunctionProp.getValueObjPtrRef())
{
	setNull();
	setupProperties();
	copyData(aCoordinate);
}

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
	_defaultSpeedValue = aCoordinate.getDefaultSpeedValue();
	_initialValue = aCoordinate.getDefaultValue();
	_range = aCoordinate._range;
	_clamped = aCoordinate._clamped;
	_locked = aCoordinate._locked;
	_isPrescribed = aCoordinate._isPrescribed;
	_prescribedFunction = (Function*)Object::SafeCopy(aCoordinate._prescribedFunction);
	_model = aCoordinate._model;
	_motionType = aCoordinate._motionType;
	_motionTypeName = aCoordinate._motionTypeName;
}


//_____________________________________________________________________________
/**
 * Set the data members of this Coordinate to their null values.
 */
void Coordinate::setNull(void)
{
	setType("Coordinate");

	_joint = NULL;
	_lockFunction = NULL;

	// By default, the motion type is Rotational.
	_motionType = Rotational;

	_lockedWarningGiven=false;
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void Coordinate::setupProperties(void)
{
	_motionTypeNameProp.setComment("Cooridnate can describe rotational, translational, or coupled values. Defaults to rotational.");
	_motionTypeNameProp.setName("motion_type");
	_motionTypeNameProp.setValue("rotational");
	_propertySet.append(&_motionTypeNameProp);

	_defaultValueProp.setName("default_value");
	_defaultValueProp.setValue(0.0);
	_propertySet.append(&_defaultValueProp);

	_defaultSpeedValueProp.setName("default_speed_value");
	_defaultSpeedValueProp.setValue(0.0);
	_propertySet.append(&_defaultSpeedValueProp);

	_initialValueProp.setName("initial_value");
	_initialValueProp.setValue(0.0);
	_propertySet.append(&_initialValueProp);


	const double defaultRange[] = {-999999.9, 999999.9};
	_rangeProp.setName("range");
	_rangeProp.setValue(2, defaultRange);
	_propertySet.append(&_rangeProp);

	_clampedProp.setName("clamped");
	_clampedProp.setValue(false);
	_propertySet.append(&_clampedProp);

	_lockedProp.setName("locked");
	_lockedProp.setValue(false);
	_propertySet.append(&_lockedProp);

	_prescribedFunctionProp.setName("prescribed_function");
	_propertySet.append(&_prescribedFunctionProp);
}

void Coordinate::createSystem(SimTK::MultibodySystem& system) const
{
	//create lock constraint automatically
	// The underlying SimTK constraint
	SimTK::Constraint *lock;
	lock = new SimTK::Constraint::PrescribedMotion( 
			_model->updMatterSubsystem(), 
			_lockFunction, 
			_bodyIndex, 
			SimTK::MobilizerQIndex(_mobilityIndex));

	// Beyond the const Component get the index so we can access the SimTK::Constraint later
	Coordinate* mutableThis = const_cast<Coordinate *>(this);
	mutableThis->_lockedConstraintIndex = lock->getConstraintIndex();

			
	SimTK::Constraint *prescribe = NULL;
	if(_prescribedFunction != NULL){
		//create prescribed motion constraint automatically
		prescribe = new SimTK::Constraint::PrescribedMotion( 
				_model->updMatterSubsystem(), 
				_prescribedFunction->createSimTKFunction(), 
				_bodyIndex, 
				SimTK::MobilizerQIndex(_mobilityIndex));
		mutableThis->_prescribedConstraintIndex = prescribe->getConstraintIndex();
	}
	else{
		_isPrescribed = false;
	}

	//TODO add clamping



}

//_____________________________________________________________________________
/**
 * Perform some set up functions that happen after the
 * object has been deserialized or copied.
 *
 * @param aModel OpenSim model containing this Coordinate.
 */
void Coordinate::setup(Model& aModel)
{
	// Base class
	ModelComponent::setup(aModel);

	if((IO::Lowercase(_motionTypeName) == "rotational") || _motionTypeName == "")
		_motionType = Rotational;
	else if(IO::Lowercase(_motionTypeName) == "translational")
		_motionType = Translational;
	else if(IO::Lowercase(_motionTypeName) == "coupled")
		_motionType = Coupled;
	else
		throw(Exception("Unknown motion type. Use rotational, translational, or coupled."));


	// Make sure the range is min to max.
	if (_range[1] < _range[0]){
		throw(Exception("Maximum coordinate range less than minimum."));
	}

	// Make sure the default value is in the range
	if (_defaultValue < _range[0])
		_defaultValue = _range[0];
	else if (_defaultValue > _range[1])
		_defaultValue = _range[1];

	// Define the locked value for the constraint as a function
	_lockFunction = new ModifiableConstant<Real>(_defaultValue, 1); 
}

void Coordinate::initState(State& s) const
{
	// Cannot enforce the constraint, since state of constraints may still be undefined
	const MobilizedBody& mb=_model->getMatterSubsystem().getMobilizedBody(_bodyIndex);
	int nq=mb.getNumQ(s);
	if (_mobilityIndex>=nq){
		//Something is wrong/inconsistent with model definition. Abort
		throw(Exception("Coordinate: "+getName()+" is not consistent with owner Joint. Aborting."));
	}
	_model->getMatterSubsystem().getMobilizedBody(_bodyIndex).setOneQ(s,_mobilityIndex,_defaultValue);
    _model->getMatterSubsystem().getMobilizedBody(_bodyIndex).setOneU(s,_mobilityIndex,_defaultSpeedValue);
	setIsPrescribed(s, _isPrescribed);
	setClamped(s, _clamped);
	// Locking takes precedence if all joint constraints are ON
	setLocked(s, _locked);
}

void Coordinate::setDefaultsFromState(const SimTK::State& state)
{
    _defaultValue = _model->getMatterSubsystem().getMobilizedBody(_bodyIndex).getOneQ(state,_mobilityIndex);
    _defaultSpeedValue = _model->getMatterSubsystem().getMobilizedBody(_bodyIndex).getOneU(state,_mobilityIndex);
    _isPrescribed = isPrescribed(state);
    _clamped= getClamped(state);
    _locked = getLocked(state);
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
    Object::operator=(aCoordinate);

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

void Coordinate::updateFromCoordinate(const Coordinate &aCoordinate)
{
	if (!aCoordinate.getValueUseDefault())
		setDefaultValue(aCoordinate.getDefaultValue());

	if (!aCoordinate.getRangeUseDefault()) {
		setRangeMin(aCoordinate.getRangeMin());
		setRangeMax(aCoordinate.getRangeMax());
	}

	if (!aCoordinate.getDefaultValueUseDefault())
		setDefaultValue(aCoordinate.getDefaultValue());

	if (!aCoordinate.getClampedUseDefault())
		setDefaultClamped(aCoordinate.getDefaultClamped());

	if (!aCoordinate.getLockedUseDefault())
		setDefaultLocked(aCoordinate.getDefaultLocked());

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
void Coordinate::setJoint(const Joint& aOwningJoint)
{
	_joint = &aOwningJoint;
}
//_____________________________________________________________________________
/**
 * Get the joint to which this coordinate belongs.
 *
 * @return Joint to which this coordinate belongs.
 */
const Joint& Coordinate::getJoint() const
{
	return(*_joint);
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
double Coordinate::getValue(const SimTK::State& s) const
{
	return _model->getMatterSubsystem().getMobilizedBody(_bodyIndex).getOneQ(s,_mobilityIndex);
}
//done_____________________________________________________________________________
/**
 * Set the value.
 *
 * @param aValue value to change to.
 * @return Whether or not the value was changed.
 */
bool Coordinate::setValue(SimTK::State& s, double aValue , bool enforceConstraints) const
{
	// If the coordinate is clamped, pull aValue into range.
	if (_clamped) {
		if (aValue < _range[0])
			aValue = _range[0];
		else if (aValue > _range[1])
			aValue = _range[1];
	}

	// If the coordinate is locked and aValue is not the current value, print an error.
	// Otherwise, set the value to aValue.
	if (_locked) {
		if (aValue != getValue(s) && !_lockedWarningGiven){
			cout<<"Coordinate.setValue: WARN- coordinate "<<getName()<<" is locked. Unable to change its value." << endl;
			_lockedWarningGiven=true;
		}
	} else {
		_model->updMatterSubsystem().getMobilizedBody(_bodyIndex).setOneQ(s,_mobilityIndex,aValue);
	}

	// The Q that was set might not satisfy constraints, so if enforceConstraints then project configuration.
	// You want to do this even if the coordinate is locked and its value hasn't changed, because this may be
	// the last setValue() call in a string of them (e.g., to set a model pose), in which case you only try to
	// enforce constraints during the last call.
	if (enforceConstraints) {
		_model->getSystem().realize(s, Stage::Position);

		if (_model->getConstraintSet().getSize()>0){
			//project states onto to constraint manifold to make sure states satisfy ALL constraints
			// Replaced _tolerance with 1e-5 per Sherm. -Ayman 4/09
			_model->getSimbodyEngine().projectConfigurationToSatisfyConstraints(s, 1e-5);
		}
	}

	if (!_locked) {
		_defaultValue = getValue(s);

		//Update the value that the coordinate is locked at.
		dynamic_cast<ModifiableConstant<Real>*>(_lockFunction)->setValue(_defaultValue);
	}

	return true;
}

double Coordinate::getSpeedValue(const SimTK::State& s) const
{
	return _model->getMatterSubsystem().getMobilizedBody(_bodyIndex).getOneU(s,_mobilityIndex);
}


bool Coordinate::setSpeedValue(SimTK::State& s, double aValue) const
{
	_model->updMatterSubsystem().getMobilizedBody(_bodyIndex).setOneU(s,_mobilityIndex,aValue);
	_defaultSpeedValue = aValue;

	return true;
}

const std::string  Coordinate::getSpeedName() const
{
	return _name + "_u";
}

double Coordinate::getAccelerationValue(const SimTK::State& s) const
{
	return _model->getMatterSubsystem().getMobilizedBody(_bodyIndex).getOneUDot(s, _mobilityIndex);
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
 * Set coordinate's motion type.
 *
 */
 void Coordinate::setMotionType(MotionType aMotionType)
 {
	 _motionType = aMotionType;

	 //Also update the motionTypeName so that it is serialized with the model
	 switch(aMotionType){
		case(Rotational) : 	
			_motionTypeName = "rotational";
			break;
		case(Translational) :
			_motionTypeName = "translational";
			break;
		case(Coupled) :
			_motionTypeName = "coupled";
			break;
		default :
			throw(Exception("Coordinate: Attempting to specify an undefined motion type."));
	 }
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
void Coordinate::setPrescribedFunction(const OpenSim::Function& function)
{
	_prescribedFunction = (Function*)function.copy();
}

//_____________________________________________________________________________
/**
 *  Determine if the the coordinate is constrained or not.
 *  Specifically, is it a dependent coordinate in any of the constraints?\
 *  If so return true, false otherwise.
 */
bool Coordinate::isConstrained() const
{
	for(int i=0; i<_model->getConstraintSet().getSize(); i++){
		Constraint& aConstraint = _model->getConstraintSet().get(i);
		if(aConstraint.getType() == "CoordinateCouplerConstraint"){
			CoordinateCouplerConstraint& coupler = dynamic_cast<CoordinateCouplerConstraint&>(aConstraint);
			if (coupler.getDependentCoordinateName() == this->_name)
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
 * If lock is applied after clamping or prescribed motion it takes precedence.
 *
 * @param aLocked If true the joint is locked; if false the joint is unlocked.
 */
bool Coordinate::setLocked(SimTK::State& s, bool aLocked) const
{
	// Do nothing if the same
	if(aLocked == getLocked(s)) return true;
	
	_lockedWarningGiven=false;	// reset flag in case needed later
	SimTK::Constraint *lock = NULL;

	// Get constraint
	if(int(_lockedConstraintIndex) != SimTK::InvalidIndex){
		lock = &_model->getSystem().updMatterSubsystem().updConstraint(_lockedConstraintIndex);
	}
	else{
		string msg = "Lock constraint for coordinate could not be found.";
		throw Exception(msg,__FILE__,__LINE__);
	}

	// Now enable if locked otherwise disable
	if(aLocked){	
		lock->enable(s);
		_locked = true;
		//Cannot be locked and have prescribed motion and/or clamping
		setIsPrescribed(s, false);
		setClamped(s, false);
	}
	else {
		lock->disable(s);
		_locked = false;
	}
	return true;
}

/**
 * Get whether or not this coordinate is locked.
 * Calls the underlying constraint at the dynamics level.
 *
 * @return true if the coordinate is locked and false if unlocked.
 */
bool Coordinate::getLocked(const SimTK::State& s) const
{
	if(int(_lockedConstraintIndex) != SimTK::InvalidIndex){
		bool disabled = _model->getSystem().updMatterSubsystem().getConstraint(_lockedConstraintIndex).isDisabled(s);
		return !disabled;
	}
	else{
		return _locked;
	}
}

//-----------------------------------------------------------------------------
// PRESCRIBED MOTION
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set whether or not this coordinate is being prescribed.
 * A prescribed constraint is used specify motion at the dynamics level.
 * If isPrescribed is set after clamping or locking it takes precedence-
 * and clamped and locked are false.
 *
 * @param isPrescribed If true the coordinate is prescribed; if false not prescribed.
 */
void Coordinate::setIsPrescribed(SimTK::State& s, bool isPrescribed) const
{
	// Do nothing if the same
	if(isPrescribed == _isPrescribed) return;
	
	// The underlying SimTK constraint
	SimTK::Constraint *prescribe = NULL;

	// Get constraint
	if(int(_prescribedConstraintIndex) != SimTK::InvalidIndex){
		//get constraint
		prescribe = &_model->getSystem().updMatterSubsystem().updConstraint(_prescribedConstraintIndex);
	}
	else{
		string msg = "Preecribed motion for coordinate not found.";
		throw Exception(msg,__FILE__,__LINE__);
	}


	// Now enable if prescribed motion constraint otherwise disable
	if(isPrescribed){	
		prescribe->enable(s);
		setLocked(s, false);
		setClamped(s, false);
	}
	else
		prescribe->disable(s);
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
 * @param aClamped If true the joint coordinate is clamped; if false the joint is unlocked.
 */

bool Coordinate::setClamped(State& s, bool aClamped ) const
{
	_clamped = aClamped;
	return false;
}

/**
 * Get whether or not this coordinate is clamped.
 * Calls the underlying constraint at the dynamics level.
 *
 * @return true if the coordinate is clamped.
 */
bool Coordinate::getClamped(const SimTK::State& s) const
{
	return _clamped;
}

//=============================================================================
// UTILITY
//=============================================================================
//_____________________________________________________________________________

