/* -------------------------------------------------------------------------- *
 *                          OpenSim:  Coordinate.cpp                          *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Author(s): Ajay Seth, Michael A. Sherman, Ayman Habib                      *
 * Contributor(s): Frank C. Anderson, Jeffrey A. Reinbolt                     *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

//=============================================================================
// INCLUDES
//=============================================================================
#include "Coordinate.h"
#include "CoordinateCouplerConstraint.h"
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/SimbodyEngine/Joint.h>
#include "simbody/internal/Constraint.h"

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace SimTK;
using namespace OpenSim;

/** @cond **/ // hide from Doxygen
/**
 * A Constant Function whose value is modifiable.
 */
class ModifiableConstant : public SimTK::Function_<SimTK::Real>{
public:
    ModifiableConstant(const SimTK::Real& value, int argumentSize) : 
        argumentSize(argumentSize), value(value) { }

    ModifiableConstant* clone() const override {
        return new ModifiableConstant(this->value, this->argumentSize);
    }

    SimTK::Real calcValue(const SimTK::Vector& x) const override {
        assert(x.size() == argumentSize);
        return value;
    }

    SimTK::Real calcDerivative(const std::vector<int>& derivComponents, 
        const SimTK::Vector& x) const {
        return calcDerivative(SimTK::ArrayViewConst_<int>(derivComponents),x);
    }
    SimTK::Real calcDerivative(const SimTK::Array_<int>& derivComponents, 
        const SimTK::Vector& x) const override {
        return 0;
    }

    int getArgumentSize() const override {
        return argumentSize;
    }
    int getMaxDerivativeOrder() const override {
        return std::numeric_limits<int>::max();
    }

    void setValue(SimTK::Real newValue){
        value = newValue;
    }
private:
    const int argumentSize;
    SimTK::Real value;
};
/** @endcond **/

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */
Coordinate::Coordinate() 
{
    constructProperties();
}

//_____________________________________________________________________________
/**
 * Constructor.
 */
Coordinate::Coordinate(const std::string &aName, MotionType aMotionType,
        double defaultValue, double aRangeMin, double aRangeMax) :
    Coordinate()
{
    setName(aName);
    //setMotionType(aMotionType);
    setDefaultValue(defaultValue);
    setRangeMin(aRangeMin);
    setRangeMax(aRangeMax);
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void Coordinate::constructProperties(void)
{
    setAuthors("Ajay Seth, Ayman Habib, Michael Sherman");

    constructProperty_default_value(0.0);
    constructProperty_default_speed_value(0.0);

    Array<double> defaultRange(-10.0, 2); //two values in range
    defaultRange[1] = 10.0; // second value in range is 10.0
    constructProperty_range(defaultRange);

    constructProperty_clamped(false);
    constructProperty_locked(false);

    constructProperty_prescribed_function();
    constructProperty_prescribed(false);
    constructProperty_is_free_to_satisfy_constraints(false);
}


//_____________________________________________________________________________
/*
 * Perform some set up functions that happen after the
 * object has been deserialized or copied.
 *
 */
void Coordinate::extendFinalizeFromProperties()
{
    Super::extendFinalizeFromProperties();

    string prefix = "Coordinate("+getName()+")::extendFinalizeFromProperties:";

    // Make sure the default value is within the range when clamped
    if (get_clamped()){
        // Make sure the range is min to max.
        SimTK_ERRCHK_ALWAYS(get_range(0) <= get_range(1), prefix.c_str(),
            "Maximum coordinate range less than minimum.");

        double dv = get_default_value();
        SimTK_ERRCHK2_ALWAYS(dv > (get_range(0) - SimTK::SqrtEps), prefix.c_str(),
            "Default coordinate value is less than range minimum.\n" 
            "Default value = %g  < min = %g.", dv,  get_range(0));

        SimTK_ERRCHK2_ALWAYS(dv < (get_range(1) + SimTK::SqrtEps), prefix.c_str(),
            "Default coordinate value is greater than range maximum.\n"
            "Default value = %g > max = %g.", dv, get_range(1));    
    }

    _lockedWarningGiven=false;

    _speedName = getName() + "/speed";
}

void Coordinate::extendAddToSystem(SimTK::MultibodySystem& system) const
{
    Super::extendAddToSystem(system);

    // Make this modifiable temporarily so we can record information needed
    // to later access our pieces of the SimTK::MultibodySystem. That info is
    // const after the system has been built.
    Coordinate* mutableThis = const_cast<Coordinate *>(this);

    // Define the locked value for the constraint as a function.
    // The PrescribedMotion will take ownership, but we'll keep a reference
    // pointer here to allow for later modification.
    std::unique_ptr<ModifiableConstant> 
        funcOwner(new ModifiableConstant(get_default_value(), 1));
    mutableThis->_lockFunction = funcOwner.get();
    
    // The underlying SimTK constraint
    SimTK::Constraint::PrescribedMotion 
        lock(system.updMatterSubsystem(), 
             funcOwner.release(),   // give up ownership 
             _bodyIndex, 
             SimTK::MobilizerQIndex(_mobilizerQIndex));

    // Save the index so we can access the SimTK::Constraint later
    mutableThis->_lockedConstraintIndex = lock.getConstraintIndex();
            
    if(!getProperty_prescribed_function().empty()){
        //create prescribed motion constraint automatically
        SimTK::Constraint::PrescribedMotion prescribe( 
                _model->updMatterSubsystem(), 
                get_prescribed_function().createSimTKFunction(), 
                _bodyIndex, 
                SimTK::MobilizerQIndex(_mobilizerQIndex));
        mutableThis->_prescribedConstraintIndex = prescribe.getConstraintIndex();
    }
    else if(get_prescribed()){
        // if prescribed is set to true, and there is no prescribed 
        // function defined, then it cannot be prescribed (set to false).
        mutableThis->upd_prescribed() = false;
    }

    //Now add clamping
    addModelingOption("is_clamped", 1);

    SimTK::SubsystemIndex sbsix =
        getModel().getMatterSubsystem().getMySubsystemIndex();

    //Expose coordinate state variable
    CoordinateStateVariable* csv 
        = new CoordinateStateVariable("value", *this,
                                       sbsix, _mobilizerQIndex );
    addStateVariable(csv);

    //Expose coordinate's speed state variable  
    SpeedStateVariable* ssv = 
        new SpeedStateVariable("speed", *this, sbsix, _mobilizerQIndex);
    addStateVariable(ssv);
}

void Coordinate::extendRealizeInstance(const SimTK::State& state) const
{
    //const MobilizedBody& mb
    //    = getModel().getMatterSubsystem().getMobilizedBody(_bodyIndex);

    //int uix = state.getUStart() + mb.getFirstUIndex(state) + _mobilizerQIndex;

    /* Set the YIndex on the StateVariable */
}

void Coordinate::extendInitStateFromProperties(State& s) const
{
    // Cannot enforce the constraint, since state of constraints may still be undefined
    const MobilizedBody& mb=_model->getMatterSubsystem().getMobilizedBody(_bodyIndex);
    int nq=mb.getNumQ(s);
    if (_mobilizerQIndex>=nq){
        //Something is wrong/inconsistent with model definition. Abort
        throw(Exception("Coordinate: "+getName()+" is not consistent with owner Joint. Aborting."));
    }
    _model->getMatterSubsystem().getMobilizedBody(_bodyIndex)
        .setOneQ(s,_mobilizerQIndex,get_default_value());
    _model->getMatterSubsystem().getMobilizedBody(_bodyIndex)
        .setOneU(s,_mobilizerQIndex,get_default_speed_value());
    setIsPrescribed(s, get_prescribed());
    setClamped(s, get_clamped());
    // Locking takes precedence if all joint constraints are ON
    setLocked(s, get_locked());
}

void Coordinate::extendSetPropertiesFromState(const SimTK::State& state)
{
    upd_default_value() = _model->getMatterSubsystem().getMobilizedBody(_bodyIndex).getOneQ(state,_mobilizerQIndex);
    upd_default_speed_value() = _model->getMatterSubsystem().getMobilizedBody(_bodyIndex).getOneU(state,_mobilizerQIndex);
    upd_prescribed() = isPrescribed(state);
    upd_clamped() = getClamped(state);
    upd_locked() = getLocked(state);
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
    return(_joint.getRef());
}

Coordinate::MotionType Coordinate::getMotionType() const
{
    int ix = getJoint().getProperty_coordinates().findIndexForName(getName());
    return getJoint().getMotionType(Joint::CoordinateIndex(ix));
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
    return _model->getMatterSubsystem().getMobilizedBody(_bodyIndex).getOneQ(s,_mobilizerQIndex);
}
//done_____________________________________________________________________________
/**
 * Set the value.
 *
 * @param aValue value to change to.
 */
void Coordinate::setValue(SimTK::State& s, double aValue , bool enforceConstraints) const
{
    // If enforceConstraints is true and coordinate is clamped, clamp aValue into range
    if (enforceConstraints && getClamped(s)) {
        if (aValue < get_range(0))
            aValue = get_range(0);
        else if (aValue > get_range(1))
            aValue = get_range(1);
    }

    // If the coordinate is locked and aValue is not the current value, print an error.
    // Otherwise, set the value to aValue.
    if (getLocked(s)) {
        if (aValue != getValue(s) && !_lockedWarningGiven){
            log_warn("Coordinate.setValue:  coordinate {} is locked. Unable to change its value.",
                getName());
            _lockedWarningGiven=true;
        }
    } else {
        _model->updMatterSubsystem().getMobilizedBody(_bodyIndex).setOneQ(s,_mobilizerQIndex,aValue);
    }

    // The Q that was set might not satisfy constraints, so if enforceConstraints then call model assemble.
    // You want to do this even if the coordinate is locked and its value hasn't changed, because this may be
    // the last setValue() call in a string of them (e.g., to set a model pose), in which case you only try to
    // enforce constraints during the last call.
    if (enforceConstraints) {
        if (_model->getConstraintSet().getSize()>0 || isConstrained(s)){
            // if this coordinate is set up to be dependent on other coordinates
            // its value should be dictated by the other coordinates and not its present value
            double weight = isDependent(s) ? 0.0  : 10;
            // assemble model so that states satisfy ALL constraints
            _model->assemble(s, this, weight);
        }
        else
            _model->getMultibodySystem().realize(s, Stage::Position );
    }
}

double Coordinate::getSpeedValue(const SimTK::State& s) const
{
    return _model->getMatterSubsystem().getMobilizedBody(_bodyIndex).getOneU(s,_mobilizerQIndex);
}


void Coordinate::setSpeedValue(SimTK::State& s, double aValue) const
{
    _model->updMatterSubsystem().getMobilizedBody(_bodyIndex).setOneU(s,_mobilizerQIndex,aValue);
}

const std::string&  Coordinate::getSpeedName() const
{
    return _speedName;
}

double Coordinate::getAccelerationValue(const SimTK::State& s) const
{
    return getModel().getMatterSubsystem().getMobilizedBody(_bodyIndex).getOneUDot(s, _mobilizerQIndex);
}


//_____________________________________________________________________________
/**
 * Set the range min and max.
 *
 * @param aRange range min and man to change to.
 * @return Whether or not the range was changed.
 */
void Coordinate::setRange(double aRange[2])
{
    if (aRange[1] >= aRange[0]) {
        upd_range(0) = aRange[0];
        upd_range(1) = aRange[1];
    }
    else
        throw Exception("Coordinate::setRange, range is invalid, "
            "min range value exceeds max."); 
}

//_____________________________________________________________________________
/**
 * Set the range min.
 *
 * @param aRange range min to change to.
 * @return Whether or not the range min was changed.
 */
void Coordinate::setRangeMin(double aMin)
{
    upd_range(0) = aMin;
}

//_____________________________________________________________________________
/**
 * Set the range max.
 *
 * @param aRange range max to change to.
 * @return Whether or not the range max was changed.
 */
void Coordinate::setRangeMax(double aMax)
{
    upd_range(1) = aMax;
}

//_____________________________________________________________________________
/**
 * Set the default value.
 *
 * @param aDefaultValue new default value to change to.
 */
void Coordinate::setDefaultValue(double aDefaultValue)
{
    upd_default_value() = aDefaultValue;
}

//_____________________________________________________________________________
/**
 * Get the prescribed motion function.
 *
 * @return const reference to the prescribed motion function.
 */
const OpenSim::Function& Coordinate::getPrescribedFunction() const
{
    return get_prescribed_function();
}

//_____________________________________________________________________________
/**
 * Set the prescribed motion function.
 */
void Coordinate::setPrescribedFunction(const OpenSim::Function& function)
{
    //Optional property so clear out previous function value if any
    updProperty_prescribed_function().clear();
    updProperty_prescribed_function().adoptAndAppendValue(function.clone());
}

//_____________________________________________________________________________
/**
 * Determine if the coordinate is dependent on other coordinates or not,
 * by checking to see whether there is a CoordinateCouplerConstraint relating
 * it to another coordinate. If so return true, false otherwise.
 * TODO: note that this will fail to detect any other kind of constraint that
 * might couple coordinates together.
 */
 bool Coordinate::isDependent(const SimTK::State& s) const
{
    if(get_is_free_to_satisfy_constraints())
        return true;

    for(int i=0; i<_model->getConstraintSet().getSize(); i++){
        Constraint& constraint = _model->getConstraintSet().get(i);
        CoordinateCouplerConstraint* couplerp = 
            dynamic_cast<CoordinateCouplerConstraint*>(&constraint);
        if(couplerp) {
            if (couplerp->getDependentCoordinateName() == getName())
                return couplerp->isEnforced(s);
        }
    }
    return false;
}
//_____________________________________________________________________________
/**
 *  Determine if the coordinate is constrained or not.
 *  Specifically, is locked, prescribed, or completely dependent on other coordinates?
 *  If so return true, false otherwise.
 */
bool Coordinate::isConstrained(const SimTK::State& s) const
{
    return (getLocked(s) || isPrescribed(s) || isDependent(s));
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
void Coordinate::setLocked(SimTK::State& s, bool aLocked) const
{
    // Do nothing if the same, but make sure _locked is also up-to-date
    if(aLocked == getLocked(s)){
        return;
    }
    
    _lockedWarningGiven=false;  // reset flag in case needed later
    SimTK::Constraint *lock = NULL;

    // Get constraint
    if(_lockedConstraintIndex.isValid()){
        lock = &_model->updMultibodySystem().updMatterSubsystem().updConstraint(_lockedConstraintIndex);
    }
    else{
        string msg = "Lock constraint for coordinate could not be found.";
        throw Exception(msg,__FILE__,__LINE__);
    }

    // Now enable if locked otherwise disable
    if(aLocked){
        // Update the locked value of the constraint before locking
        _lockFunction->setValue(getValue(s));
        lock->enable(s);
        //Cannot be locked and have prescribed motion and/or clamping
        setIsPrescribed(s, false);
    }
    else {
        lock->disable(s);
    }
}

/**
 * Get whether or not this coordinate is locked.
 * Calls the underlying constraint at the dynamics level.
 *
 * @return true if the coordinate is locked and false if unlocked.
 */
bool Coordinate::getLocked(const SimTK::State& s) const
{
    if(_lockedConstraintIndex.isValid()){
        bool disabled = _model->updMultibodySystem().updMatterSubsystem().getConstraint(_lockedConstraintIndex).isDisabled(s);
        return !disabled;
    }
    else{
        return get_locked();
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
    if(isPrescribed == this->isPrescribed(s) ) return;
    
    // The underlying SimTK constraint
    SimTK::Constraint *prescribe = NULL;

    // Get constraint
    if(_prescribedConstraintIndex.isValid()){
        //get constraint
        prescribe = &_model->updMultibodySystem().updMatterSubsystem().updConstraint(_prescribedConstraintIndex);
    }
    else{
        string msg = "Prescribed motion for coordinate not found.";
        throw Exception(msg,__FILE__,__LINE__);
    }

    // Now enable if prescribed motion constraint otherwise disable
    if(isPrescribed){   
        prescribe->enable(s);
        setLocked(s, false);
    }
    else
        prescribe->disable(s);
}

bool Coordinate::isPrescribed(const SimTK::State& s) const
{
    if(_prescribedConstraintIndex.isValid()){
        bool disabled = _model->updMultibodySystem().updMatterSubsystem().getConstraint(_prescribedConstraintIndex).isDisabled(s);
        return !disabled;
    }
    else{
        return get_prescribed();
    }
    
}

//-----------------------------------------------------------------------------
// CLAMP
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
bool Coordinate::getClamped(const SimTK::State& s) const
{
    return getModelingOption(s, "is_clamped") > 0;
}

void Coordinate::setClamped(SimTK::State& s, bool aLocked) const
{
    setModelingOption(s, "is_clamped", (int)aLocked);
}

//-----------------------------------------------------------------------------
// Coordinate::CoordinateStateVariable
//-----------------------------------------------------------------------------
double Coordinate::CoordinateStateVariable::
    getValue(const SimTK::State& state) const
{
    return ((Coordinate *)&getOwner())->getValue(state);
}

void Coordinate::CoordinateStateVariable::
    setValue(SimTK::State& state, double value) const
{
    ((Coordinate *)&getOwner())->setValue(state, value, false);
}

double Coordinate::CoordinateStateVariable::
    getDerivative(const SimTK::State& state) const
{
    //TODO: update to get qdot value from the mobilized body
    return ((Coordinate *)&getOwner())->getSpeedValue(state); 
}


void Coordinate::CoordinateStateVariable::
    setDerivative(const SimTK::State& state, double deriv) const
{
    string msg = "CoordinateStateVariable::setDerivative() - ERROR \n";
    msg +=  "Coordinate derivative (qdot) is computed by the Multibody system.";
    throw Exception(msg);
}


//-----------------------------------------------------------------------------
// Coordinate::SpeedStateVariable
//-----------------------------------------------------------------------------
double Coordinate::SpeedStateVariable::
    getValue(const SimTK::State& state) const
{
    //TODO: update to get qdot value from the mobilized body
    return ((Coordinate *)&getOwner())->getSpeedValue(state);
}

void Coordinate::SpeedStateVariable::
    setValue(SimTK::State& state, double deriv) const
{
    //TODO: update to set qdot value from the mobilized body
    ((Coordinate *)&getOwner())->setSpeedValue(state, deriv);
}

double Coordinate::SpeedStateVariable::
    getDerivative(const SimTK::State& state) const
{
    const Coordinate& owner = *((Coordinate *)&getOwner());
    const MobilizedBody& mb = owner.getModel().getMatterSubsystem()
                                .getMobilizedBody(owner.getBodyIndex());

    return mb.getUDotAsVector(state)[owner.getMobilizerQIndex()];
}

void Coordinate::SpeedStateVariable::
    setDerivative(const SimTK::State& state, double deriv) const
{
    string msg = "SpeedStateVariable::setDerivative() - ERROR \n";
    msg +=  "Generalized speed derivative (udot) can only be set by the Multibody system.";
    throw Exception(msg);
}

//=============================================================================
// XML Deserialization
//=============================================================================
void Coordinate::updateFromXMLNode(SimTK::Xml::Element& aNode,
    int versionNumber)
{
    if (versionNumber < XMLDocument::getLatestVersion()) {
        if (versionNumber < 30514) {
            SimTK::Xml::element_iterator iter = aNode.element_begin("motion_type");
            if (iter != aNode.element_end()) {
                SimTK::Xml::Element motionTypeElm = 
                    SimTK::Xml::Element::getAs(aNode.removeNode(iter));
                std::string typeName = motionTypeElm.getValue();

                if ((IO::Lowercase(typeName) == "rotational"))
                    _userSpecifiedMotionTypePriorTo40 = Rotational;
                else if (IO::Lowercase(typeName) == "translational")
                    _userSpecifiedMotionTypePriorTo40 = Translational;
                else if (IO::Lowercase(typeName) == "coupled")
                    _userSpecifiedMotionTypePriorTo40 = Coupled;
                else
                    _userSpecifiedMotionTypePriorTo40 = Undefined;
            }
        }
    }
    Super::updateFromXMLNode(aNode, versionNumber);
}

const Coordinate::MotionType& Coordinate::getUserSpecifiedMotionTypePriorTo40() const
{
    return _userSpecifiedMotionTypePriorTo40;
}
