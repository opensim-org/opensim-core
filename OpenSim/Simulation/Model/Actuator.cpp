// Actuator.cpp
// Author: Ajay Seth
/*
 * Copyright (c)  2010, Stanford University. All rights reserved. 
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
#include "Actuator.h"
#include <OpenSim/Common/Object.h>
#include <OpenSim/Common/DebugUtilities.h>
#include <OpenSim/Common/StateFunction.h>
#include <sstream>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Control/Controller.h>
#include "SimTKsimbody.h"
#include "ForceAdapter.h"
#include <OpenSim/Simulation/SimbodyEngine/SimbodyEngine.h>

using namespace std;
using namespace OpenSim;
using namespace SimTK;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Construct an actuator  of  controls.
 *
 */
Actuator_::Actuator_() : Force(),
	_controlSuffixes(""),
    _subsystemIndex(SimTK::InvalidIndex)
{
	setNull();
}
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aActuator Actuator to copy.
 */
Actuator_::Actuator_(const Actuator_ &aAct) :
	Force(aAct),
	_controlSuffixes(""),
    _subsystemIndex(SimTK::InvalidIndex)
{
	setNull();

	// ASSIGN
	*this = aAct;
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
Actuator_::~Actuator_()
{
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the data members of this Actuator to their null values.
 */
void Actuator_::setNull()
{
	setType("Actuator_");
}

//_____________________________________________________________________________
/**
 * Perform set up functions after model has been deserialized or copied.
 *
 * @param aModel model containing this actuator.
 */
void Actuator_::setup(Model& aModel)
{
	Force::setup(aModel);
}

// Create the underlying computational system component(s) that support the
// Actuator model component
void Actuator_::createSystem(SimTK::MultibodySystem& system) const
{
	Force::createSystem(system);
    // Beyond the const Component get the index so we can access the SimTK::Force later
	Actuator_* mutableThis = const_cast<Actuator_ *>(this);

	mutableThis->_subsystemIndex = getIndexOfSubsystemForAllocations();

	// Model is in charge of creating the shared cache for all all actuator controls
	// but it does so based on the size and order in its _defaultControls
	// Actuator has the opportunity here to add slots for its control and record
	// the index into the shared cache Vector.
	mutableThis->_controlIndex = _model->updDefaultControls().size();
	_model->updDefaultControls().resizeKeep(_controlIndex + numControls());
	_model->updDefaultControls()(_controlIndex, numControls()) = Vector(numControls(), 0.0);
}

void Actuator_::initState(SimTK::State& state) const
{
	Force::initState(state);
}

void Actuator_::setDefaultsFromState(const SimTK::State& state)
{
	Force::setDefaultsFromState(state);
}


//_____________________________________________________________________________
/**
 * Update the geometric representation of the Actuator if any.
 * The resulting geometry is maintained at the VisibleObject layer
 * 
 */
void Actuator_::updateGeometry()
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
Actuator_& Actuator_::operator=(const Actuator_ &aAct)
{
	// BASE CLASS
	Force::operator=(aAct);

	// Define the label associated with the control(s) by appending these suffixes
	// to the actuator name.
	_controlSuffixes = aAct._controlSuffixes;

	return(*this);
}
// CONTROLS
//_____________________________________________________________________________
/**
 * Get the actuator's control values
 *
 * @param the current state
 * @return The value of the controls.
 */
const VectorView_<Real> Actuator_::getControls( const SimTK::State& s ) const
{
	const Vector &controlsCache = _model->getControls(s);
	return  controlsCache(_controlIndex, numControls());
}

void Actuator_::getControls(const Vector& modelControls, Vector& actuatorControls) const
{
	SimTK_ASSERT(modelControls.size() == _model->getNumControls(), 
		"Actuator_::getControls, input modelControls size does not match model.getNumControls().\n");
	SimTK_ASSERT(actuatorControls.size() == numControls(), 
		"Actuator_::getControls, output actuatorControls incompatible with actuator's numControls().\n");
	actuatorControls = modelControls(_controlIndex, numControls());
}

void Actuator_::setControls(const Vector& actuatorControls, Vector& modelControls) const
{
	SimTK_ASSERT(actuatorControls.size() == numControls(), 
		"Actuator_::setControls, input actuatorControls incompatible with actuator's numControls().\n");

	SimTK_ASSERT(modelControls.size() == _model->getNumControls(), 
	"Actuator_::setControls, output modelControls size does not match model.getNumControls()\n");

	modelControls(_controlIndex, numControls()) = actuatorControls;
}

void Actuator_::addInControls(const Vector& actuatorControls, Vector& modelControls) const
{
	SimTK_ASSERT(actuatorControls.size() == numControls(), 
		"Actuator_::addInControls, input actuatorControls incompatible with actuator's numControls()\n");

	SimTK_ASSERT(modelControls.size() == _model->getNumControls(), 
	"Actuator_::addInControls, output modelControls size does not match model.getNumControls().\n");

	modelControls(_controlIndex, numControls()) += actuatorControls;
}



//_____________________________________________________________________________
/**
 * Replace one of the actuator's functions in the property array.
 *
 * @param aOldFunction the function being replaced.
 * @param aNewFunction the new function.
 */
void Actuator_::replacePropertyFunction(OpenSim::Function* aOldFunction, OpenSim::Function* aNewFunction)
{
	if (aOldFunction && aNewFunction) {
		PropertySet& propSet = getPropertySet();

		for (int i=0; i <propSet.getSize(); i++) {
			Property* prop = propSet.get(i);
			if (prop->getType() == Property::ObjPtr) {
				if (prop->getValueObjPtr() == aOldFunction) {
					prop->setValue(aNewFunction);
				}
			}
		}
	}
}



//=============================================================================
// Scalar Actuator Implementation
//=============================================================================
//_____________________________________________________________________________

/** Default constructor */
Actuator::Actuator() : Actuator_(),
	_minControl(_propMinControl.getValueDbl()),
	_maxControl(_propMaxControl.getValueDbl()),
    _overrideForceFunction(0)
{
	setNull();
}

/**
 * Copy constructor.
 *
 * @param aActuator Actuator to copy.
 */
Actuator::Actuator(const Actuator &aAct) : Actuator_(aAct),
	_minControl(_propMinControl.getValueDbl()),
	_maxControl(_propMaxControl.getValueDbl()),
    _overrideForceFunction(0)
{
	setNull();
	_minControl = aAct._minControl;
	_maxControl = aAct._maxControl;
}

/**
 * Destructor.
 */
Actuator::~Actuator()
{
}

/**
 * Set up the serializable member variables.  This involves generating
 * properties and connecting local variables to those properties.
 */
void Actuator::setupProperties()
{
	_propMinControl.setComment("Minimum allowed value for control signal. Used primarily when solving for control values");
	_propMinControl.setName("min_control");
	_propMinControl.setValue(-SimTK::Infinity);
	_propertySet.append( &_propMinControl );
	
	_propMaxControl.setComment("Maximum allowed value for control signal. Used primarily when solving for control values");
	_propMaxControl.setName("max_control");
	_propMaxControl.setValue(SimTK::Infinity);
	_propertySet.append( &_propMaxControl );	
}

void Actuator::updateFromXMLNode()
{
	Force::updateFromXMLNode();
}

/**
 * Set the data members of this Actuator to their null values.
 */
void Actuator::setNull()
{
	setType("Actuator");
	setupProperties();
}

/**
 * Assignment operator.
 *
 * @return Reference to this object.
 */
Actuator& Actuator::operator=(const Actuator &aAct)
{
	// BASE CLASS
	Actuator_::operator=(aAct);

	_minControl=aAct._minControl;
	_maxControl=aAct._maxControl;

	return(*this);
}

// Setup the underlying computational system component(s) that implement the
// Actuator 
void Actuator::setup(Model& aModel)
{
	Actuator_::setup(aModel);
}

// Create the underlying computational system component(s) that support the
// Actuator model component
void Actuator::createSystem(SimTK::MultibodySystem& system) const
{
	Actuator_::createSystem(system);
    // Beyond the const Component get the index so we can access the SimTK::Force later
	Actuator* mutableThis = const_cast<Actuator *>(this);

	// Add modeling flag to compute actuation with dynamic or by-pass with override force provided
	Array<std::string> modelingFlags;
	modelingFlags.append("ComputeActuation");
	modelingFlags.append("OverrideForce");
	mutableThis->addModelingOption(modelingFlags);

	// Cache the computed force and speed of the scalar valued actuator
	mutableThis->addCacheVariable<double>("force", 0.0, Stage::Velocity);
	mutableThis->addCacheVariable<double>("speed", 0.0, Stage::Velocity);

	// Discrete state variable is the override force value if in override mode
	mutableThis->addDiscreteVariables(Array<string>("override_force",1), Stage::Time);
}

// do any state initialization
void Actuator::initState(SimTK::State& state) const
{
	Actuator_::initState(state);
	// Before initializing any state variable, get the current indices
	Actuator* mutableThis = const_cast<Actuator *>(this);

	// keep track of the cache indices
	mutableThis->_forceIndex = getCacheVariableIndex("force");
	mutableThis->_forceIndex = getCacheVariableIndex("speed");

	// if in override mode, this index will be valid
	mutableThis->_overrideForceIndex = getDiscreteVariableIndex("override_force");
}

void Actuator::setDefaultsFromState(const SimTK::State& state)
{
	Actuator_::setDefaultsFromState(state);
}

double Actuator::getControl(const SimTK::State& s ) const
{
	return getControls(s)[0];
}

//_____________________________________________________________________________
/**
 * getStress needs to be overridden by derived classes to be usable
 */
double Actuator::getStress(const SimTK::State& s ) const
{
	OPENSIM_ERROR_IF_NOT_OVERRIDDEN();
}
//_____________________________________________________________________________
/**
 * getOptimalForce needs to be overridden by derived classes to be usable
 */
double Actuator::getOptimalForce() const
{
	OPENSIM_ERROR_IF_NOT_OVERRIDDEN();
}

double Actuator::getForce(const State &s) const
{
    return getCacheVariable<double>(s, "force");
}

void Actuator::setForce(const State& s, double aForce) const
{
    updCacheVariable<double>(s, "force") = aForce;
}

double Actuator::getSpeed(const State& s) const
{
    return getCacheVariable<double>(s, "speed");
}

void Actuator::setSpeed(const State &s, double speed) const
{
    updCacheVariable<double>(s, "speed") = speed;
}


//_____________________________________________________________________________
/**
 * overrideForce sets flag indicating if an actuator's force compuation is overriden
 */
void Actuator::overrideForce(SimTK::State& s, bool flag ) const 
{
    setModelingOption(s, int(flag));
}

bool Actuator::isForceOverriden(const SimTK::State& s ) const 
{
    return (getModelingOption(s) > 0);
}
       
//_____________________________________________________________________________
/**
 * setOverrideForce sets the value used when an actuator's force compuation is orriden
 */
void Actuator::setOverrideForce(SimTK::State& s, double force ) const
{
    SimTK::Value<double>::downcast(s.updDiscreteVariable( _subsystemIndex, _overrideForceIndex)).upd() = force;
}
double Actuator::getOverrideForce(const SimTK::State& s ) const
{
    return getDiscreteVariable(s, "override_force");
}
double Actuator::computeOverrideForce( const SimTK::State& s ) const {
      if( _overrideForceFunction ) {
          return( _overrideForceFunction->calcValue(s ) );
      } else {
          return( getOverrideForce(s) );
      }
}
void Actuator::setOverrideForceFunction( StateFunction* overrideFunc ) {
       _overrideForceFunction = overrideFunc;
}

const StateFunction* Actuator::getOverrideForceFunction() const {
       return( _overrideForceFunction); 
}

StateFunction* Actuator::updOverrideForceFunction() {
       return( _overrideForceFunction); 
}

void Actuator::resetOverrideForceFunction() {
     _overrideForceFunction = 0;
}
