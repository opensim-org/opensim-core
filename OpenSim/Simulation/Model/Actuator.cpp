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
Actuator_::Actuator_() : Force()
{
	setNull();
}
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aActuator Actuator to copy.
 */
Actuator_::Actuator_(const Actuator_ &aAct) : Force(aAct)
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

// Create the underlying computational system component(s) that support the
// Actuator model component
void Actuator_::createSystem(SimTK::MultibodySystem& system) const
{
	Force::createSystem(system);
    // Beyond the const Component get the index so we can access the SimTK::Force later
	Actuator_* mutableThis = const_cast<Actuator_ *>(this);

	// Model is in charge of creating the shared cache for all all actuator controls
	// but it does so based on the size and order in its _defaultControls
	// Actuator has the opportunity here to add slots for its control and record
	// the index into the shared cache Vector.
	mutableThis->_controlIndex = _model->updDefaultControls().size();
	_model->updDefaultControls().resizeKeep(_controlIndex + numControls());
	_model->updDefaultControls()(_controlIndex, numControls()) = Vector(numControls(), 0.0);
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




//=============================================================================
// Scalar Actuator Implementation
//=============================================================================
//_____________________________________________________________________________

/** Default constructor */
Actuator::Actuator() : Actuator_(),
    _overrideForceFunction(0)
{
	setNull();
	setupProperties();
}

/**
 * Copy constructor.
 *
 * @param aActuator Actuator to copy.
 */
Actuator::Actuator(const Actuator &aAct) : Actuator_(aAct),
    _overrideForceFunction(0)
{
	setNull();
	setupProperties();
	setPropertyValue("min_control", aAct.getPropertyValue<double>("min_control"));
	setPropertyValue("max_control", aAct.getPropertyValue<double>("max_control"));
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
	addProperty<double>("min_control",
		"double",
		"Minimum allowed value for control signal. Used primarily when solving for control values",
		-std::numeric_limits<SimTK::Real>::infinity());
	addProperty<double>("max_control",
		"double",
		"Maximum allowed value for control signal. Used primarily when solving for control values",
		std::numeric_limits<SimTK::Real>::infinity());
}

/**
 * Set the data members of this Actuator to their null values.
 */
void Actuator::setNull()
{
	setType("Actuator");
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

	setPropertyValue("min_control", aAct.getPropertyValue<double>("min_control"));
	setPropertyValue("max_control", aAct.getPropertyValue<double>("max_control"));

	return(*this);
}

// Create the underlying computational system component(s) that support the
// Actuator model component
void Actuator::createSystem(SimTK::MultibodySystem& system) const
{
	Actuator_::createSystem(system);
    // Beyond the const Component get the index so we can access the SimTK::Force later
	Actuator* mutableThis = const_cast<Actuator *>(this);

	// Add modeling flag to compute actuation with dynamic or by-pass with override force provided
	mutableThis->addModelingOption("override_force", 1);

	// Cache the computed force and speed of the scalar valued actuator
	mutableThis->addCacheVariable<double>("force", 0.0, Stage::Velocity);
	mutableThis->addCacheVariable<double>("speed", 0.0, Stage::Velocity);

	// Discrete state variable is the override force value if in override mode
	mutableThis->addDiscreteVariables(Array<string>("override_force",1), Stage::Time);
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
    setCacheVariable<double>(s, "force", aForce);
}

double Actuator::getSpeed(const State& s) const
{
    return getCacheVariable<double>(s, "speed");
}

void Actuator::setSpeed(const State &s, double speed) const
{
    setCacheVariable<double>(s, "speed", speed);
}


//_____________________________________________________________________________
/**
 * overrideForce sets flag indicating if an actuator's force compuation is overriden
 */
void Actuator::overrideForce(SimTK::State& s, bool flag ) const 
{
    setModelingOption(s, "override_force", int(flag));
}

bool Actuator::isForceOverriden(const SimTK::State& s ) const 
{
    return (getModelingOption(s, "override_force") > 0);
}
       
//_____________________________________________________________________________
/**
 * setOverrideForce sets the value used when an actuator's force compuation is orriden
 */
void Actuator::setOverrideForce(SimTK::State& s, double force ) const
{
     setDiscreteVariable(s, "override_force", force);;
}

double Actuator::getOverrideForce(const SimTK::State& s ) const
{
    return getDiscreteVariable(s, "override_force");
}
double Actuator::computeOverrideForce( const SimTK::State& s ) const {
	double appliedForce = 0;
      if( _overrideForceFunction ) {
          appliedForce = _overrideForceFunction->calcValue(s);
      } else {
          appliedForce = getOverrideForce(s);
      }
	  setForce(s, appliedForce);
	  return appliedForce;
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
