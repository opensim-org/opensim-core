/* -------------------------------------------------------------------------- *
 *                           OpenSim:  Actuator.cpp                           *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Ajay Seth                                                       *
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
#include <OpenSim/Simulation/Model/Model.h>
#include "Actuator.h"


using namespace std;
using namespace OpenSim;
using namespace SimTK;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Construct an actuator of controls.
 *
 */
Actuator_::Actuator_()
{
    setNull();
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
    setAuthors("Ajay Seth");
    _controlIndex = -1;
}

// Create the underlying computational system component(s) that support the
// Actuator model component
void Actuator_::addToSystem(SimTK::MultibodySystem& system) const
{
    Super::addToSystem(system);

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
Actuator::Actuator()
{
    constructInfrastructure();
}


/**
 * Set up the serializable member variables. This involves constructing and
 * initializing properties.
 */
void Actuator::constructProperties()
{
    constructProperty_min_control(-Infinity);
    constructProperty_max_control( Infinity);
}

void Actuator::constructOutputs() 
{
    constructOutput<double>("force", &Actuator::getForce, SimTK::Stage::Velocity);
    constructOutput<double>("speed", &Actuator::getSpeed, SimTK::Stage::Velocity);  
}

// Create the underlying computational system component(s) that support the
// Actuator model component
void Actuator::addToSystem(SimTK::MultibodySystem& system) const
{
    Super::addToSystem(system);

    // Add modeling flag to compute actuation with dynamic or by-pass with 
    // override force provided
    addModelingOption("override_force", 1);

    // Cache the computed force and speed of the scalar valued actuator
    addCacheVariable<double>("force", 0.0, Stage::Velocity);
    addCacheVariable<double>("speed", 0.0, Stage::Velocity);

    // Discrete state variable is the override force value if in override mode
    addDiscreteVariable("override_force", Stage::Time);
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
    if (isDisabled(s)) return 0.0;
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


//___________________________________________________________________________
/**
 * overrideForce sets flag to indicate actuator's force compuation is being
 * overridden
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
/** set the value used when an actuator's force compuation is overridden
 */
void Actuator::setOverrideForce(SimTK::State& s, double force ) const
{
     setDiscreteVariable(s, "override_force", force);;
}

double Actuator::getOverrideForce(const SimTK::State& s ) const
{
    return getDiscreteVariable(s, "override_force");
}
double Actuator::computeOverrideForce( const SimTK::State& s ) const 
{
    double appliedForce = getOverrideForce(s);
    setForce(s, appliedForce);
    return appliedForce;
}