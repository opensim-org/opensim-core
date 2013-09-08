/* -------------------------------------------------------------------------- *
 *                   OpenSim:  MuscleActivationDynamics.cpp                   *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2013 Stanford University and the Authors                *
 * Author(s): Thomas Uchida, Ajay Seth                                        *
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

#include "MuscleActivationDynamics.h"

//==============================================================================
// STATICS
//==============================================================================
using namespace std;
using namespace OpenSim;
using namespace SimTK;

//==============================================================================
// CONSTRUCTORS
//==============================================================================
MuscleActivationDynamics::MuscleActivationDynamics()
{
    setNull();
    constructProperties();
    setName("default_MuscleActivationDynamics");
}

MuscleActivationDynamics::MuscleActivationDynamics(Muscle* muscle,
                                                   double minimumActivation,
                                                   double maximumActivation,
                                                   double defaultActivation,
                                                   const std::string& name)
{
    setNull();
    constructProperties();

    setMuscle(muscle);
    setMinimumActivation(minimumActivation);
    setMaximumActivation(maximumActivation);
    setDefaultActivation(defaultActivation);
    setName(name);
}

//==============================================================================
// ACCESSORS AND MUTATORS
//==============================================================================
const Muscle* MuscleActivationDynamics::getMuscle() const
{   return m_muscle; }

double MuscleActivationDynamics::getMinimumActivation() const
{   return get_minimum_activation(); }

double MuscleActivationDynamics::getMaximumActivation() const
{   return get_maximum_activation(); }

double MuscleActivationDynamics::getDefaultActivation() const
{   return get_default_activation(); }

void MuscleActivationDynamics::setMuscle(Muscle* muscle)
{   m_muscle = muscle; }

void MuscleActivationDynamics::setMinimumActivation(double minimumActivation)
{
    // Minimum must be in the interval [0,maximum].
    set_minimum_activation( clamp(0,minimumActivation,getMaximumActivation()) );
    // Ensure default remains in the interval [minimum,maximum].
    setDefaultActivation(getDefaultActivation());
}

void MuscleActivationDynamics::setMaximumActivation(double maximumActivation)
{
    // Maximum must be in the interval [minimum,1].
    set_maximum_activation( clamp(getMinimumActivation(),maximumActivation,1) );
    // Ensure default remains in the interval [minimum,maximum].
    setDefaultActivation(getDefaultActivation());
}

void MuscleActivationDynamics::setDefaultActivation(double defaultActivation)
{
    // Default must be in the interval [minimum,maximum].
    set_default_activation( clamp(getMinimumActivation(),
                                  defaultActivation,
                                  getMaximumActivation()) );
}

//==============================================================================
// MODELCOMPONENT INTERFACE REQUIREMENTS
//==============================================================================
void MuscleActivationDynamics::connectToModel(Model& model)
{
    Super::connectToModel(model);
    if(!m_muscle)
        throw OpenSim::Exception("MuscleActivationDynamics::connectToModel() "
                                 "requires a valid Muscle pointer to proceed.");
}

//==============================================================================
// STATE-DEPENDENT METHODS
//==============================================================================
double MuscleActivationDynamics::getActivation(const SimTK::State& s) const
{
    if(!m_muscle)
        throw OpenSim::Exception("MuscleActivationDynamics::getActivation() "
                                 "requires a valid Muscle pointer to proceed.");
    return m_muscle->getControl(s);
}

void MuscleActivationDynamics::setActivation(SimTK::State& s,
                                             double activation) const
{
    if(!m_muscle)
        throw OpenSim::Exception("MuscleActivationDynamics::setActivation() "
                                 "requires a valid Muscle pointer to proceed.");
    SimTK::Vector& controls(m_muscle->getModel().updControls(s));
    m_muscle->setControls(SimTK::Vector(1,activation), controls);
    m_muscle->getModel().setControls(s,controls);
}

//==============================================================================
// PRIVATE METHODS
//==============================================================================
void MuscleActivationDynamics::setNull()
{
    setAuthors("Thomas Uchida, Ajay Seth");
    setMuscle(NULL);
}

void MuscleActivationDynamics::constructProperties()
{
    constructProperty_minimum_activation(0);
    constructProperty_maximum_activation(1);
    constructProperty_default_activation(0.5);
}
