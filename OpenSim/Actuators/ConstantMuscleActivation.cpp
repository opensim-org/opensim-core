/* -------------------------------------------------------------------------- *
 *                   OpenSim:  ConstantMuscleActivation.cpp                   *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
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

#include "ConstantMuscleActivation.h"

using namespace std;
using namespace OpenSim;
using namespace SimTK;

//==============================================================================
// CONSTRUCTORS
//==============================================================================
ConstantMuscleActivation::ConstantMuscleActivation() :
MuscleActivationDynamics()
{
    setNull();
    setName("default_ConstantMuscleActivation");
}

ConstantMuscleActivation::ConstantMuscleActivation(const std::string& name,
                                                   ExcitationGetter* getter) :
MuscleActivationDynamics(name, getter)
{
    setNull();
}

//==============================================================================
// MODELCOMPONENT INTERFACE REQUIREMENTS
//==============================================================================
void ConstantMuscleActivation::extendAddToSystem(SimTK::MultibodySystem& system) const
{
    Super::extendAddToSystem(system);
    this->_activationCV = addCacheVariable("activation", getDefaultActivation(), SimTK::Stage::Topology);
}

void ConstantMuscleActivation::extendInitStateFromProperties(SimTK::State& s) const
{
    Super::extendInitStateFromProperties(s);
    setActivation(s, getDefaultActivation());
}

void ConstantMuscleActivation::extendSetPropertiesFromState(const SimTK::State& s)
{
    Super::extendSetPropertiesFromState(s);
    setDefaultActivation(getActivation(s));
}

//==============================================================================
// STATE-DEPENDENT METHODS
//==============================================================================
double ConstantMuscleActivation::getActivation(const SimTK::State& s) const
{
    return clampToValidInterval(getCacheVariableValue(s, _activationCV));
}

void ConstantMuscleActivation::
setActivation(SimTK::State& s, double activation) const
{
    setCacheVariableValue(s, _activationCV, clampToValidInterval(activation));
}

//==============================================================================
// PRIVATE METHODS
//==============================================================================
void ConstantMuscleActivation::setNull()
{
    setAuthors("Thomas Uchida, Ajay Seth");
}
