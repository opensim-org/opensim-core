/* -------------------------------------------------------------------------- *
 *                   OpenSim:  MuscleActivationDynamics.cpp                   *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Thomas Uchida, Ajay Seth, Michael Sherman                       *
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

using namespace std;
using namespace OpenSim;
using namespace SimTK;

//==============================================================================
// CONSTRUCTORS AND DESTRUCTOR
//==============================================================================
MuscleActivationDynamics::MuscleActivationDynamics()
{
    setNull();
    constructProperties();
    setName("default_MuscleActivationDynamics");
}

MuscleActivationDynamics::MuscleActivationDynamics(const std::string& name,
                                                   ExcitationGetter* getter)
{
    setNull();
    constructProperties();
    setName(name);
    setExcitationGetter(getter);
}

MuscleActivationDynamics::~MuscleActivationDynamics()
{
    delete _excitationGetter;
}

//==============================================================================
// ACCESSORS AND MUTATORS
//==============================================================================
double MuscleActivationDynamics::getMinimumActivation() const
{   return get_minimum_activation(); }
double MuscleActivationDynamics::getMaximumActivation() const
{   return get_maximum_activation(); }
double MuscleActivationDynamics::getDefaultActivation() const
{   return get_default_activation(); }

void MuscleActivationDynamics::setMinimumActivation(double minimumActivation)
{
    set_minimum_activation( clamp(0,minimumActivation,getMaximumActivation()) );
    set_default_activation( clampToValidInterval(getDefaultActivation()) );
}

void MuscleActivationDynamics::setMaximumActivation(double maximumActivation)
{
    set_maximum_activation( clamp(getMinimumActivation(),maximumActivation,1) );
    set_default_activation( clampToValidInterval(getDefaultActivation()) );
}

void MuscleActivationDynamics::setDefaultActivation(double defaultActivation)
{
    set_default_activation(clampToValidInterval(defaultActivation));
}

void MuscleActivationDynamics::setExcitationGetter(ExcitationGetter* getter)
{
    delete _excitationGetter;
    _excitationGetter = getter;
}

//==============================================================================
// STATE-DEPENDENT METHODS
//==============================================================================
double MuscleActivationDynamics::getExcitation(const SimTK::State& s) const
{
    if(!_excitationGetter)
        return 0;
    return _excitationGetter->getExcitation(s);
}

//==============================================================================
// PROTECTED METHODS
//==============================================================================
double MuscleActivationDynamics::clampToValidInterval(double val) const
{   return clamp(getMinimumActivation(),val,getMaximumActivation()); }

//==============================================================================
// PRIVATE METHODS
//==============================================================================
void MuscleActivationDynamics::setNull()
{
    setAuthors("Thomas Uchida, Ajay Seth, Michael Sherman");
    _excitationGetter = NULL;
}

void MuscleActivationDynamics::constructProperties()
{
    constructProperty_minimum_activation(0);
    constructProperty_maximum_activation(1);
    constructProperty_default_activation(0.5);
}
