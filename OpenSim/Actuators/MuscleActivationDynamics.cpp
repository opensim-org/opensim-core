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
MuscleActivationDynamics::MuscleActivationDynamics() {
    setNull();
    constructProperties();
    setName("default_MuscleActivationDynamics");
}

MuscleActivationDynamics::MuscleActivationDynamics(double minimumActivation,
                                                   double maximumActivation,
                                                   double defaultActivation,
                                                   const std::string& name) {
    setNull();
    constructProperties();
    setMinimumActivation(minimumActivation);
    setMaximumActivation(maximumActivation);
    setDefaultActivation(defaultActivation);
    setName(name);
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

void MuscleActivationDynamics::setMinimumActivation(double minimumActivation) {
    // Minimum must be in the interval [0,maximum].
    set_minimum_activation( clamp(0,minimumActivation,getMaximumActivation()) );
    // Default must be in the interval [minimum,maximum].
    setDefaultActivation(getDefaultActivation());
}

void MuscleActivationDynamics::setMaximumActivation(double maximumActivation) {
    // Maximum must be in the interval [minimum,1].
    set_maximum_activation( clamp(getMinimumActivation(),maximumActivation,1) );
    // Default must be in the interval [minimum,maximum].
    setDefaultActivation(getDefaultActivation());
}

void MuscleActivationDynamics::setDefaultActivation(double defaultActivation) {
    // Default must be in the interval [minimum,maximum].
    set_default_activation( clamp(getMinimumActivation(),
                                  defaultActivation,
                                  getMaximumActivation()) );
}

//==============================================================================
// OTHER PUBLIC METHODS
//==============================================================================
double MuscleActivationDynamics::getActivation(const SimTK::State& s) const {
    return getDefaultActivation();
}

//==============================================================================
// PRIVATE METHODS
//==============================================================================
void MuscleActivationDynamics::setNull()
{   setAuthors("Thomas Uchida, Ajay Seth"); }

void MuscleActivationDynamics::constructProperties() {
    constructProperty_minimum_activation(0);
    constructProperty_maximum_activation(1);
    constructProperty_default_activation(0.5);
}
