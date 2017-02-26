/* -------------------------------------------------------------------------- *
 *              OpenSim:  FirstOrderMuscleActivationDynamics.cpp              *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Thomas Uchida, Matthew Millard, Ajay Seth                       *
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

#include "FirstOrderMuscleActivationDynamics.h"

using namespace std;
using namespace OpenSim;
using namespace SimTK;

const std::string FirstOrderMuscleActivationDynamics::
                  STATE_NAME_ACTIVATION = "activation";

//==============================================================================
// CONSTRUCTORS
//==============================================================================
FirstOrderMuscleActivationDynamics::FirstOrderMuscleActivationDynamics() :
MuscleActivationDynamics()
{
    setNull();
    constructProperties();
    setName("default_FirstOrderMuscleActivationDynamics");
}

FirstOrderMuscleActivationDynamics::
FirstOrderMuscleActivationDynamics(const std::string& name,
                                   ExcitationGetter* getter) :
MuscleActivationDynamics(name, getter)
{
    setNull();
    constructProperties();
}

//==============================================================================
// ACCESSORS AND MUTATORS
//==============================================================================
double FirstOrderMuscleActivationDynamics::getActivationTimeConstant() const
{   return get_activation_time_constant(); }
double FirstOrderMuscleActivationDynamics::getDeactivationTimeConstant() const
{   return get_deactivation_time_constant(); }

void FirstOrderMuscleActivationDynamics::
setActivationTimeConstant(double activationTimeConstant)
{
    set_activation_time_constant(max(SimTK::SignificantReal,
                                     activationTimeConstant));
}

void FirstOrderMuscleActivationDynamics::
setDeactivationTimeConstant(double deactivationTimeConstant)
{
    set_deactivation_time_constant(max(SimTK::SignificantReal,
                                       deactivationTimeConstant));
}

//==============================================================================
// MODELCOMPONENT INTERFACE REQUIREMENTS
//==============================================================================
void FirstOrderMuscleActivationDynamics::
extendAddToSystem(SimTK::MultibodySystem& system) const
{
    Super::extendAddToSystem(system);
    addStateVariable(STATE_NAME_ACTIVATION, SimTK::Stage::Dynamics);
}

void FirstOrderMuscleActivationDynamics::
extendInitStateFromProperties(SimTK::State& s) const
{
    Super::extendInitStateFromProperties(s);
    setActivation(s, getDefaultActivation());
}

void FirstOrderMuscleActivationDynamics::
extendSetPropertiesFromState(const SimTK::State& s)
{
    Super::extendSetPropertiesFromState(s);
    setDefaultActivation(getActivation(s));
}

void FirstOrderMuscleActivationDynamics::
computeStateVariableDerivatives(const SimTK::State& s) const
{
     double adot = 
         calcActivationDerivative(getExcitation(s), getActivation(s));

     setStateVariableDerivativeValue(s, STATE_NAME_ACTIVATION, adot);
}

//==============================================================================
// STATE-DEPENDENT METHODS
//==============================================================================
double FirstOrderMuscleActivationDynamics::
getActivation(const SimTK::State& s) const
{
    return clampToValidInterval(getStateVariableValue(s, STATE_NAME_ACTIVATION));
}

void FirstOrderMuscleActivationDynamics::setActivation(SimTK::State& s,
                                                       double activation) const
{
    setStateVariableValue(s, STATE_NAME_ACTIVATION,
                     clampToValidInterval(activation));
}

//==============================================================================
// PRIVATE METHODS
//==============================================================================
void FirstOrderMuscleActivationDynamics::setNull()
{
    setAuthors("Thomas Uchida, Matthew Millard, Ajay Seth");
}

void FirstOrderMuscleActivationDynamics::constructProperties()
{
    constructProperty_activation_time_constant(0.010);
    constructProperty_deactivation_time_constant(0.040);
}

double FirstOrderMuscleActivationDynamics::
calcActivationDerivative(double excitation, double activation) const
{
    excitation = clampToValidInterval(excitation);
    activation = clampToValidInterval(activation);
    double tau = (excitation > activation)
                 ? getActivationTimeConstant() * (0.5 + 1.5*activation)
                 : getDeactivationTimeConstant() / (0.5 + 1.5*activation);
    return (excitation-activation)/tau;
}
