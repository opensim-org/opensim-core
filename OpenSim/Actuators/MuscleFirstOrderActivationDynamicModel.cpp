/* -------------------------------------------------------------------------- *
 *            OpenSim:  MuscleFirstOrderActivationDynamicModel.cpp            *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Matthew Millard                                                 *
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
#include "MuscleFirstOrderActivationDynamicModel.h"

using namespace std;
using namespace OpenSim;
using namespace SimTK;

//==============================================================================
// CONSTRUCTION
//==============================================================================
MuscleFirstOrderActivationDynamicModel::MuscleFirstOrderActivationDynamicModel()
{
    setNull();
    constructProperties();
    setName("default_MuscleFirstOrderActivationDynamicModel");
}

MuscleFirstOrderActivationDynamicModel::
MuscleFirstOrderActivationDynamicModel(double tauActivation,
                                       double tauDeactivation,
                                       double minActivation,
                                       const std::string& muscleName)
{
    setNull();
    constructProperties();

    std::string name = muscleName + "_activation";
    setName(name);

    set_activation_time_constant(tauActivation);
    set_deactivation_time_constant(tauDeactivation);
    set_minimum_activation(minActivation);
}

void MuscleFirstOrderActivationDynamicModel::setNull()
{
    setAuthors("Matthew Millard");
}

void MuscleFirstOrderActivationDynamicModel::constructProperties()
{
    constructProperty_activation_time_constant(0.010);
    constructProperty_deactivation_time_constant(0.040);
    constructProperty_minimum_activation(0.01);
}

//==============================================================================
// SERVICES
//==============================================================================
double MuscleFirstOrderActivationDynamicModel::
clampActivation(double activation) const
{
    return clamp(get_minimum_activation(), activation, 1.0);
}

double MuscleFirstOrderActivationDynamicModel::
calcDerivative(double activation, double excitation) const
{
    // This model respects a lower bound on activation while preserving the
    // expected steady-state value.
    double clampedExcitation = clamp(get_minimum_activation(), excitation, 1.0);
    double clampedActivation = clamp(get_minimum_activation(), activation, 1.0);
    double tau = SimTK::NaN;

    if(clampedExcitation > clampedActivation) {
        tau = get_activation_time_constant() * (0.5 + 1.5*clampedActivation);
    } else {
        tau = get_deactivation_time_constant() / (0.5 + 1.5*clampedActivation);
    }
    return (clampedExcitation - clampedActivation) / tau;
}

//==============================================================================
// COMPONENT INTERFACE
//==============================================================================
void MuscleFirstOrderActivationDynamicModel::extendFinalizeFromProperties()
{
    Super::extendFinalizeFromProperties();

    std::string errorLocation = getName() +
        " MuscleFirstOrderActivationDynamicModel::extendFinalizeFromProperties";

    // Ensure property values are within appropriate ranges.
    SimTK_ERRCHK1_ALWAYS(get_activation_time_constant() > SimTK::SignificantReal,
        "MuscleFirstOrderActivationDynamicModel::extendFinalizeFromProperties",
        "%s: Activation time constant must be greater than zero",
        getName().c_str());
    SimTK_ERRCHK1_ALWAYS(get_deactivation_time_constant() > SimTK::SignificantReal,
        "MuscleFirstOrderActivationDynamicModel::extendFinalizeFromProperties",
        "%s: Deactivation time constant must be greater than zero",
        getName().c_str());
    SimTK_VALUECHECK_ALWAYS(0.0, get_minimum_activation(),
        1.0-SimTK::SignificantReal, "minimum_activation",
        errorLocation.c_str());
}
