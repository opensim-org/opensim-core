/* -------------------------------------------------------------------------- *
 *            OpenSim:  MuscleFirstOrderActivationDynamicModel.cpp            *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
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
    activation = clamp(get_minimum_activation(), activation, 1.0);

    double tau = (excitation > activation) ?
        get_activation_time_constant() * (0.5 + 1.5*activation) : 
        get_deactivation_time_constant() / (0.5 + 1.5*activation);

    return (excitation - activation) / tau;
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
    OPENSIM_THROW_IF_FRMOBJ(
        get_activation_time_constant() < SimTK::SignificantReal,
        InvalidPropertyValue,
        getProperty_activation_time_constant().getName(),
        "Activation time constant must be greater than zero");
    OPENSIM_THROW_IF_FRMOBJ(
        get_deactivation_time_constant() < SimTK::SignificantReal,
        InvalidPropertyValue,
        getProperty_deactivation_time_constant().getName(),
        "Deactivation time constant must be greater than zero");
    OPENSIM_THROW_IF_FRMOBJ(
        get_minimum_activation() < 0 ||
        get_minimum_activation() > 1.0-SimTK::SignificantReal,
        InvalidPropertyValue,
        getProperty_minimum_activation().getName(),
        "Minimum activation must be in the range [0, 1)");
}
