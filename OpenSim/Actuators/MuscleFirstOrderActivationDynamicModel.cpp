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
    buildModel();
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

    SimTK_ERRCHK1_ALWAYS(tauActivation>SimTK::SignificantReal
                         && tauDeactivation>SimTK::SignificantReal,
        "MuscleFirstOrderActivationDynamicModel::"
        "MuscleFirstOrderActivationDynamicModel", "%s: Activation and "
        "deactivation time constants must be greater than 0",
        name.c_str());

    SimTK_ERRCHK1_ALWAYS(minActivation >= 0
                         && minActivation < 1.0-SimTK::SignificantReal,
        "MuscleFirstOrderActivationDynamicModel::"
        "MuscleFirstOrderActivationDynamicModel",
        "%s: Minimum activation must be in the range [0,1)",
        name.c_str());

    set_activation_time_constant(tauActivation);
    set_deactivation_time_constant(tauDeactivation);
    set_minimum_activation(minActivation);
    buildModel();
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

void MuscleFirstOrderActivationDynamicModel::buildModel()
{
    setObjectIsUpToDateWithProperties();
}

void MuscleFirstOrderActivationDynamicModel::ensureModelUpToDate()
{
    if(!isObjectUpToDateWithProperties()) {
        buildModel();
    }

    // The name is not counted as a property but it can change, so it must be
    // updated as well.
    std::string name = getName();
    setName(name);
}

//==============================================================================
// GET AND SET METHODS
//==============================================================================
double MuscleFirstOrderActivationDynamicModel::getActivationTimeConstant() const
{   return get_activation_time_constant(); }
double MuscleFirstOrderActivationDynamicModel::
getDeactivationTimeConstant() const
{   return get_deactivation_time_constant(); }
double MuscleFirstOrderActivationDynamicModel::getMinimumActivation() const
{   return get_minimum_activation(); }
double MuscleFirstOrderActivationDynamicModel::getMaximumActivation() const
{   return 1.0; }

bool MuscleFirstOrderActivationDynamicModel::
setActivationTimeConstant(double activationTimeConstant)
{
    if(activationTimeConstant > SimTK::SignificantReal) {
        set_activation_time_constant(activationTimeConstant);
        buildModel();
        return true;
    }
    return false;
}

bool MuscleFirstOrderActivationDynamicModel::
setDeactivationTimeConstant(double deactivationTimeConstant)
{
    if(deactivationTimeConstant > SimTK::SignificantReal) {
        set_deactivation_time_constant(deactivationTimeConstant);
        buildModel();
        return true;
    }
    return false;
}

bool MuscleFirstOrderActivationDynamicModel::
setMinimumActivation(double minimumActivation)
{
    if(minimumActivation >= 0
       && minimumActivation < 1.0-SimTK::SignificantReal) {

        set_minimum_activation(minimumActivation);
        buildModel();
        return true;
    }
    return false;
}

//==============================================================================
// SERVICES
//==============================================================================
double MuscleFirstOrderActivationDynamicModel::
clampActivation(double activation) const
{
    return clamp(getMinimumActivation(), activation, 1.0);
}

double MuscleFirstOrderActivationDynamicModel::
calcDerivative(double activation, double excitation) const
{
    // This model respects a lower bound on activation while preserving the
    // expected steady-state value.
    double clampedExcitation = clamp(getMinimumActivation(), excitation, 1.0);
    double clampedActivation = clamp(getMinimumActivation(), activation, 1.0);
    double tau = SimTK::NaN;

    if(clampedExcitation > clampedActivation) {
        tau = getActivationTimeConstant() * (0.5 + 1.5*clampedActivation);
    } else {
        tau = getDeactivationTimeConstant() / (0.5 + 1.5*clampedActivation);
    }
	return (clampedExcitation - clampedActivation) / tau;
}
