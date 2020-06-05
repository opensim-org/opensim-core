/* -------------------------------------------------------------------------- *
 *             OpenSim:  ZerothOrderMuscleActivationDynamics.cpp              *
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

#include "ZerothOrderMuscleActivationDynamics.h"

using namespace std;
using namespace OpenSim;
using namespace SimTK;

//==============================================================================
// CONSTRUCTORS
//==============================================================================
ZerothOrderMuscleActivationDynamics::ZerothOrderMuscleActivationDynamics() :
MuscleActivationDynamics()
{
    setNull();
    setName("default_ZerothOrderMuscleActivationDynamics");
}

ZerothOrderMuscleActivationDynamics::
ZerothOrderMuscleActivationDynamics(const std::string& name,
                                    ExcitationGetter* getter) :
MuscleActivationDynamics(name, getter)
{
    setNull();
}

//==============================================================================
// STATE-DEPENDENT METHODS
//==============================================================================
double ZerothOrderMuscleActivationDynamics::
getActivation(const SimTK::State& s) const
{   return clampToValidInterval(getExcitation(s)); }

void ZerothOrderMuscleActivationDynamics::
setActivation(SimTK::State& s, double activation) const
{
    log_warn("Attempting to set activation of '{}', which is of type {}, and, "
             "therefore, has no activation variable to set.", getName(),
            getConcreteClassName());
}

//==============================================================================
// PRIVATE METHODS
//==============================================================================
void ZerothOrderMuscleActivationDynamics::setNull()
{
    setAuthors("Thomas Uchida, Ajay Seth");
}
