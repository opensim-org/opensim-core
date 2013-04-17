/* -------------------------------------------------------------------------- *
 *                  OpenSim:  MetabolicMuscleParameter.cpp                    *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Tim Dorn                                                        *
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
// INCLUDES & STATICS
//=============================================================================
#include "MetabolicMuscleParameter.h"

using namespace std;
using namespace SimTK;
using namespace OpenSim;

//=============================================================================
// CONSTRUCTOR
//=============================================================================
// Uses default (compiler-generated) destructor, copy constructor, copy 
// assignment operator.

//_____________________________________________________________________________
/**
 * Default constructor
 */
MetabolicMuscleParameter::MetabolicMuscleParameter() : Object()
{
    setNull();
    constructProperties();
}

//_____________________________________________________________________________
/** 
 * Convenience constructor
 */
MetabolicMuscleParameter::MetabolicMuscleParameter(
    const double muscle_mass, 
    const bool calculate_mass_from_muscle_properties, 
    const double ratio_slow_twitch_fibers, 
    const double activation_constant_slow_twitch, 
    const double activation_constant_fast_twitch, 
    const double maintenance_constant_slow_twitch, 
    const double maintenance_constant_fast_twitch): Object()
{
    setNull();
    constructProperties();

    set_muscle_mass(muscle_mass);
    set_calculate_mass_from_muscle_properties(calculate_mass_from_muscle_properties);
    set_ratio_slow_twitch_fibers(ratio_slow_twitch_fibers);
    set_activation_constant_slow_twitch(activation_constant_slow_twitch);
    set_activation_constant_fast_twitch(activation_constant_fast_twitch);
    set_maintenance_constant_slow_twitch(maintenance_constant_slow_twitch);
    set_maintenance_constant_fast_twitch(maintenance_constant_fast_twitch);
}



//_____________________________________________________________________________
/**
 * Set the data members of this MetabolicMuscleParameter to their null values.
 */
void MetabolicMuscleParameter::setNull()
{
	setAuthors("Tim Dorn");

    // Actual muscle mass used. If calculate_mass_from_muscle_properties = true, 
    // this value will set to the property value <muscle_mass> provided by the 
    // user. If calculate_mass_from_muscle_properties = false, then this value
    // will be set (by the metabolic probes) to the calculated mass based on
    // the muscle's Fmax, opeimal fiber length, and specific tension. 
    _muscMass = 0.0;            
}


//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void MetabolicMuscleParameter::constructProperties(void)
{
    constructProperty_muscle_mass(1.0);
    constructProperty_calculate_mass_from_muscle_properties(false);
    constructProperty_ratio_slow_twitch_fibers(0.5);
    constructProperty_activation_constant_slow_twitch(1.0);
    constructProperty_activation_constant_fast_twitch(1.0);
    constructProperty_maintenance_constant_slow_twitch(1.0);
    constructProperty_maintenance_constant_fast_twitch(1.0);
}
