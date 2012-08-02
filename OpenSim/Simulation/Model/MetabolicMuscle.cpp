/* -------------------------------------------------------------------------- *
 *                       OpenSim:  MetabolicMuscle.cpp                        *
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
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied    *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

//=============================================================================
// INCLUDES & STATICS
//=============================================================================
#include "MetabolicMuscle.h"

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
MetabolicMuscle::MetabolicMuscle() : Object()
{
    setNull();
    constructProperties();
}

//_____________________________________________________________________________
/** 
 * Convenience constructor
 */
MetabolicMuscle::MetabolicMuscle(double muscle_mass, double ratio_slow_twitch_fibers, 
    double activation_constant_slow_twitch, double activation_constant_fast_twitch, 
    double maintenance_constant_slow_twitch, double maintenance_constant_fast_twitch): Object()
{
    setNull();
    constructProperties();

    set_muscle_mass(muscle_mass);
    set_ratio_slow_twitch_fibers(ratio_slow_twitch_fibers);
    set_activation_constant_slow_twitch(activation_constant_slow_twitch);
    set_activation_constant_fast_twitch(activation_constant_fast_twitch);
    set_maintenance_constant_slow_twitch(maintenance_constant_slow_twitch);
    set_maintenance_constant_fast_twitch(maintenance_constant_fast_twitch);
}



//_____________________________________________________________________________
/**
 * Set the data members of this MetabolicMuscle to their null values.
 */
void MetabolicMuscle::setNull()
{
    // no data members
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void MetabolicMuscle::constructProperties(void)
{
    constructProperty_muscle_mass(1.0);
    constructProperty_ratio_slow_twitch_fibers(0.5);
    constructProperty_activation_constant_slow_twitch(1.0);
    constructProperty_activation_constant_fast_twitch(1.0);
    constructProperty_maintenance_constant_slow_twitch(1.0);
    constructProperty_maintenance_constant_fast_twitch(1.0);
}



//=============================================================================
// GET AND SET
//=============================================================================
//_____________________________________________________________________________
/**
 * Get the parameters.
 */
double MetabolicMuscle::getMuscleMass() const
{
    return get_muscle_mass();
}

double MetabolicMuscle::getRatioSlowTwitchFibers() const
{
    return get_ratio_slow_twitch_fibers();
}

double MetabolicMuscle::getActivationConstantSlowTwitch() const
{
    return get_activation_constant_slow_twitch();
}

double MetabolicMuscle::getActivationConstantFastTwitch() const
{
    return get_activation_constant_fast_twitch();
}

double MetabolicMuscle::getMaintenanceConstantSlowTwitch() const
{
    return get_maintenance_constant_slow_twitch();
}

double MetabolicMuscle::getMaintenanceConstantFastTwitch() const
{
    return get_maintenance_constant_fast_twitch();
}

//_____________________________________________________________________________
/**
 * Set the parameters.
 */

void MetabolicMuscle::setMuscleMass(const double aMuscleMass)
{
    set_muscle_mass(aMuscleMass);
}

void MetabolicMuscle::setRatioSlowTwitchFibers(const double aRatioSlowTwitchFibers)
{
    set_ratio_slow_twitch_fibers(aRatioSlowTwitchFibers);
}

void MetabolicMuscle::setActivationConstantSlowTwitch(const double aActivationConstantSlowTwitch)
{
    set_activation_constant_slow_twitch(aActivationConstantSlowTwitch);
}

void MetabolicMuscle::setActivationConstantFastTwitch(const double aActivationConstantFastTwitch)
{
    set_activation_constant_fast_twitch(aActivationConstantFastTwitch);
}

void MetabolicMuscle::setMaintenanceConstantSlowTwitch(const double aMaintenanceSlowTwitch)
{
    set_maintenance_constant_slow_twitch(aMaintenanceSlowTwitch);
}

void MetabolicMuscle::setMaintenanceConstantFastTwitch(const double aMaintenanceFastTwitch)
{
    set_maintenance_constant_fast_twitch(aMaintenanceFastTwitch);
}

