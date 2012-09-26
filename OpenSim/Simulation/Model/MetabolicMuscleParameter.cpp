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
MetabolicMuscleParameter::MetabolicMuscleParameter() : ModelComponent()
{
    setNull();
    constructProperties();
}

//_____________________________________________________________________________
/** 
 * Convenience constructor
 */
MetabolicMuscleParameter::MetabolicMuscleParameter(double muscle_mass, 
    bool calculate_mass_from_muscle_properties, double ratio_slow_twitch_fibers, 
    double activation_constant_slow_twitch, double activation_constant_fast_twitch, 
    double maintenance_constant_slow_twitch, double maintenance_constant_fast_twitch): ModelComponent()
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
    m = NULL;
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



//=============================================================================
// MODEL COMPONENT METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Perform some set up functions that happen after the
 * object has been deserialized or copied.
 *
 * @param aModel OpenSim model containing this MuscleMetabolicPowerProbeUmberger2003.
 */
void MetabolicMuscleParameter::connectToModel(Model& aModel)
{
    Super::connectToModel(aModel);

    stringstream errorMessage;

    // check that the muscle exists
    int k = _model->getMuscles().getIndex(getName());
    if( k < 0 )	{
        errorMessage << "MetabolicMuscleParameter: Invalid muscle '" 
            << getName() << "' specified." << endl;
        throw (Exception(errorMessage.str()));
    }
    else {
        m = &_model->updMuscles().get(k);
    }

    // set the muscle mass internal member variable: _muscMass
    if (get_calculate_mass_from_muscle_properties()) {
        double sigma = 0.25e6;      // (Pa), specific tension of mammalian muscle.
        double rho = 1059.7;        // (kg/m^3), density of mammalian muscle.

        _muscMass =  (m->getMaxIsometricForce() / sigma) * rho * m->getOptimalFiberLength();
    }
    else {
        _muscMass = get_muscle_mass();

        if (_muscMass <= 0) {
            errorMessage << "MetabolicMuscleParameter: Invalid muscle_mass specified for muscle: " 
                << getName() << ". muscle_mass must be positive." << endl;
            throw (Exception(errorMessage.str()));
        }
    }

    // error checking: ratio_slow_twitch_fibers
    if (getRatioSlowTwitchFibers() < 0 || getRatioSlowTwitchFibers() > 1)	{
        errorMessage << "MetabolicMuscleParameter: Invalid ratio_slow_twitch_fibers for muscle: " 
            << getName() << ". ratio_slow_twitch_fibers must be between 0 and 1." << endl;
        throw (Exception(errorMessage.str()));
    }
}



//=============================================================================
// GET AND SET
//=============================================================================
//_____________________________________________________________________________
/**
 * Get the parameters.
 */
Muscle* MetabolicMuscleParameter::getMuscle()
{
    return m;
}

double MetabolicMuscleParameter::getMuscleMass() const
{
    return _muscMass;
}

double MetabolicMuscleParameter::getRatioSlowTwitchFibers() const
{
    return get_ratio_slow_twitch_fibers();
}

double MetabolicMuscleParameter::getActivationConstantSlowTwitch() const
{
    return get_activation_constant_slow_twitch();
}

double MetabolicMuscleParameter::getActivationConstantFastTwitch() const
{
    return get_activation_constant_fast_twitch();
}

double MetabolicMuscleParameter::getMaintenanceConstantSlowTwitch() const
{
    return get_maintenance_constant_slow_twitch();
}

double MetabolicMuscleParameter::getMaintenanceConstantFastTwitch() const
{
    return get_maintenance_constant_fast_twitch();
}

//_____________________________________________________________________________
/**
 * Set the parameters.
 */

void MetabolicMuscleParameter::setMuscleMass(const double aMuscleMass)
{
    set_muscle_mass(aMuscleMass);
}

void MetabolicMuscleParameter::setRatioSlowTwitchFibers(const double aRatioSlowTwitchFibers)
{
    set_ratio_slow_twitch_fibers(aRatioSlowTwitchFibers);
}

void MetabolicMuscleParameter::setActivationConstantSlowTwitch(const double aActivationConstantSlowTwitch)
{
    set_activation_constant_slow_twitch(aActivationConstantSlowTwitch);
}

void MetabolicMuscleParameter::setActivationConstantFastTwitch(const double aActivationConstantFastTwitch)
{
    set_activation_constant_fast_twitch(aActivationConstantFastTwitch);
}

void MetabolicMuscleParameter::setMaintenanceConstantSlowTwitch(const double aMaintenanceSlowTwitch)
{
    set_maintenance_constant_slow_twitch(aMaintenanceSlowTwitch);
}

void MetabolicMuscleParameter::setMaintenanceConstantFastTwitch(const double aMaintenanceFastTwitch)
{
    set_maintenance_constant_fast_twitch(aMaintenanceFastTwitch);
}

