/* -------------------------------------------------------------------------- *
 *                      OpenSim:  SystemEnergyProbe.cpp                       *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
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

//==============================================================================
// INCLUDE
//==============================================================================
#include "SystemEnergyProbe.h"
#include "Model.h"

using namespace std;
using namespace SimTK;
using namespace OpenSim;


//==============================================================================
// CONSTRUCTORS
//==============================================================================
// Uses default (compiler-generated) destructor, copy constructor, copy 
// assignment operator.

//_____________________________________________________________________________
// Default constructor.
SystemEnergyProbe::SystemEnergyProbe() 
{
    setNull();
    constructProperties();
}

//_____________________________________________________________________________
// Convenience constructor.
SystemEnergyProbe::SystemEnergyProbe(bool computeKE, bool computePE)
{
    setNull();
    constructProperties();
    set_compute_kinetic_energy(computeKE);
    set_compute_potential_energy(computePE);
}


//_____________________________________________________________________________
// Set the data members of this SystemEnergyProbe to their null values.
void SystemEnergyProbe::setNull(void)
{
    setAuthors("Tim Dorn");
}

//_____________________________________________________________________________
// Allocate and initialize properties.
void SystemEnergyProbe::constructProperties(void)
{
    constructProperty_compute_kinetic_energy(true);
    constructProperty_compute_potential_energy(true);
}

//==============================================================================
// GET AND SET
//==============================================================================
// Returns whether KE is to be included in the system energy computation.
bool SystemEnergyProbe::getComputeKineticEnergy() const
{
    return get_compute_kinetic_energy();
}

//_____________________________________________________________________________
// Returns whether PE is to be included in the system energy computation.
bool SystemEnergyProbe::getComputePotentialEnergy() const
{
    return get_compute_potential_energy();
}

//_____________________________________________________________________________
// Sets whether KE is to be included in the system energy computation.
void SystemEnergyProbe::setComputeKineticEnergy(bool c)
{
    return set_compute_kinetic_energy(c);
}

//_____________________________________________________________________________
// Sets whether PE is to be included in the system energy computation.
void SystemEnergyProbe::setComputePotentialEnergy(bool c)
{
    return set_compute_potential_energy(c);
}

//==============================================================================
// MODEL COMPONENT METHODS
//==============================================================================
//_____________________________________________________________________________
/**
 * Perform some set up functions that happen after the
 * object has been deserialized or copied.
 *
 * @param aModel OpenSim model containing this SystemEnergyProbe.
 */
void SystemEnergyProbe::extendConnectToModel(Model& aModel)
{
    Super::extendConnectToModel(aModel);
}

//==============================================================================
// COMPUTATION
//==============================================================================
//_____________________________________________________________________________
/** 
 * Compute the System energy which the Probe operation will be based on.
 */
SimTK::Vector SystemEnergyProbe::computeProbeInputs(const State& s) const
{
    SimTK::Vector TotalE(1, 0.0);       // Initialize at zero
    
    if (getComputeKineticEnergy())
        TotalE(0) += _model->getMultibodySystem().calcKineticEnergy(s);

    if (getComputePotentialEnergy())
        TotalE(0) += _model->getMultibodySystem().calcPotentialEnergy(s);
    
    return TotalE;
}


//_____________________________________________________________________________
/** 
 * Returns the number of probe inputs in the vector returned by computeProbeInputs().
 */
int SystemEnergyProbe::getNumProbeInputs() const
{
    return 1;
}


//_____________________________________________________________________________
/** 
 * Provide labels for the probe values being reported.
 */
Array<string> SystemEnergyProbe::getProbeOutputLabels() const 
{
    Array<string> labels;
    labels.append(getName());
    return labels;
}
