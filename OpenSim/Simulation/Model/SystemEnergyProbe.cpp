// SystemEnergyProbe.cpp
// Authors: Tim Dorn
/*
 * Copyright (c)  2012, Stanford University. All rights reserved. 
* Use of the OpenSim software in source form is permitted provided that the following
* conditions are met:
* 	1. The software is used only for non-commercial research and education. It may not
*     be used in relation to any commercial activity.
* 	2. The software is not distributed or redistributed.  Software distribution is allowed 
*     only through https://simtk.org/home/opensim.
* 	3. Use of the OpenSim software or derivatives must be acknowledged in all publications,
*      presentations, or documents describing work in which OpenSim or derivatives are used.
* 	4. Credits to developers may not be removed from executables
*     created from modifications of the source.
* 	5. Modifications of source code must retain the above copyright notice, this list of
*     conditions and the following disclaimer. 
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
*  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
*  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
*  SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
*  TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
*  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR BUSINESS INTERRUPTION) OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
*  WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

//==============================================================================
// INCLUDE
//==============================================================================
#include "SystemEnergyProbe.h"

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
    // no data members
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
void SystemEnergyProbe::connectToModel(Model& aModel)
{
    Super::connectToModel(aModel);
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
 * Provide labels for the probe values being reported.
 */
Array<string> SystemEnergyProbe::getProbeLabels() const 
{
    Array<string> labels;

    if (getScaleFactor() != 1.0) {
        char n[10];
        sprintf(n, "%f", getScaleFactor());
        labels.append(getName()+"_SCALED_BY_"+n+"X");
    }
    else
        labels.append(getName()+"_"+getOperation());

    return labels;
}
