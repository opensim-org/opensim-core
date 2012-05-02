// SystemEnergyProbe.cpp
// Authors: Tim Dorn
/*
 * Copyright (c)  2006, Stanford University. All rights reserved. 
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

//=============================================================================
// INCLUDES and STATICS
//=============================================================================
#include "SystemEnergyProbe.h"


using namespace std;
using namespace SimTK;
using namespace OpenSim;


//=============================================================================
// CONSTRUCTOR(S) AND SETUP
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */
SystemEnergyProbe::SystemEnergyProbe() 
{
    setNull();
}

//_____________________________________________________________________________
/**
 * Convenience constructor.
 */
SystemEnergyProbe::SystemEnergyProbe(bool computeKE, bool computePE)
{
    setNull();
    setPropertyValue("compute_kinetic_energy", computeKE);
    setPropertyValue("compute_potential_energy", computePE);
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
SystemEnergyProbe::~SystemEnergyProbe()
{
}


//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aSystemEnergyProbe SystemEnergyProbe to be copied.
 */
SystemEnergyProbe::SystemEnergyProbe(const SystemEnergyProbe &aSystemEnergyProbe) :
   Probe(aSystemEnergyProbe)
{
    setNull();
    copyData(aSystemEnergyProbe);
}


//_____________________________________________________________________________
/**
 * Copy data members from one SystemEnergyProbe to another.
 *
 * @param aProbe SystemEnergyProbe to be copied.
 */
void SystemEnergyProbe::copyData(const SystemEnergyProbe &aProbe)
{
    Super::copyData(aProbe);

    setPropertyValue("compute_kinetic_energy", aProbe.getPropertyValue<bool>("compute_kinetic_energy"));
    setPropertyValue("compute_potential_energy", aProbe.getPropertyValue<bool>("compute_potential_energy"));
}

//_____________________________________________________________________________
/**
 * Set the data members of this SystemEnergyProbe to their null values.
 */
void SystemEnergyProbe::setNull(void)
{
    setupProperties();
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void SystemEnergyProbe::setupProperties(void)
{
    addProperty<bool>("compute_kinetic_energy",
        "Specify whether kinetic energy is to be included in the system energy computation (true/false).",
        true);

    addProperty<bool>("compute_potential_energy",
        "Specify whether potential energy is to be included in the system energy computation (true/false).",
        true);
}


//=============================================================================
// OPERATORS
//=============================================================================
//_____________________________________________________________________________
/**
 * Assignment operator.
 *
 * @return Reference to this object.
 */
#ifndef SWIG
SystemEnergyProbe& SystemEnergyProbe::operator=(const SystemEnergyProbe &aSystemEnergyProbe)
{
    // BASE CLASS
    Super::operator=(aSystemEnergyProbe);
    return(*this);
}
#endif



//=============================================================================
// GET AND SET
//=============================================================================
/**
 * Returns whether kinetic energy is to be included in the system energy computation.
 */
bool SystemEnergyProbe::getComputeKineticEnergy() const
{
    return getPropertyValue<bool>("compute_kinetic_energy");
}

//_____________________________________________________________________________
/**
 * Returns whether kinetic energy is to be included in the system energy computation.
 */
bool SystemEnergyProbe::getComputePotentialEnergy() const
{
    return getPropertyValue<bool>("compute_kinetic_energy");
}

//_____________________________________________________________________________
/**
 * Sets whether kinetic energy is to be included in the system energy computation.
 */
void SystemEnergyProbe::setComputeKineticEnergy(bool c)
{
    return setPropertyValue<bool>("compute_kinetic_energy", c);
}

//_____________________________________________________________________________
/**
 * Sets whether potential energy is to be included in the system energy computation.
 */
void SystemEnergyProbe::setComputePotentialEnergy(bool c)
{
    return setPropertyValue<bool>("compute_potential_energy", c);
}



//=============================================================================
// MODEL COMPONENT METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Perform some set up functions that happen after the
 * object has been deserialized or copied.
 *
 * @param aModel OpenSim model containing this SystemEnergyProbe.
 */
void SystemEnergyProbe::setup(Model& aModel)
{
    Super::setup(aModel);
}





//=============================================================================
// COMPUTATION
//=============================================================================
//_____________________________________________________________________________
/**
 * Compute the System energy which the Probe operation will be based on.
 */
double SystemEnergyProbe::computeProbeValue(const State& s) const
{
    double TotalP = 0;
    
    if (getComputeKineticEnergy())
        TotalP += _model->getMultibodySystem().calcKineticEnergy(s);

    if (getComputePotentialEnergy())
        TotalP += _model->getMultibodySystem().calcPotentialEnergy(s);
    
    return(TotalP);
}

