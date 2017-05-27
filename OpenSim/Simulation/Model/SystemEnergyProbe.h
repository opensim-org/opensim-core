#ifndef OPENSIM_SYSTEM_ENERGY_PROBE_H_
#define OPENSIM_SYSTEM_ENERGY_PROBE_H_
/* -------------------------------------------------------------------------- *
 *                       OpenSim:  SystemEnergyProbe.h                        *
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


// INCLUDE
#include "Probe.h"

namespace OpenSim {

class Model;

//==============================================================================
//                         SYSTEM ENERGY PROBE
//==============================================================================
/**
 * SystemEnergyProbe is a ModelComponent Probe for computing an operation on a 
 * total system energy during a simulation.
 * E.g. Work is the integral of power with respect to time.
 *
 * @author Tim Dorn
 */
class OSIMSIMULATION_API SystemEnergyProbe : public Probe {
OpenSim_DECLARE_CONCRETE_OBJECT(SystemEnergyProbe, Probe);
public:
//==============================================================================
// PROPERTIES
//==============================================================================
    /** Default is true. **/
    OpenSim_DECLARE_PROPERTY(compute_kinetic_energy, bool,
        "Specify whether kinetic energy is to be included in the system energy"
        " computation (true/false).");
    /** Default is true. **/
    OpenSim_DECLARE_PROPERTY(compute_potential_energy, bool,
        "Specify whether potential energy is to be included in the system"
        " energy computation (true/false).");

//==============================================================================
// PUBLIC METHODS
//==============================================================================
    //--------------------------------------------------------------------------
    // Constructor(s) and Setup
    //--------------------------------------------------------------------------
    /** Default constructor */
    SystemEnergyProbe();
    /** Convenience constructor */
    SystemEnergyProbe(bool computeKE, bool computePE);

    // Uses default (compiler-generated) destructor, copy constructor, copy 
    // assignment operator.

    //--------------------------------------------------------------------------
    // Get and Set
    //--------------------------------------------------------------------------
    /** Returns whether kinetic energy is to be included in the system energy 
    computation. */
    bool getComputeKineticEnergy() const;

    /** Returns whether potential energy is to be included in the system energy 
    computation. */
    bool getComputePotentialEnergy() const;

    /** Sets whether kinetic energy is to be included in the system energy 
    computation. */
    void setComputeKineticEnergy(bool c);

    /** Sets whether potential energy is to be included in the system energy 
    computation. */
    void setComputePotentialEnergy(bool c);


    //--------------------------------------------------------------------------
    // Computation
    //--------------------------------------------------------------------------
    /** Compute the System energy which the Probe operation will be based on. */
    SimTK::Vector computeProbeInputs(const SimTK::State& state) const override;

    /** Returns the number of probe inputs in the vector returned by computeProbeInputs(). */
    int getNumProbeInputs() const override;

    /** Returns the column labels of the probe values for reporting. 
        Currently uses the Probe name as the column label, so be sure
        to name your probe appropriately! */
    virtual OpenSim::Array<std::string> getProbeOutputLabels() const override;

//==============================================================================
// PRIVATE
//==============================================================================
private:
    //--------------------------------------------------------------------------
    // Implement the ModelComponent Interface
    //--------------------------------------------------------------------------
    void extendConnectToModel(Model& aModel) override;
    
    void setNull();
    void constructProperties();
//==============================================================================
};  // END of class SystemEnergyProbe
//==============================================================================
//==============================================================================

} // end of namespace OpenSim

#endif // OPENSIM_SYSTEM_ENERGY_PROBE_H_


