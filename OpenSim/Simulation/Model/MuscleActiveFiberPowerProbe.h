#ifndef OPENSIM_ACTIVE_FIBER_POWER_PROBE_H_
#define OPENSIM_ACTIVE_FIBER_POWER_PROBE_H_
/* -------------------------------------------------------------------------- *
 *                  OpenSim:  MuscleActiveFiberPowerProbe.h                   *
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


// INCLUDE
#include "Probe.h"


namespace OpenSim {

class Model;

//===============================================================================
//                         ACTIVE FIBER POWER PROBE
//===============================================================================
/**
MuscleActiveFiberPowerProbe is a ModelComponent probe for computing an 
operation on the active fiber power of a muscle.  
  
  @author Matt Millard
 */
class OSIMSIMULATION_API MuscleActiveFiberPowerProbe : public Probe {
OpenSim_DECLARE_CONCRETE_OBJECT(MuscleActiveFiberPowerProbe, Probe);

public:
//==============================================================================
// PROPERTIES
//==============================================================================
    /** List of Muscles to probe.  **/
    OpenSim_DECLARE_LIST_PROPERTY(muscle_names, std::string,
        "Specify a list of muscles whose work should be calculated. "
        "If multiple muscles are given, the probe value will be the summation"
        " of all actuator powers.");


//==============================================================================
// PUBLIC METHODS
//==============================================================================
    //--------------------------------------------------------------------------
    // Constructor(s) and Setup
    //--------------------------------------------------------------------------
    /** Default constructor */
    MuscleActiveFiberPowerProbe();
    /** Convenience constructor */
    MuscleActiveFiberPowerProbe(const Array<std::string> muscle_names);

    // Uses default (compiler-generated) destructor, copy constructor, and copy
    // assignment operator.

    //--------------------------------------------------------------------------
    // Get and Set
    //--------------------------------------------------------------------------
    /** Returns the names of the Actuators being probed. */
    const Property<std::string>& getMuscleNames() const;

    /** Sets the names of the Actuators being probed. */
    void setMuscleNames(const Array<std::string>& muscleNames);

    //--------------------------------------------------------------------------
    // Computation
    //--------------------------------------------------------------------------
    /** Compute the Actuator power upon which the Probe operation will be based. */
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
    // ModelComponent Interface
    //--------------------------------------------------------------------------
    void extendConnectToModel(Model& aModel) override;
    
    void setNull();
    void constructProperties();

//==============================================================================
};  // END of class MuscleActiveFiberPowerProbe
//==============================================================================
//==============================================================================

} // end of namespace OpenSim

#endif // OPENSIM_ACTUATOR_POWER_PROBE_H_


