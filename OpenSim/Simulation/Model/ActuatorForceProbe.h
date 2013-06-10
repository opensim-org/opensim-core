#ifndef OPENSIM_ACTUATOR_FORCE_PROBE_H_
#define OPENSIM_ACTUATOR_FORCE_PROBE_H_
/* -------------------------------------------------------------------------- *
 *                       OpenSim:  ActuatorForceProbe.h                       *
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

#include "Probe.h"
#include "Model.h"

namespace OpenSim { 

//==============================================================================
//                               FORCE PROBE
//==============================================================================
/**
 * ActuatorForceProbe is a ModelComponent Probe for computing an operation on a 
 * force or sum of forces in the model during a simulation.
 * E.g. Impulse is the integral of force with respect to time.
 *
 * @author Tim Dorn
 */


class OSIMSIMULATION_API ActuatorForceProbe : public Probe {
OpenSim_DECLARE_CONCRETE_OBJECT(ActuatorForceProbe, Probe);
public:
//==============================================================================
// PROPERTIES
//==============================================================================
    /** @name Property declarations
    These are the serializable properties associated with this class. **/
    /**@{**/

    /** List of Actuator forces to probe.  **/
    OpenSim_DECLARE_LIST_PROPERTY(actuator_names, std::string,
        "Specify a list of model actuator forces whose value should be calculated."
        "Use 'all' to probe all actuators.");

    /** Flag to specify whether to report the sum of all forces,
        or report each force value separately.  **/
    OpenSim_DECLARE_PROPERTY(sum_forces_together, bool,
        "Flag to specify whether to report the sum of all actuator forces, "
        "or report each force value separately.");

    /** Element-wise power exponent to apply to each force prior to the Probe operation. 
    For example, if two actuators A1 and A2 are given in actuator_names, then the
    Probe value will be equal to Force_A1^exponent + Force_A2^exponent.  **/
    OpenSim_DECLARE_PROPERTY(exponent, double,
        "Element-wise power exponent to apply to each actuator force prior to the Probe operation.");

    /**@}**/

//==============================================================================
// PUBLIC METHODS
//==============================================================================
    //--------------------------------------------------------------------------
    // Constructor(s) and Setup
    //--------------------------------------------------------------------------
    /** Default constructor */
    ActuatorForceProbe();
    /** Convenience constructor */
    ActuatorForceProbe(const Array<std::string>& actuator_names, 
        const bool sum_forces_together, const double exponent);

    // Uses default (compiler-generated) destructor, copy constructor, and copy
    // assignment operator.

    //--------------------------------------------------------------------------
    // Get and Set
    //--------------------------------------------------------------------------
    /** Returns the name(s) of the Actuator forces being probed. */
    const Property<std::string>& getActuatorNames() const;

    /** Returns whether to report sum of all Actuator forces together
        or report the forces individually. */
    const bool getSumForcesTogether() const;

    /** Returns the exponent to apply to each Actuator force. */
    const double getExponent() const;

    /** Sets the name(s) of the Actuator forces being probed. */
    void setActuatorNames(const Array<std::string>& actuatorNames);

    /** Sets whether to report sum of all Actuator force values together
        or report the force values individually. */
    void setSumForcesTogether(bool sum_forces_together);

    /** Sets the exponent to apply to each Actuator force. */
    void setExponent(const double exponent);


    //-----------------------------------------------------------------------------
    // Computation
    //-----------------------------------------------------------------------------
    /** Compute the Force */
    virtual SimTK::Vector computeProbeInputs(const SimTK::State& state) const OVERRIDE_11;

    /** Returns the number of probe inputs in the vector returned by computeProbeInputs(). */
    int getNumProbeInputs() const OVERRIDE_11;

    /** Returns the column labels of the probe values for reporting. 
        Currently uses the Probe name as the column label, so be sure
        to name your probe appropiately! */
    virtual OpenSim::Array<std::string> getProbeOutputLabels() const OVERRIDE_11;

    void connectToModel(Model& model) OVERRIDE_11 FINAL_11;
//==============================================================================
// PRIVATE
//==============================================================================
private:
    // The index inside OpenSim::ActuatorSet that corresponds to each actuator
    // force being probed.
    SimTK::Array_<int> _actuatorIndex;

    //--------------------------------------------------------------------------
    // ModelComponent Interface
    //--------------------------------------------------------------------------
    void setNull();
    void constructProperties();

//==============================================================================
};	// END of class ActuatorForceProbe

}; //namespace
//==============================================================================
//==============================================================================


#endif // OPENSIM_ACTUATOR_FORCE_PROBE_H_
