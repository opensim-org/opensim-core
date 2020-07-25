#ifndef OPENSIM_JOINT_INTERNAL_POWER_PROBE_H_
#define OPENSIM_JOINT_INTERNAL_POWER_PROBE_H_
/* -------------------------------------------------------------------------- *
 *                    OpenSim:  JointInternalPowerProbe.h                     *
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
//                        JOINT INTERNAL POWER PROBE
//==============================================================================
/**
 * JointInternalPowerProbe is a ModelComponent Probe for computing an operation on 
 * internal joint power or sum of joint powers in the model during a simulation.
 * E.g. Joint internal work is the integral of joint internal power with respect to time, 
 * so by using the JointInternalPowerProbe with the 'integrate' operation, Joint internal
 * work may be computed.
 *
 * @author Tim Dorn
 * @version 1.0
 */
class OSIMSIMULATION_API JointInternalPowerProbe : public Probe {
OpenSim_DECLARE_CONCRETE_OBJECT(JointInternalPowerProbe, Probe);
public:
//==============================================================================
// PROPERTIES
//==============================================================================

    /** List of Joints to probe.  **/
    OpenSim_DECLARE_LIST_PROPERTY(joint_names, std::string,
        "Specify a list of model Joints whose power should be calculated."
        "Use 'all' to probe all joints.");

    /** Flag to specify whether to report the sum of all powers,
        or report each power value separately.  **/
    OpenSim_DECLARE_PROPERTY(sum_powers_together, bool,
        "Flag to specify whether to report the sum of all joint powers, "
        "or report each joint power value separately.");

    /** Element-wise power exponent to apply to each joint power prior to the Probe operation. 
    For example, if two joints J1 and J2 are given in joint_names, then the
    Probe value will be equal to JointPower_J1^exponent + JointPower_J2^exponent.  **/
    OpenSim_DECLARE_PROPERTY(exponent, double,
        "Element-wise power exponent to apply to each joint power prior to the Probe operation.");

//=============================================================================
// PUBLIC METHODS
//=============================================================================
public:
    //--------------------------------------------------------------------------
    // Constructor(s) and Setup
    //--------------------------------------------------------------------------
    /** Default constructor */
    JointInternalPowerProbe();
    /** Convenience constructor */
    JointInternalPowerProbe(const Array<std::string>& joint_names, 
        const bool sum_powers_together, const double exponent);

    // Uses default (compiler-generated) destructor, copy constructor, copy 
    // assignment operator.

    //--------------------------------------------------------------------------
    // Get and Set
    //--------------------------------------------------------------------------
    /** Returns the names of the Joints being probed. */
    const Property<std::string>& getJointNames() const;

    /** Returns whether to report sum of all joint powers together
        or report the joint powers individually. */
    bool getSumPowersTogether() const;

    /** Returns the exponent to apply to each joint power. */
    double getExponent() const;

    /** Sets the names of the Joints being probed. */
    void setJointNames(const Array<std::string>& aJointNames);

    /** Sets whether to report sum of all joint powers together
        or report the joint powers individually. */
    void setSumPowersTogether(bool sum_powers_together);

    /** Sets the exponent to apply to each joint power. */
    void setExponent(const double exponent);


    //--------------------------------------------------------------------------
    // Computation
    //--------------------------------------------------------------------------
    /** Compute the Joint power. **/
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
    // The index inside OpenSim::JointSet that corresponds to each joint
    // being probed.
    SimTK::Array_<int> _jointIndex;

    //--------------------------------------------------------------------------
    // ModelComponent Interface
    //--------------------------------------------------------------------------
    void extendConnectToModel(Model& aModel) override;
    
    void setNull();
    void constructProperties();

//=============================================================================
};  // END of class JointInternalPowerProbe
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // OPENSIM_JOINT_INTERNAL_POWER_PROBE_H_


