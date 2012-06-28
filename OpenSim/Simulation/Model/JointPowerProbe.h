#ifndef OPENSIM_JOINT_POWER_PROBE_H_
#define OPENSIM_JOINT_POWER_PROBE_H_

// JointPowerProbe.h
// Author: Ajay Seth, Tim Dorn
/*
 * Copyright (c)  2011, Stanford University. All rights reserved. 
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


// INCLUDE
#include "Probe.h"
#include "Model.h"


namespace OpenSim {

//==============================================================================
//                               JOINT POWER PROBE
//==============================================================================
/**
 * JointPowerProbe is a ModelComponent Probe for computing an operation on a 
 * joint power or sum of joint powers in the model during a simulation.
 * E.g. Joint work is the integral of joint power with respect to time, so by using the
 * JointPowerProbe with the 'integrate' operation, Joint work may be computed.
 *
 * @author Tim Dorn
 * @version 1.0
 */
class OSIMSIMULATION_API JointPowerProbe : public Probe {
OpenSim_DECLARE_CONCRETE_OBJECT(JointPowerProbe, Probe);
public:
//==============================================================================
// PROPERTIES
//==============================================================================
    /** @name Property declarations
    These are the serializable properties associated with this class. Others
    are inherited from the superclass. **/
    /**@{**/

    /** List of Joints to probe.  **/
    OpenSim_DECLARE_LIST_PROPERTY(joint_names, std::string,
        "Specify a list of model Joints whose work should be calculated. "
        "If multiple Joints are given, the probe value will be the summation"
        " of all joint powers.");

    /** Exponent to apply to each joint power prior to the Probe operation. 
    For example, if two joints J1 and J2 are given in joint_names, then the
    Probe value will be equal to JointPower_J1^exponent + JointPower_J2^exponent.  **/
    OpenSim_DECLARE_PROPERTY(exponent, double,
        "Exponent to apply to each joint power prior to the Probe operation.");
    /**@}**/

//=============================================================================
// PUBLIC METHODS
//=============================================================================
public:
    //--------------------------------------------------------------------------
    // Constructor(s) and Setup
    //--------------------------------------------------------------------------
    /** Default constructor */
    JointPowerProbe();
    /** Convenience constructor */
    JointPowerProbe(const Array<std::string>& joint_names, const double exponent);

    // Uses default (compiler-generated) destructor, copy constructor, copy 
    // assignment operator.

    //--------------------------------------------------------------------------
    // Get and Set
    //--------------------------------------------------------------------------
    /** Returns the names of the Joints being probed. */
    const Property<std::string>& getJointNames() const;

    /** Sets the names of the Joints being probed. */
    void setJointNames(const Array<std::string>& aJointNames);

    /** Returns the exponent to apply to each joint power. */
    const double getExponent() const;

    /** Sets the exponent to apply to each joint power. */
    void setExponent(const double exponent);


    //--------------------------------------------------------------------------
    // Computation
    //--------------------------------------------------------------------------
    /** Compute the Joint power. **/
    double computeProbeValue(const SimTK::State& state) const OVERRIDE_11;


//==============================================================================
// PRIVATE
//==============================================================================
private:
    //--------------------------------------------------------------------------
    // ModelComponent Interface
    //--------------------------------------------------------------------------
    void connectToModel(Model& aModel) OVERRIDE_11;
    
    void setNull();
    void constructProperties();

//=============================================================================
};	// END of class JointPowerProbe
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // OPENSIM_JOINT_POWER_PROBE_H_


