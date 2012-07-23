#ifndef OPENSIM_FORCE_PROBE_H_
#define OPENSIM_FORCE_PROBE_H_

// ForceProbe.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHOR: Tim Dorn
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
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

#include "Probe.h"
#include "Model.h"

namespace OpenSim { 

//==============================================================================
//                               FORCE PROBE
//==============================================================================
/**
 * ForceProbe is a ModelComponent Probe for computing an operation on a 
 * force or sum of forces in the model during a simulation.
 * E.g. Impulse is the integral of force with respect to time.
 *
 * @author Tim Dorn
 */


class OSIMSIMULATION_API ForceProbe : public Probe {
OpenSim_DECLARE_CONCRETE_OBJECT(ForceProbe, Probe);
public:
//==============================================================================
// PROPERTIES
//==============================================================================
    /** @name Property declarations
    These are the serializable properties associated with this class. **/
    /**@{**/

    /** List of Forces to probe.  **/
    OpenSim_DECLARE_LIST_PROPERTY(force_names, std::string,
        "Specify a list of model Forces whose value should be calculated.");

    /** Flag to specify whether to report the sum of all forces,
        or report each force value separately.  **/
    OpenSim_DECLARE_PROPERTY(sum_forces_together, bool,
        "Flag to specify whether to report the sum of all forces, "
        "or report each force value separately.");

    /** Exponent to apply to each force prior to the Probe operation. 
    For example, if two forces F1 and F2 are given in force_names, then the
    Probe value will be equal to Force_F1^exponent + Force_F2^exponent.  **/
    OpenSim_DECLARE_PROPERTY(exponent, double,
        "Exponent to apply to each force prior to the Probe operation.");

    /**@}**/

//==============================================================================
// PUBLIC METHODS
//==============================================================================
    //--------------------------------------------------------------------------
    // Constructor(s) and Setup
    //--------------------------------------------------------------------------
    /** Default constructor */
    ForceProbe();
    /** Convenience constructor */
    ForceProbe(const Array<std::string>& force_names, 
        const bool sum_forces_together, const double exponent);

    // Uses default (compiler-generated) destructor, copy constructor, and copy
    // assignment operator.

    //--------------------------------------------------------------------------
    // Get and Set
    //--------------------------------------------------------------------------
    /** Returns the name(s) of the Forces being probed. */
    const Property<std::string>& getForceNames() const;

    /** Returns whether to report sum of all forces together
        or report the forces individually. */
    const bool getSumForcesTogether() const;

    /** Returns the exponent to apply to each force. */
    const double getExponent() const;

    /** Sets the name(s) of the Forces being probed. */
    void setForceNames(const Array<std::string>& forceNames);

    /** Sets whether to report sum of all force values together
        or report the force values individually. */
    void setSumForcesTogether(bool sum_forces_together);

    /** Sets the exponent to apply to each force. */
    void setExponent(const double exponent);


    //-----------------------------------------------------------------------------
    // Computation
    //-----------------------------------------------------------------------------
    /** Compute the Force */
    virtual SimTK::Vector computeProbeInputs(const SimTK::State& state) const OVERRIDE_11;

    /** Returns the number of probe inputs in the vector returned by computeProbeInputs(). */
    int getNumProbeInputs() const OVERRIDE_11;

    /** Returns the column labels of the probe values for reporting.  */
    virtual OpenSim::Array<std::string> getProbeLabels() const OVERRIDE_11;


//==============================================================================
// PRIVATE
//==============================================================================
private:
    //--------------------------------------------------------------------------
    // ModelComponent Interface
    //--------------------------------------------------------------------------
    void connectToModel(Model& model) OVERRIDE_11;

    void setNull();
    void constructProperties();

//==============================================================================
};	// END of class ForceProbe

}; //namespace
//==============================================================================
//==============================================================================


#endif // OPENSIM_FORCE_PROBE_H_
