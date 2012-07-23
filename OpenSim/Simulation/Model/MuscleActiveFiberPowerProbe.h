#ifndef OPENSIM_ACTIVE_FIBER_POWER_PROBE_H_
#define OPENSIM_ACTIVE_FIBER_POWER_PROBE_H_

// MuscleActiveFiberPowerProbe.h
// Author: Matt Millard
/* 
 * Permission is hereby granted, free of charge, to any person obtaining a    *
 * copy of this software and associated documentation files (the "Software"), *
 * to deal in the Software without restriction, including without limitation  *
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,   *
 * and/or sell copies of the Software, and to permit persons to whom the      *
 * Software is furnished to do so, subject to the following conditions:       *
 *                                                                            *
 * The above copyright notice and this permission notice shall be included in *
 * all copies or substantial portions of the Software.                        *
 *                                                                            *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR *
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,   *
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL    *
 * THE AUTHORS, CONTRIBUTORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,    *
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR      *
 * OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE  *
 * USE OR OTHER DEALINGS IN THE SOFTWARE.                                     *
 * -------------------------------------------------------------------------- */


// INCLUDE
#include "Muscle.h"
#include "Probe.h"
#include "Model.h"


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
    /** @name Property declarations
    These are the serializable properties associated with this class. **/
    /**@{**/
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
    SimTK::Vector computeProbeInputs(const SimTK::State& state) const OVERRIDE_11;

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
    void connectToModel(Model& aModel) OVERRIDE_11;
    
    void setNull();
    void constructProperties();

//==============================================================================
};	// END of class MuscleActiveFiberPowerProbe
//==============================================================================
//==============================================================================

} // end of namespace OpenSim

#endif // OPENSIM_ACTUATOR_POWER_PROBE_H_


