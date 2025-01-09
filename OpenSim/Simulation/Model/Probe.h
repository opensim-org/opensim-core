#ifndef OPENSIM_PROBE_H_
#define OPENSIM_PROBE_H_
/* -------------------------------------------------------------------------- *
 *                             OpenSim:  Probe.h                              *
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
#include "ModelComponent.h"

namespace OpenSim {

class Model;

//==============================================================================
//==============================================================================
/**
 * This class represents a Probe which is designed to query a Vector of model
 * values given system state. This model quantity is specified as a
 * SimTK::Vector by the pure virtual method computeProbeInputs(), which must be
 * specified for each child Probe.  In addition, the Probe model component
 * interface allows <I> operations </I> to be performed on this value
 * (specified by the property: probe_operation), and then have this result
 * scaled (by the scalar property: 'scale_factor'). 
 * 
 * The data flow of the Probe is shown below:
  \code
 
  ===========================
  |  SimTK::Vector          |       DEVELOPER NEEDS TO IMPLEMENT THIS
  |  computeProbeInputs(s)  |
  ===========================
               |
               |
               |                    THIS FUNCTIONALITY BELOW IS
               |                    PROVIDED BY THE PROBE INTERFACE
  |------------|---------------------------------------------------------|
  |            V                                                         |
  |   =========================         ======================           |
  |   |  Apply the operation  |  ---->  |  Scale the output  |           |
  |   |   'probe_operation'   |         |      'gain'        |           |
  |   =========================         ======================           |
  |                                               |                      |
  |                                               V                      |
  |                                     ========================         |  
  |                                     |  SimTK::Vector       |         |
  |                                     |  getProbeOutputs(s)  |---------------->
  |                                     ========================         |
  |                                    This method is called by the      |
  |                                   ProbeReporter, or alternatively    |
  |                                        by the API developer          |
  |----------------------------------------------------------------------|

  \endcode
 * The model query is performed at Stage::Report, so that model values are up
 * to date and is based on the specific Probe's overridden method
 * computeProbeInputs(s).  The final output of the probe is available by
 * accessing getProbeOutputs(s).  Note that all queries, operations, and
 * scaling are performed by SimTK::Measures.  Note also that to define a new
 * child Probe class, three methods which are declared as pure virtual in this
 * Probe abstract class need to be overridden:\n
 * - computeProbeInputs()     ---   returns the input probe values (i.e., model
 *                                  queries).
 * - getNumProbeInputs()      ---   returns the size of the vector of input
 *                                  probe values (i.e., model queries).
 * - getProbeOutputLabels()   ---   returns the labels that correspond to each
 *                                  probe value.
 *
 * <B> Available probe operations: </B>
 * - 'value' (default): returns the probe input value.
 * - 'integrate'      : returns the integral of the probe input value.
 * - 'differentiate'  : returns the derivative of the probe input value.
 * - 'minimum'        : returns the minimum of the probe input value.
 * - 'minabs'         : returns the absolute minimum of the probe input value
 *                      (always positive).
 * - 'maximum'        : returns the maximum of the probe input value.
 * - 'maxabs'         : returns the absolute maximum of the probe input value
 *                      (always positive).
 *
 * The Probe interface differs from the Analysis interface in two fundamental
 * ways:
 * -  (1) Operations can be performed on probes (i.e., in addition to simply
 *        reporting a model value, model values (probe input values) may have
 *        operations performed on them such as integration and
 *        differentiation).
 *
 * -  (2) Analyses are not formally part of the model structure (i.e. they are
 *        not ModelComponents), and because of this, analysis results can not
 *        be accessed with the model and state value -- they can only be
 *        accessed by file at the end of a simulation. Probes, on the other
 *        hand, are ModelComponents and therefore can be accessed at any time
 *        during a simulation from the API, and can also be used to compute
 *        model values that are fed back into the system via custom
 *        designed Controllers. Note that Probe values can also be reported to
 *        file at the end of a simulation by attaching a ProbeReporter analysis
 *        to the simulation.
 *
 *
 * @author Tim Dorn
 */

class OSIMSIMULATION_API Probe : public ModelComponent {
OpenSim_DECLARE_ABSTRACT_OBJECT(Probe, ModelComponent);
public:
//==============================================================================
// PROPERTIES
//==============================================================================
    /** Enabled (true) by default. **/
    OpenSim_DECLARE_PROPERTY(enabled, bool,
        "Flag indicating whether the Probe is Enabled.");

    OpenSim_DECLARE_PROPERTY(probe_operation, std::string,
        "The operation to perform on the probe input value: "
        "'value'(no operation, just return the probe value), 'integrate', "
        "'differentiate', 'minimum', 'minabs', 'maximum', 'maxabs'.");

    OpenSim_DECLARE_LIST_PROPERTY(initial_conditions_for_integration, double,
        "Array of initial conditions to be specified if the 'integrate' operation is "
        "selected. Note that the size of initial conditions must be the same size as "
        "the data being integrated, otherwise an exception will be thrown.");

    OpenSim_DECLARE_PROPERTY(gain, double,
        "Constant gain to scale the probe output by.");

//==============================================================================
// OUTPUTS
//==============================================================================
    OpenSim_DECLARE_OUTPUT(probe_outputs, SimTK::Vector, getProbeOutputs,
            SimTK::Stage::Report);

//=============================================================================
// METHODS
//=============================================================================
    Probe();

    // Uses default (compiler-generated) destructor, copy constructor, copy 
    // assignment operator.

    /** Reset (initialize) the underlying Probe SimTK::Measure. */
    void reset(SimTK::State& s);

    /** Get the number of states in the underlying SimTK::Measure. */
    int getNumInternalMeasureStates() const;

    /** Returns true if the Probe is enabled and false if the probe is 
        disabled. */
    bool isEnabled() const;
    
    /** %Set the Probe as enabled (true) or disabled (false). */
    void setEnabled(bool enabled);

    /** Return the operation being performed on the probe value. */
    std::string getOperation() const;
    /** %Set the operation being performed on the probe value. */
    void setOperation(std::string probe_operation);

    /** Return the initial conditions (when the probe_operation is set to 'integrate'). */
    SimTK::Vector getInitialConditions() const;
    /** %Set the initial conditions (when the probe_operation is set to 'integrate'). */
    void setInitialConditions(SimTK::Vector initial_conditions_for_integration);

    /** Return the gain to apply to the probe output. */
    double getGain() const;
    /** %Set the gain to apply to the probe output. */
    void setGain(double gain);
    

#ifndef SWIG
    // This is the Probe interface that must be implemented by concrete Probe
    // objects.

    /** Returns the column labels of the Probe values for reporting. 
        This method must be overridden for each subclass Probe to
        provide meaningful names to the probe outputs. 

        @return         The Array<std::string> of Probe labels. **/
    virtual OpenSim::Array<std::string> getProbeOutputLabels() const=0;


    /**Computes the values of the probe inputs prior to any operation being performed.
       This method must be overridden for each subclass Probe.

    @param  state   System state from which value is computed.  
    @return         The SimTK::Vector of Probe input values. **/
    virtual SimTK::Vector computeProbeInputs(const SimTK::State& state) const=0;


    /**Returns the number of probe inputs in the vector returned by computeProbeInputs().
       This method must be overridden for each subclass Probe.

    @return         The SimTK::Vector of Probe input values. **/
    virtual int getNumProbeInputs() const=0;
#endif

    /** Returns the values of the probe after the operation has been performed.

    @param  state   System state from which value is computed.  
    @return         The SimTK::Vector of probe output values.**/
    SimTK::Vector getProbeOutputs(const SimTK::State& state) const;

    void updateFromXMLNode(SimTK::Xml::Element& node,
                           int versionNumber) override;

protected:
    // ModelComponent interface.
    /** Concrete probes may override; be sure to invoke
    Super::extendConnectToModel() at the beginning of the overriding method.
     **/
    void extendConnectToModel(Model& model) override;

    /** Concrete probes may override; be sure to invoke
    Super::extendAddToSystem() at the beginning of the overriding method. **/
    void extendAddToSystem(SimTK::MultibodySystem& system) const override;


private:
    void setNull();
    void constructProperties();


//=============================================================================
// DATA
//=============================================================================
    //SimTK::Measure_<SimTK::Vector> afterOperationValueVector;
    SimTK::Array_<SimTK::Measure> afterOperationValues;


//=============================================================================
};  // END of class Probe
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // OPENSIM_PROBE_H_


