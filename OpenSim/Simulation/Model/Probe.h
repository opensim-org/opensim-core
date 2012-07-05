#ifndef OPENSIM_PROBE_H_
#define OPENSIM_PROBE_H_

// Probe.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHOR: Tim Dorn
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/* Copyright (c)  2012 Stanford University
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
#include "Model.h"
#include "ModelComponent.h"

namespace OpenSim {

class Model;

//==============================================================================
//==============================================================================
/**
 * This class represents a Probe which will can perform an "operation" on any model value.
 * This model value must be a scalar value. Each Probe subclass represents a different 
 * type of Probe. The actual "operation" is done by a SimTK::Measure. 
 * In creating child class Probes, the user must override the computeProbeValue() method.
 *
 * @author Tim Dorn
 */
class OSIMSIMULATION_API Probe : public ModelComponent {
OpenSim_DECLARE_ABSTRACT_OBJECT(Probe, ModelComponent);
public:
//==============================================================================
// PROPERTIES
//==============================================================================
    /** @name Property declarations
    These are the serializable properties associated with this class. **/
    /**@{**/
    /** Enabled by default. **/
    OpenSim_DECLARE_PROPERTY(isDisabled, bool,
        "Flag indicating whether the Probe is disabled or not. Disabled"
        " means that the Probe will not be reported using the ProbeReporter.");

    OpenSim_DECLARE_PROPERTY(operation, std::string,
        "The operation to perform on the probe value: "
        "'value'(no operation, just return the probe value), 'integrate', "
        "'differentiate', 'scale'");

    OpenSim_DECLARE_PROPERTY(operation_parameter, double,
        "Parameter that is required for some operations only. "
        "For 'integrate' operation_parameter is the initial condition, "
        "for 'scale' operation_parameter is the scale factor.");
    /**@}**/

//=============================================================================
// PUBLIC METHODS
//=============================================================================
    Probe();

    // Uses default (compiler-generated) destructor, copy constructor, copy 
    // assignment operator.

    /** Returns true if the Probe is disabled or false if the probe is enabled. */
    bool isDisabled() const;
    /** Set the Probe as disabled (true) or enabled (false). */
    void setDisabled(bool isDisabled);

    /** Return the operation being performed on the probe value. */
    std::string getOperation() const;
    /** Set the operation being performed on the probe value. */
    void setOperation(std::string operation);

    /** Return the operation parameter for the operation. */
    double getOperationParameter() const;
    /** Set the operation parameter for the operation. */
    void setOperationParameter(double operation_parameter);

    
    /** Returns the column labels for the probe values for reporting. 
        Note that in OpenSim 3.0, only scalar values are returned, so 
        developers will need to call getRecordLabels.get(0) to return 
        the probe label.*/
    Array<std::string> getRecordLabels() const;

    /** Returns the probe values post-operation. 
        Note that in OpenSim 3.0, only scalar values are returned, so 
        developers will need to call getRecordValues.get(0) to return 
        the value of the probe (after the operation has been performed)*/
    Array<double> getRecordValues(const SimTK::State& state) const;

#ifndef SWIG
    // This is the Probe interface that must be implemented by concrete Probe
    // objects.

    /**Computes the probe value pre-operation (i.e., prior to any operation being performed on it).
       Probe value is computed at the SimTK::Report Stage.
       This method must be overridden for each subclass Probe.

    @param  state   System state from which value is computed.  
    @return         The SimTK::Vector of probe values. **/
    virtual double computeProbeValue(const SimTK::State& state) const=0;
#endif
protected:
    // ModelComponent interface.
    /** Concrete probes may override; be sure to invoke Super::connectToModel()
    at the beginning of the overriding method. **/
    void connectToModel(Model& model) OVERRIDE_11;
    /** Concrete probes may override; be sure to invoke Super::addToSystem()
    at the beginning of the overriding method. **/
    void addToSystem(SimTK::MultibodySystem& system) const OVERRIDE_11;


private:
    void setNull();
    void constructProperties();

//=============================================================================
// DATA
//=============================================================================
    SimTK::Measure afterOperationValue;
//=============================================================================
};	// END of class Probe
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // OPENSIM_PROBE_H_


