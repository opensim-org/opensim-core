// Probe.cpp
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

#include "Probe.h"

using namespace std;
using namespace SimTK;


//This Measure returns a probe value value only at the Acceleration stage
template <class T>
class ProbeMeasure : public SimTK::Measure_<T> {
public:
    SimTK_MEASURE_HANDLE_PREAMBLE(ProbeMeasure, Measure_<T>);
 
    ProbeMeasure(Subsystem& sub, const OpenSim::Probe& probe)
    :   SimTK::Measure_<T>(sub, new Implementation(probe), AbstractMeasure::SetHandle()) {}
    SimTK_MEASURE_HANDLE_POSTSCRIPT(ProbeMeasure, Measure_<T>);
};
 
 
template <class T>
class ProbeMeasure<T>::Implementation : public SimTK::Measure_<T>::Implementation {
public:
    Implementation(const OpenSim::Probe& probe)
    :   SimTK::Measure_<T>::Implementation(1), m_probe(probe) {}
 
    // Default copy constructor, destructor, copy assignment are fine.
 
    // Implementations of virtual methods.
    Implementation* cloneVirtual() const {return new Implementation(*this);}
    int getNumTimeDerivativesVirtual() const {return 0;}
    Stage getDependsOnStageVirtual(int order) const
    {   return Stage::Acceleration; }
 
    void calcCachedValueVirtual(const State& s, int derivOrder, T& value) const
    {
        SimTK_ASSERT1_ALWAYS(derivOrder==0,
            "ProbeMeasure::Implementation::calcCachedValueVirtual():"
            " derivOrder %d seen but only 0 allowed.", derivOrder);
 
        value = m_probe.computeProbeInputs(s);
    }


private:
    const OpenSim::Probe& m_probe;
};


template <>
void ProbeMeasure<double>::Implementation::calcCachedValueVirtual(const State& s, int derivOrder, double& value) const
{
    SimTK_ASSERT1_ALWAYS(derivOrder==0,
        "ProbeMeasure::Implementation::calcCachedValueVirtual():"
        " derivOrder %d seen but only 0 allowed.", derivOrder);
 
    value = m_probe.computeProbeInputs(s)(0);
}


namespace OpenSim {

//=============================================================================
// CONSTRUCTOR
//=============================================================================
// Uses default (compiler-generated) destructor, copy constructor, copy 
// assignment operator.

//_____________________________________________________________________________
// Default constructor.
Probe::Probe()
{
    setNull();
    constructProperties();
}

//_____________________________________________________________________________
// Set the data members of this Probe to their null values.
void Probe::setNull(void)
{
    // no data members that need initializing
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void Probe::constructProperties(void)
{
    constructProperty_isDisabled(false);
    constructProperty_probe_operation("value");     // means "pass the value through".
    Vector defaultInitCond(1, 0.0);         // Set the default initial condition to be a scalar equal to zero.
    constructProperty_initial_conditions_for_integration(defaultInitCond);
    constructProperty_scale_factor(1.0);
}

//_____________________________________________________________________________
/**
 * Create an underlying OpenSim::Probe
 */
void Probe::connectToModel(Model& model)
{
    Super::connectToModel(model);
}

//_____________________________________________________________________________
/**
 * Create the underlying system component(s).
 */
void Probe::addToSystem(MultibodySystem& system) const
{
    Super::addToSystem(system);

    // Make writable briefly so we can finalize the Probe to include 
    // references to the allocated System resources.
    Probe* mutableThis = const_cast<Probe*>(this);

    // Create a Measure of the value to be probed (operand).
    // NOTE: NEED TO HANDLE THIS FOR INTEGRATION AS A SPECIAL CASE FOR NOW...
    // In the case of using the 'Integrate' operation, we need to create
    // an array of scalar ProbeMeasures (this will be fixed by Sherm soon
    // because currently, the Integrate SimTK::Measure can only reliably
    // operate on scalar values).
    ProbeMeasure<SimTK::Vector> beforeOperationValue(system, *this); 
    SimTK::Array_<ProbeMeasure<double> > beforeOperationValueIntegrate;

    if (getOperation() == "integrate") { 
        for (int i=0; i<getInitialConditions().size(); ++i) {
            ProbeMeasure<double> tmpPM(system, *this); 
            beforeOperationValueIntegrate.push_back(tmpPM);
        }
    }
    //Measure::Constant beforeOperationValue(system, 1);		// debug



    // Assign the correct (operation) Measure subclass to the operand
    // ----------------------------------------------------------------

    // Return the original probe value (no operation)
    if (getOperation() == "value")
        mutableThis->afterOperationValue = Measure_<SimTK::Vector>::Scale(system, getScaleFactor(), beforeOperationValue);

    // Integrate the probe value
    // -----------------------------
    else if (getOperation() == "integrate") {
        // check to see that size of initial condition vector
        // is the same size as the data being integrated.
        if (getInitialConditions().size() != getProbeLabels().getSize())  {
            char numIC[5];
            sprintf(numIC, "%d", getInitialConditions().size());
            char numData[5];
            sprintf(numData, "%d", getProbeLabels().getSize());

            string errorMessage = getConcreteClassName() + "(" + getName() + 
                "): Mismatch between the size of the data labels corresponding to the size of the data vector being integrated ("
                +numData+") and size of initial conditions vector ("+numIC+").";
            throw (Exception(errorMessage.c_str()));
        }
        for (int i=0; i<getInitialConditions().size(); ++i) {
            Measure::Constant initCond(system, getInitialConditions()(i));		// initial conditions
            mutableThis->afterOperationValueIntegrate = Measure::Scale(system, getScaleFactor(), 
                Measure::Integrate(system, beforeOperationValueIntegrate[i], initCond));
        }
    }


    // Differentiate the probe value
    // -----------------------------
    else if (getOperation() == "differentiate")
        mutableThis->afterOperationValue = Measure_<SimTK::Vector>::Scale(system, getScaleFactor(), 
        Measure_<SimTK::Vector>::Differentiate(system, beforeOperationValue));


    // Get the minimum of the probe value (Sherm to implement)
    // ----------------------------------
    //else if (getOperation() == "minimum")
    //	mutableThis->afterOperationValue = Measure_<SimTK::Vector>::Scale(system, getScaleFactor(), 
    //Measure_<SimTK::Vector>::Minimum(system, beforeOperationValue));


    // Get the maximum of the probe value (Sherm to implement)
    // ----------------------------------
    //else if (getOperation() == "maximum")
    //	mutableThis->afterOperationValue = Measure_<SimTK::Vector>::Scale(system, getScaleFactor(), 
    //Measure_<SimTK::Vector>::Maximum(system, beforeOperationValue));


    // Throw exception (invalid operation)
    // -------------------------------------
    else {
        string errorMessage = getConcreteClassName() + ": Invalid probe operation: " + getOperation() + ". Currently supports 'value', 'integrate', 'differentiate'.";
        throw (Exception(errorMessage.c_str()));
    }

}

//=============================================================================
// GET AND SET
//=============================================================================
//_____________________________________________________________________________
/**
 * Gets whether the Probe is disabled or not.
 *
 * @return true if the Probe is disabled, false if enabled.
 */
bool Probe::isDisabled() const
{
    return get_isDisabled();
}

//_____________________________________________________________________________
/**
 * Gets the operation being performed on the Probe value.
 *
 * @return operation string
 */
string Probe::getOperation() const
{
    return get_probe_operation();
}

//_____________________________________________________________________________
/**
 * Gets the initial_conditions_for_integration.
 *
 * @return initial_conditions_for_integration SimTK::Vector
 */
Vector Probe::getInitialConditions() const
{
    return get_initial_conditions_for_integration();
}

//_____________________________________________________________________________
/**
 * Gets the scaling_factor.
 *
 * @return scaling_factor double
 */
double Probe::getScaleFactor() const
{
    return get_scale_factor();
}

//_____________________________________________________________________________
/**
 * Sets whether the Probe is disabled or not.
 *
 */
void Probe::setDisabled(bool isDisabled) 
{
    set_isDisabled(isDisabled);
}

//_____________________________________________________________________________
/**
 * Sets the operation being performed on the Probe value.
 *
 */
void Probe::setOperation(string probe_operation) 
{
    set_probe_operation(probe_operation);
}


//_____________________________________________________________________________
/**
 * Sets the initial_conditions_for_integration.
 *
 */
void Probe::setInitialConditions(Vector initial_conditions_for_integration) 
{
    set_initial_conditions_for_integration(initial_conditions_for_integration);
}

//_____________________________________________________________________________
/**
 * Sets the scaling_factor.
 *
 */
void Probe::setScaleFactor(double scaling_factor) 
{
    set_scale_factor(scaling_factor);
}


//=============================================================================
// REPORTING
//=============================================================================
//_____________________________________________________________________________
/**
 * Provide the probe values to be reported that correspond to the probe labels.
 */
SimTK::Vector Probe::getProbeOutputs(const State& s) const 
{
    // NOTE: NEED TO HANDLE THIS FOR INTEGRATION AS A SPECIAL CASE FOR NOW...
    // In the case of using the 'Integrate' operation, we need to merge
    // the scalar outputs back into a SimTK::Vector (this will be fixed by Sherm soon
    // because currently, the Integrate SimTK::Measure can only reliably
    // operate on scalar values).
    if (getOperation() == "integrate") {
        SimTK::Vector output(getInitialConditions().size());
        for (int i=0; i<getInitialConditions().size(); ++i) {
            output(i) = afterOperationValueIntegrate.getValue(s);
        }
        return output;
    }
    else
        return afterOperationValue.getValue(s);         // need to return a Vector of the same size as the input (computeProbeInputs(s))

}


} // end of namespace OpenSim
