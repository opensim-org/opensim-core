/* -------------------------------------------------------------------------- *
 *                            OpenSim:  Probe.cpp                             *
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
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied    *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

#include "Probe.h"

using namespace std;
using namespace SimTK;


//This Measure returns a probe value value only at the Acceleration stage
template <class T>
class ProbeMeasure : public SimTK::Measure_<T> {
public:
    SimTK_MEASURE_HANDLE_PREAMBLE(ProbeMeasure, Measure_<T>);
 
    ProbeMeasure(Subsystem& sub, const OpenSim::Probe& probe, int index)
    :   SimTK::Measure_<T>(sub, new Implementation(probe, index), AbstractMeasure::SetHandle()) {}
    SimTK_MEASURE_HANDLE_POSTSCRIPT(ProbeMeasure, Measure_<T>);
};
 
 
template <class T>
class ProbeMeasure<T>::Implementation : public SimTK::Measure_<T>::Implementation {
public:
    Implementation(const OpenSim::Probe& probe, int index)
    :   SimTK::Measure_<T>::Implementation(1), m_probe(probe), i(index) {}
 
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
    int i;
};


template <>
void ProbeMeasure<double>::Implementation::calcCachedValueVirtual(const State& s, int derivOrder, double& value) const
{
    SimTK_ASSERT1_ALWAYS(derivOrder==0,
        "ProbeMeasure::Implementation::calcCachedValueVirtual():"
        " derivOrder %d seen but only 0 allowed.", derivOrder);
 
    value = m_probe.computeProbeInputs(s)(i);
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
    constructProperty_probe_operation("value");             // means "pass the value through".
    Vector defaultInitCond(1, 0.0);                         // Set the default initial condition to zero.
    constructProperty_initial_conditions_for_integration(defaultInitCond);
    constructProperty_gain(1.0);
}

//_____________________________________________________________________________
/**
 * Create an underlying OpenSim::Probe
 */
void Probe::connectToModel(Model& model)
{
    Super::connectToModel(model);

    if (getName() == "")
        setName("UnnamedProbe");
}

//_____________________________________________________________________________
/**
 * Create the underlying system component(s).
 */
void Probe::addToSystem(MultibodySystem& system) const
{
    Super::addToSystem(system);

    if (isDisabled())
        return;

    // Make writable briefly so we can finalize the Probe to include 
    // references to the allocated System resources.
    Probe* mutableThis = const_cast<Probe*>(this);

    // ---------------------------------------------------------------------
    // Create a <double> Measure of the value to be probed (operand).
    // For now, this is scalarized, i.e. a separate Measure is created
    // for each probe input element in the Vector.
    // ---------------------------------------------------------------------
    // save for when we can directly operate on Vector SimTK::Measures
    //ProbeMeasure<SimTK::Vector> beforeOperationValueVector(system, *this);

    SimTK::Array_<ProbeMeasure<double> > beforeOperationValues;
    mutableThis->afterOperationValues.resize(getNumProbeInputs());

    for (int i=0; i<getNumProbeInputs(); ++i) {
        ProbeMeasure<double> tmpPM(system, *this, i); 
        beforeOperationValues.push_back(tmpPM);
    }



    // Assign the correct (operation) Measure subclass to the operand
    // ==============================================================

    // ---------------------------------------------------------------------
    // Return the original probe value (no operation)
    // ---------------------------------------------------------------------
    if (getOperation() == "value") {
        for (int i=0; i<getNumProbeInputs(); ++i) {
            mutableThis->afterOperationValues[i] = Measure::Scale(system, getGain(), 
                beforeOperationValues[i]);   
        }
    }


    // ---------------------------------------------------------------------
    // Integrate the probe value
    // ---------------------------------------------------------------------
    else if (getOperation() == "integrate") {
        // check to see that size of initial condition vector
        // is the same size as the data being integrated.
        if (getInitialConditions().size() != getProbeOutputLabels().getSize())  {
            char numIC[5];
            sprintf(numIC, "%d", getInitialConditions().size());
            char numData[5];
            sprintf(numData, "%d", getProbeOutputLabels().getSize());

            string errorMessage = getConcreteClassName() + "(" + getName() + 
                "): Mismatch between the size of the data labels corresponding to the size of the data vector being integrated ("
                +numData+") and size of initial conditions vector ("+numIC+").";
            throw (Exception(errorMessage.c_str()));
        }
        for (int i=0; i<getNumProbeInputs(); ++i) {
            Measure::Constant initCond(system, getInitialConditions()(i));		// initial conditions
            mutableThis->afterOperationValues[i] = Measure::Scale(system, getGain(), 
                Measure::Integrate(system, beforeOperationValues[i], initCond));
        }
    }


    // ---------------------------------------------------------------------
    // Differentiate the probe value
    // ---------------------------------------------------------------------
    else if (getOperation() == "differentiate") {
        for (int i=0; i<getNumProbeInputs(); ++i) {
            mutableThis->afterOperationValues[i] = Measure::Scale(system, getGain(), 
                Measure::Differentiate(system, beforeOperationValues[i]));
        }
    }


    // ---------------------------------------------------------------------
    // Get the minimum of the probe value (Sherm to implement)
    // ---------------------------------------------------------------------
    //else if (getOperation() == "minimum") {
    //    for (int i=0; i<getNumProbeInputs(); ++i) {
    //        mutableThis->afterOperationValues[i] = Measure::Scale(system, getGain(), 
    //            Measure::Minimum(system, beforeOperationValues[i]));
    //    }
    //}
    	

    // ---------------------------------------------------------------------
    // Get the maximum of the probe value (Sherm to implement)
    // ---------------------------------------------------------------------
    //else if (getOperation() == "maximum") {
    //    for (int i=0; i<getNumProbeInputs(); ++i) {
    //        mutableThis->afterOperationValues[i] = Measure::Scale(system, getGain(), 
    //            Measure::Maximum(system, beforeOperationValues[i]));
    //    }
    //}


    // ---------------------------------------------------------------------
    // Throw exception (invalid operation)
    // ---------------------------------------------------------------------
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
double Probe::getGain() const
{
    return get_gain();
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
void Probe::setGain(double gain) 
{
    set_gain(gain);
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
    if (isDisabled()) {
        string errorMessage = getConcreteClassName() + ": Cannot get the output from Probe '" + getName() + "' because it has been disabled.";
        throw (Exception(errorMessage.c_str()));
    }


    // For now, this is scalarized, i.e. compile the result of the separate
    // Measure for each scalar element of the probe input into a SimTK::Vector
    // of outputs.
    SimTK::Vector output(getNumProbeInputs());
    for (int i=0; i<getNumProbeInputs(); ++i) {
        if (getOperation() == "integrate")
            output[i] = afterOperationValues[i].getValue(s) + getInitialConditions()(i);  // temp fix for now because init condition is always overridden to zero.
        else
            output[i] = afterOperationValues[i].getValue(s);
    }
    return output;

    //return afterOperationValueVector.getValue(s);         // save for when we can directly operate on Vector SimTK::Measures
}


} // end of namespace OpenSim
