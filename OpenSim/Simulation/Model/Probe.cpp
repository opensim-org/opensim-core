/* -------------------------------------------------------------------------- *
 *                            OpenSim:  Probe.cpp                             *
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

#include "Probe.h"
#include "OpenSim/Common/XMLDocument.h"

using namespace std;
using namespace SimTK;


//This Measure returns a probe value only at the Acceleration stage
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
    Implementation* cloneVirtual() const override {
        return new Implementation(*this);
    }

    int getNumTimeDerivativesVirtual() const override {
        return 0;
    }

    Stage getDependsOnStageVirtual(int order) const override {   
        return Stage::Acceleration; 
    }
 
    void calcCachedValueVirtual(const State& s, int derivOrder, T& value) const
        override
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
/**
 * Default constructor.
 */
Probe::Probe()
{
    setNull();
    constructProperties();
}

//_____________________________________________________________________________
/**
 * Set the data members of this Probe to their null values.
 */
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
    constructProperty_enabled(true);
    constructProperty_probe_operation("value");  // means "pass the value through".
    constructProperty_initial_conditions_for_integration();
    constructProperty_gain(1.0);
}

//_____________________________________________________________________________
/**
 * Create an underlying OpenSim::Probe
 */
void Probe::extendConnectToModel(Model& model)
{
    Super::extendConnectToModel(model);
}

//_____________________________________________________________________________
/**
 * Create the underlying system component(s).
 */
void Probe::extendAddToSystem(MultibodySystem& system) const
{
    Super::extendAddToSystem(system);

    if (!isEnabled())
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

    int npi = getNumProbeInputs();
    SimTK::Array_<ProbeMeasure<double> > beforeOperationValues;
    mutableThis->afterOperationValues.resize(npi);

    for (int i=0; i<npi; ++i) {
        ProbeMeasure<double> tmpPM(system, *this, i); 
        beforeOperationValues.push_back(tmpPM);
    }

    // Assign the correct (operation) Measure subclass to the operand
    // ==============================================================

    // ---------------------------------------------------------------------
    // Return the original probe value (no operation)
    // ---------------------------------------------------------------------
    if (getOperation() == "value") {
        for (int i=0; i<npi; ++i) {
            mutableThis->afterOperationValues[i] = beforeOperationValues[i];   
        }
    }
    // ---------------------------------------------------------------------
    // Integrate the probe value
    // ---------------------------------------------------------------------
    else if (getOperation() == "integrate") {
        // check to see that size of initial condition vector
        // is the same size as the data being integrated.
        if (getInitialConditions().size() != getProbeOutputLabels().getSize())  {
            stringstream errorMessage;
            errorMessage << getConcreteClassName() << "(" + getName() << 
                "): Mismatch between the size of the data labels corresponding to the "
                "size of the data vector being integrated (" 
                <<  getProbeOutputLabels().getSize() 
                << ") and size of initial conditions vector (" 
                << getInitialConditions().size() << ").\nAssuming an initial condition vector of " 
                << getProbeOutputLabels().getSize() << " zeros." << endl;
            log_error(errorMessage.str());

            SimTK::Vector newInitCond(getProbeOutputLabels().getSize());
            newInitCond = 0;
            Probe* mutableThis = const_cast<Probe*>(this);
            mutableThis->setInitialConditions(newInitCond);

            //throw (Exception(errorMessage.str()));
        }

        for (int i=0; i<getNumProbeInputs(); ++i) {
            //Measure::Constant initCond(system, getInitialConditions()(i));  // init cond is handled as a special case in getProbeOutputs()
            Measure::Constant initCond(system, 0.0);
            mutableThis->afterOperationValues[i] = Measure::Integrate(
                system, beforeOperationValues[i], initCond);
        }
    }


    // ---------------------------------------------------------------------
    // Differentiate the probe value
    // ---------------------------------------------------------------------
    else if (getOperation() == "differentiate") {
        for (int i=0; i<getNumProbeInputs(); ++i) {
            mutableThis->afterOperationValues[i] = Measure::Differentiate(
                system, beforeOperationValues[i]);
        }
    }
    // ---------------------------------------------------------------------
    // Get the minimum of the probe value
    // ---------------------------------------------------------------------
    else if (getOperation() == "minimum") {
        for (int i=0; i<getNumProbeInputs(); ++i) {
            mutableThis->afterOperationValues[i] = Measure::Minimum(
                system, beforeOperationValues[i]);
        }
    }
    // ---------------------------------------------------------------------
    // Get the absolute minimum of the probe value
    // ---------------------------------------------------------------------
    else if (getOperation() == "minabs") {
        for (int i=0; i<getNumProbeInputs(); ++i) {
            mutableThis->afterOperationValues[i] = Measure::MinAbs(
                system, beforeOperationValues[i]);
        }
    }
    // ---------------------------------------------------------------------
    // Get the maximum of the probe value
    // ---------------------------------------------------------------------
    else if (getOperation() == "maximum") {
        for (int i=0; i<getNumProbeInputs(); ++i) {
            mutableThis->afterOperationValues[i] = Measure::Maximum(
                system, beforeOperationValues[i]);
        }
    }

    // ---------------------------------------------------------------------
    // Get the absolute maximum of the probe value
    // ---------------------------------------------------------------------
    else if (getOperation() == "maxabs") {
        for (int i=0; i<getNumProbeInputs(); ++i) {
            mutableThis->afterOperationValues[i] = Measure::MaxAbs(
                system, beforeOperationValues[i]);
        }
    }

    // ---------------------------------------------------------------------
    // Throw exception (invalid operation)
    // ---------------------------------------------------------------------
    else {
        stringstream errorMessage;
        errorMessage << getConcreteClassName() << ": Invalid probe operation: " 
            << getOperation() 
            << ". Currently supports 'value', 'integrate', 'differentiate', "
            "'minimum', 'minabs', 'maximum', 'maxabs'." << endl;
        throw (Exception(errorMessage.str()));
    }
}


//_____________________________________________________________________________
/**
 * Reset (initialize) the underlying Probe SimTK::Measure.
 * Only can do this for integrate, minimum, minabs, maximum, 
 * maxabs SimTK::Measures. Else, do no resetting.
 */
void Probe::reset(SimTK::State& s)
{
    const double resetValue = 0.0;
    
    for (int i=0; i<getNumProbeInputs(); ++i) {
        if (isEnabled()) {
            //cout << "Resetting probe " << getName() << ",  (" << i << " / " << getNumProbeInputs() << ")." << endl;
            //const Measure::Scale& scaleMeasure = Measure::Scale::getAs(afterOperationValues[i]);

            if (getOperation() == "integrate")
                SimTK::Measure::Integrate::getAs(afterOperationValues[i]).setValue(s, resetValue);

            else if (getOperation() == "minimum")
                SimTK::Measure::Minimum::getAs(afterOperationValues[i]).setValue(s, resetValue);

            else if (getOperation() == "minabs")
                SimTK::Measure::MinAbs::getAs(afterOperationValues[i]).setValue(s, resetValue);

            else if (getOperation() == "maximum")
                SimTK::Measure::Maximum::getAs(afterOperationValues[i]).setValue(s, resetValue);

            else if (getOperation() == "maxabs")
                SimTK::Measure::MaxAbs::getAs(afterOperationValues[i]).setValue(s, resetValue);
        }
    }
}



//=============================================================================
// GET AND SET
//=============================================================================
//_____________________________________________________________________________
/**
 * Gets whether the Probe is enabled.
 *
 * @return true if the Probe is enabled, false if disabled.
 */
bool Probe::isEnabled() const
{
    return get_enabled();
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
    int size = getProperty_initial_conditions_for_integration().size();
    Vector v(size);
    for(int i = 0; i < size ; i++)
    {
        v[i] = get_initial_conditions_for_integration(i);
    }
    return v;
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
 * Sets whether the Probe is enabled or not.
 *
 */
void Probe::setEnabled(bool enabled) 
{
    set_enabled(enabled);
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
    if (!isEnabled()) {
        stringstream errorMessage;
        errorMessage << getConcreteClassName() 
            << ": Cannot get the output from Probe '" 
            << getName() << "' because it has been disabled." << endl;
        throw (Exception(errorMessage.str()));
    }


    // For now, this is scalarized, i.e. compile the result of the separate
    // Measure for each scalar element of the probe input into a SimTK::Vector
    // of outputs.
    SimTK::Vector output(getNumProbeInputs());
    for (int i=0; i<getNumProbeInputs(); ++i) {
        if (getOperation() == "integrate")
            output[i] = getGain() * (afterOperationValues[i].getValue(s) + getInitialConditions()(i));
        else
            output[i] = getGain() * afterOperationValues[i].getValue(s);
    }
    
    return output;

    //return afterOperationValueVector.getValue(s);         // save for when we can directly operate on Vector SimTK::Measures
}


//_____________________________________________________________________________
/**
 * Returns the number of state variables this probe uses. It adds up
 * all states from the underlying SimTK::Measures.
 */
int Probe::getNumInternalMeasureStates() const
{
    if (!isEnabled())
        return 0;

    int n = 0;
    for (int i=0; i<getNumProbeInputs(); ++i)
        n += afterOperationValues[i].getNumTimeDerivatives();

    return n;
}

void Probe::updateFromXMLNode(SimTK::Xml::Element& node,
                              int versionNumber) {
    if(versionNumber < XMLDocument::getLatestVersion()) {
        if(versionNumber < 30511) {
            // Rename property 'isDisabled' to 'enabled' and
            // negate the contained value.
            std::string oldName{"isDisabled"};
            std::string newName{"enabled"};
            if(node.hasElement(oldName)) {
                auto elem = node.getRequiredElement(oldName);
                bool isDisabled = false;
                elem.getValue().tryConvertToBool(isDisabled);

                // now update tag name to 'enabled'
                elem.setElementTag(newName);
                // update its value to be the opposite of 'isDisabled'
                elem.setValue(SimTK::String(!isDisabled));
            }
        }
    }

    Super::updateFromXMLNode(node, versionNumber);
}

} // end of namespace OpenSim
