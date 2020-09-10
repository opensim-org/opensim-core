/* -------------------------------------------------------------------------- *
*                      OpenSim:  ActuatorPowerProbe.cpp                      *
* -------------------------------------------------------------------------- *
* The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
* See http://opensim.stanford.edu and the NOTICE file for more information.  *
* OpenSim is developed at Stanford University and supported by the US        *
* National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
* through the Warrior Web program.                                           *
*                                                                            *
* Copyright (c) 2005-2017 Stanford University and the Authors                *
* Author(s): Frank C. Anderson, Ajay Seth, Tim Dorn                          *
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

//=============================================================================
// INCLUDES and STATICS
//=============================================================================
#include "ActuatorPowerProbe.h"
#include "Model.h"
#include <OpenSim/Common/IO.h>

using namespace std;
using namespace SimTK;
using namespace OpenSim;


//=============================================================================
// CONSTRUCTOR(S) AND SETUP
//=============================================================================
//_____________________________________________________________________________
// Default constructor.
ActuatorPowerProbe::ActuatorPowerProbe()
{
    setNull();
    constructProperties();
}

//_____________________________________________________________________________
// Convenience constructor.
ActuatorPowerProbe::ActuatorPowerProbe(const Array<string> actuator_names,
    const bool sum_powers_together, const double exponent)
{
    setNull();
    constructProperties();

    set_actuator_names(actuator_names);
    set_sum_powers_together(sum_powers_together);
    set_exponent(exponent);
}


//_____________________________________________________________________________
// Set the data members of this ActuatorPowerProbe to their null values.
void ActuatorPowerProbe::setNull(void)
{
    setAuthors("Tim Dorn");
    _actuatorIndex.clear();
}

//_____________________________________________________________________________
// Connect properties to local pointers.
void ActuatorPowerProbe::constructProperties(void)
{
    constructProperty_actuator_names();
    constructProperty_sum_powers_together(false);
    constructProperty_exponent(1.0);
}

//=============================================================================
// GET AND SET
//=============================================================================
//_____________________________________________________________________________
/**
* Returns the names of the Actuators being probed.
*/
const Property<string>& ActuatorPowerProbe::getActuatorNames() const
{
    return getProperty_actuator_names();
}

//_____________________________________________________________________________
/**
* Returns whether to report sum of all actuator powers together
or report the actuator powers individually.
*/
bool ActuatorPowerProbe::getSumPowersTogether() const
{
    return get_sum_powers_together();
}

//_____________________________________________________________________________
/**
* Returns the exponent to apply to each actuator power.
*/
double ActuatorPowerProbe::getExponent() const
{
    return get_exponent();
}

//_____________________________________________________________________________
/**
* Sets the names of the Actuators being probed.
*/
void ActuatorPowerProbe::setActuatorNames(const Array<string>& actuatorNames)
{
    set_actuator_names(actuatorNames);
}

//_____________________________________________________________________________
/**
* Sets whether to report sum of all actuator powers together
or report the actuator powers individually.
*/
void ActuatorPowerProbe::setSumPowersTogether(const bool sum_powers_together)
{
    set_sum_powers_together(sum_powers_together);
}

//_____________________________________________________________________________
/**
* Sets the exponent to apply to each actuator power.
*/
void ActuatorPowerProbe::setExponent(const double exponent)
{
    set_exponent(exponent);
}


//=============================================================================
// MODEL COMPONENT METHODS
//=============================================================================
//_____________________________________________________________________________
/**
* Perform some set up functions that happen after the
* object has been deserialized or copied.
*
* @param aModel OpenSim model containing this ActuatorPowerProbe.
*/
void ActuatorPowerProbe::extendConnectToModel(Model& model)
{
    Super::extendConnectToModel(model);

    // Check to see if 'all' actuators are selected for probing.
    if (getProperty_actuator_names().size() > 0)
    {
        if (IO::Uppercase(get_actuator_names(0)) == "ALL")
        {
            Array<string> allActNames;
            _model->getActuators().getNames(allActNames);
            set_actuator_names(allActNames);
            //cout << "Set to all actuators: " << allActNames << endl;
        }
    }

    // check that each Actuator in the actuator_names array exists in the model.
    _actuatorIndex.clear();
    const int nA = getActuatorNames().size();
    for (int i = 0; i<nA; i++) {
        const string& actName = getActuatorNames()[i];
        const int k = model.getActuators().getIndex(actName);
        if (k<0) {
            string errorMessage = getConcreteClassName() + ": Invalid Actuator '" + actName + "' specified in <actuator_names>.";
            log_warn("{} Probe will be disabled.", errorMessage);
            setEnabled(false);
            //throw (Exception(errorMessage.c_str()));
        }
        else
            _actuatorIndex.push_back(k);
    }

    // Sanity check. Should never actually happen!
    if (nA != int(_actuatorIndex.size()))
        throw (Exception("Size of _actuatorIndex does not match number of Actuators listed in <actuator_names>."));
}





//=============================================================================
// COMPUTATION
//=============================================================================
//_____________________________________________________________________________
/**
* Compute the Actuator power.
*/
SimTK::Vector ActuatorPowerProbe::computeProbeInputs(const State& s) const
{
    int nA = getActuatorNames().size();
    SimTK::Vector TotalP(getNumProbeInputs());
    TotalP = 0;

    // Loop through each actuator in the list of actuator_names.
    for (int i = 0; i<nA; ++i)
    {
        // Get the "Actuator" power from the Actuator object.
        const double actPower = _model->getActuators()[_actuatorIndex[i]].getPower(s);

        // Append to output vector.
        if (getSumPowersTogether())
            TotalP(0) += std::pow(actPower, getExponent());
        else
            TotalP(i) = std::pow(actPower, getExponent());
    }

    return TotalP;
}


//_____________________________________________________________________________
/**
* Returns the number of probe inputs in the vector returned by computeProbeInputs().
*/
int ActuatorPowerProbe::getNumProbeInputs() const
{
    if (getSumPowersTogether())
        return 1;
    else
        return getActuatorNames().size();
}


//_____________________________________________________________________________
/**
* Provide labels for the probe values being reported.
*/
Array<string> ActuatorPowerProbe::getProbeOutputLabels() const
{
    Array<string> labels;

    // Report sum of actuator powers
    if (getSumPowersTogether())
        labels.append(getName() + "_Summed");

    // Report actuator powers individually
    else {
        for (int i = 0; i<getActuatorNames().size(); ++i)
            labels.append(getName() + "_" + getActuatorNames()[i]);
    }

    return labels;
}
