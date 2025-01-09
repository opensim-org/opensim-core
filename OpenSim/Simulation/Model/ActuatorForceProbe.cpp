/* -------------------------------------------------------------------------- *
*                      OpenSim:  ActuatorForceProbe.cpp                      *
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


//=============================================================================
// INCLUDE
//=============================================================================
#include "ActuatorForceProbe.h"
#include "Model.h"
#include <OpenSim/Common/IO.h>

using namespace std;
using namespace SimTK;
using namespace OpenSim;


//=============================================================================
// CONSTRUCTOR(S) AND SETUP
//=============================================================================
// Uses default (compiler-generated) destructor, copy constructor, and copy
// assignment operator.


//_____________________________________________________________________________
/**
* Default constructor.
*/
ActuatorForceProbe::ActuatorForceProbe()
{
    setNull();
    constructProperties();
}

//_____________________________________________________________________________
/**
* Convenience constructor
*/
ActuatorForceProbe::ActuatorForceProbe(const Array<string>& actuator_names,
    const bool sum_forces_together, const double exponent)
{
    setNull();
    constructProperties();

    set_actuator_names(actuator_names);
    set_sum_forces_together(sum_forces_together);
    set_exponent(exponent);
}


//_____________________________________________________________________________
// Set the data members of this ActuatorForceProbe to their null values.
void ActuatorForceProbe::setNull()
{
    setAuthors("Tim Dorn");
    _actuatorIndex.clear();
}

//_____________________________________________________________________________
/**
* Connect properties to local pointers.
*/
void ActuatorForceProbe::constructProperties()
{
    constructProperty_actuator_names();
    constructProperty_sum_forces_together(false);
    constructProperty_exponent(1.0);
}


//=============================================================================
// GET AND SET
//=============================================================================
//_____________________________________________________________________________
/**
* Returns the name(s) of the Actuator forces being probed.
*/
const Property<string>& ActuatorForceProbe::getActuatorNames() const
{
    return getProperty_actuator_names();
}

//_____________________________________________________________________________
/**
* Returns whether to report sum of all Actuator forces together
or report the forces individually.
*/
bool ActuatorForceProbe::getSumForcesTogether() const
{
    return get_sum_forces_together();
}

//_____________________________________________________________________________
/**
* Returns the exponent to apply to each Actuator force.
*/
double ActuatorForceProbe::getExponent() const
{
    return get_exponent();
}

//_____________________________________________________________________________
/**
* Sets the name(s) of the Actuator forces being probed.
*/
void ActuatorForceProbe::setActuatorNames(const Array<string>& actuator_names)
{
    set_actuator_names(actuator_names);
}

//_____________________________________________________________________________
/**
* Sets whether to report sum of all actuator force values together
or report the force values individually.
*/
void ActuatorForceProbe::setSumForcesTogether(const bool sum_forces_together)
{
    set_sum_forces_together(sum_forces_together);
}

//_____________________________________________________________________________
/**
* Sets the exponent to apply to each actuator force.
*/
void ActuatorForceProbe::setExponent(const double exponent)
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
* @param aModel OpenSim model containing this ActuatorForceProbe.
*/
void ActuatorForceProbe::extendConnectToModel(Model& model)
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
* Compute the Force.
*/
SimTK::Vector ActuatorForceProbe::computeProbeInputs(const State& s) const
{
    int nA = getActuatorNames().size();
    SimTK::Vector TotalF(getNumProbeInputs());
    TotalF = 0;

    // Loop through each actuator in the list of actuator_names.
    for (int i = 0; i<nA; ++i)
    {
        // Get the Actuator force.
        ScalarActuator* act = dynamic_cast<ScalarActuator*>(&_model->getActuators()[_actuatorIndex[i]]);
        const double Ftmp = act->getActuation(s);

        // Append to output vector.
        if (getSumForcesTogether())
            TotalF(0) += std::pow(Ftmp, getExponent());
        else
            TotalF(i) = std::pow(Ftmp, getExponent());
    }
    return TotalF;
}


//_____________________________________________________________________________
/**
* Returns the number of probe inputs in the vector returned by computeProbeInputs().
*/
int ActuatorForceProbe::getNumProbeInputs() const
{
    if (getSumForcesTogether())
        return 1;
    else
        return getActuatorNames().size();
}


//_____________________________________________________________________________
/**
* Provide labels for the probe values being reported.
*/
Array<string> ActuatorForceProbe::getProbeOutputLabels() const
{
    Array<string> labels;

    // Report sum of actuator forces
    if (getSumForcesTogether())
        labels.append(getName() + "_Summed");

    // Report actuator forces individually
    else {
        for (int i = 0; i<getActuatorNames().size(); ++i)
            labels.append(getName() + "_" + getActuatorNames()[i]);
    }

    return labels;
}
