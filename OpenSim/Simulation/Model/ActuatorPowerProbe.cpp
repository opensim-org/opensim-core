// ActuatorPowerProbe.cpp
// Authors: Frank C. Anderson, Ajay Seth, Tim Dorn
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

//=============================================================================
// INCLUDES and STATICS
//=============================================================================
#include "ActuatorPowerProbe.h"


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
    // No data members.
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
const bool ActuatorPowerProbe::getSumPowersTogether() const
{
    return get_sum_powers_together();
}

//_____________________________________________________________________________
/**
 * Returns the exponent to apply to each actuator power.
 */
const double ActuatorPowerProbe::getExponent() const
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
void ActuatorPowerProbe::connectToModel(Model& model)
{
    Super::connectToModel(model);

    // check that each Actuator in the actuator_names array exists in the model
    int nA = getActuatorNames().size();
    for (int i=0; i<nA; i++) {
        string actName = getActuatorNames()[i];
        int k = model.getActuators().getIndex(actName);
        if (k<0) {
            string errorMessage = getConcreteClassName() + ": Invalid Actuator '" + actName + "' specified in <actuator_names>.";
            throw (Exception(errorMessage.c_str()));
        }
    }
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
    SimTK::Vector TotalP;

    if (getSumPowersTogether()) {
        TotalP.resize(1);
        TotalP(0) = 0;       // Initialize to zero
    }
    else
        TotalP.resize(nA);

    // Loop through each actuator in the list of actuator_names
    for (int i=0; i<nA; ++i)
    {
        string actName = getActuatorNames()[i];
        int k = _model->getActuators().getIndex(actName);

        // Get the "Actuator" power from the Actuator object
        double actPower = _model->getActuators().get(k).getPower(s);
        
        // Append to output vector
        if (getSumPowersTogether())
            TotalP(0) += std::pow(actPower, getExponent());
        else
            TotalP(i) = std::pow(actPower, getExponent());
    }

    return(TotalP);
}


//_____________________________________________________________________________
/** 
 * Provide labels for the probe values being reported.
 */
Array<string> ActuatorPowerProbe::getProbeLabels() const 
{
    Array<string> labels;

    // Report sum of actuator powers
    if (getSumPowersTogether()) {
        if (getScaleFactor() != 1.0) {
            char n[10];
            sprintf(n, "%f", getScaleFactor());
            labels.append(getName()+"_Summed_SCALED_BY_"+n+"X");
        }
        else
            labels.append(getName()+"_Summed_"+getOperation());
    }

    // Report actuator powers individually
    else {
        for (int i=0; i<getActuatorNames().size(); ++i) {
            if (getScaleFactor() != 1.0) {
            char n[10];
            sprintf(n, "%f", getScaleFactor());
            labels.append(getName()+"_"+getActuatorNames()[i]+"_SCALED_BY_"+n+"X");
        }
        else
            labels.append(getName()+"_"+getActuatorNames()[i]+"_"+getOperation());
        }
    }


    return labels;
}

