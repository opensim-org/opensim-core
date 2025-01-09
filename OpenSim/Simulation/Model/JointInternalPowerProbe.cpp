/* -------------------------------------------------------------------------- *
 *                   OpenSim:  JointInternalPowerProbe.cpp                    *
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
#include "JointInternalPowerProbe.h"
#include "Model.h"
#include <OpenSim/Common/IO.h>

using namespace std;
using namespace SimTK;
using namespace OpenSim;


//=============================================================================
// CONSTRUCTOR(S) AND SETUP
//=============================================================================
// Uses default (compiler-generated) destructor, copy constructor, copy 
// assignment operator.

//_____________________________________________________________________________
// Default constructor.
JointInternalPowerProbe::JointInternalPowerProbe() 
{
    setNull();
    constructProperties();
}

//_____________________________________________________________________________
// Convenience constructor.
JointInternalPowerProbe::JointInternalPowerProbe(const Array<string>& joint_names, 
    const bool sum_powers_together, const double exponent)
{
    setNull();
    constructProperties();

    set_joint_names(joint_names);
    set_sum_powers_together(sum_powers_together);
    set_exponent(exponent);
}

//_____________________________________________________________________________
// Set the data members of this JointInternalPowerProbe to their null values.
void JointInternalPowerProbe::setNull()
{
    setAuthors("Tim Dorn");
    _jointIndex.clear();
}

//_____________________________________________________________________________
// Allocate and initialize properties.
void JointInternalPowerProbe::constructProperties()
{
    constructProperty_joint_names();
    constructProperty_sum_powers_together(false);
    constructProperty_exponent(1.0);
}


//=============================================================================
// GET AND SET
//=============================================================================
/**
 * Returns the names of the Joints being probed.
 */
const Property<string>& JointInternalPowerProbe::getJointNames() const
{
    return getProperty_joint_names();
}

//_____________________________________________________________________________
/**
 * Returns whether to report sum of all joint powers together
   or report the joint powers individually.
 */
bool JointInternalPowerProbe::getSumPowersTogether() const
{
    return get_sum_powers_together();
}

//_____________________________________________________________________________
/**
 * Returns the exponent to apply to each joint power.
 */
double JointInternalPowerProbe::getExponent() const
{
    return get_exponent();
}

//_____________________________________________________________________________
/**
 * Sets the names of the Joints being probed.
 */
void JointInternalPowerProbe::setJointNames(const Array<string>& aJointNames)
{
    set_joint_names(aJointNames);
}

//_____________________________________________________________________________
/**
 * Sets whether to report sum of all joint powers together
   or report the joint powers individually.
 */
void JointInternalPowerProbe::setSumPowersTogether(const bool sum_powers_together)
{
    set_sum_powers_together(sum_powers_together);
}

//_____________________________________________________________________________
/**
 * Sets the exponent to apply to each joint power.
 */
void JointInternalPowerProbe::setExponent(const double exponent)
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
 * @param aModel OpenSim model containing this JointInternalPowerProbe.
 */
void JointInternalPowerProbe::extendConnectToModel(Model& aModel)
{
    Super::extendConnectToModel(aModel);

    // Check to see if 'all' joints are selected for probing.
    if(getProperty_joint_names().size() > 0)
    {
        if(IO::Uppercase(get_joint_names(0)) == "ALL")
        {
            Array<string> allJointNames;
            _model->getJointSet().getNames(allJointNames);
            set_joint_names(allJointNames);
        }
    }

    // check that each Joints in the joint_names array exists in the model.
    _jointIndex.clear();
    const int nJ = getJointNames().size();
    for (int i=0; i<nJ; i++) {
        const string& jointName = getJointNames()[i];
        const int k = _model->getJointSet().getIndex(jointName);
        if (k<0) {
            string errorMessage = getConcreteClassName() + ": Invalid Joint '" 
                    + jointName + "' specified in <joint_names>.";
            log_warn("{} Probe will be disabled.", errorMessage);
            setEnabled(false);
        }
        else
            _jointIndex.push_back(k);
    }

    // Sanity check. Should never actually happen!
    if (nJ != int(_jointIndex.size()))
        throw (Exception("Size of _jointIndex does not match number of Joints listed in <joint_names>."));
}





//=============================================================================
// COMPUTATION
//=============================================================================
//_____________________________________________________________________________
/**
 * Compute the Joint power.
 */
SimTK::Vector JointInternalPowerProbe::computeProbeInputs(const State& s) const
{
    int nJ = getJointNames().size();
    SimTK::Vector TotalP(getNumProbeInputs());
    TotalP = 0;

    // Loop through each joint in the list of joint_names.
    for (int i=0; i<nJ; ++i)
    {
        // Get the "Joint" power from the Joint object.
        const double jointPower = _model->getJointSet()[_jointIndex[i]].calcPower(s);
        
        // Append to output vector.
        if (getSumPowersTogether())
            TotalP(0) += std::pow(jointPower, getExponent());
        else
            TotalP(i) = std::pow(jointPower, getExponent());
    }

    return TotalP;
}


//_____________________________________________________________________________
/** 
 * Returns the number of probe inputs in the vector returned by computeProbeInputs().
 */
int JointInternalPowerProbe::getNumProbeInputs() const
{
    if (getSumPowersTogether())
        return 1;
    else
        return getJointNames().size();
}


//_____________________________________________________________________________
/** 
 * Provide labels for the probe values being reported.
 */
Array<string> JointInternalPowerProbe::getProbeOutputLabels() const 
{
    Array<string> labels;

    // Report sum of joint powers
    if (getSumPowersTogether())
        labels.append(getName()+"_Summed");

    // Report joint powers individually
    else {
        for (int i=0; i<getJointNames().size(); ++i)
            labels.append(getName()+"_"+getJointNames()[i]);
    }

    return labels;
}
