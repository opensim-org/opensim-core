/* -------------------------------------------------------------------------- *
 *                 OpenSim:  MuscleActiveFiberPowerProbe.cpp                  *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Matthew Millard                                                 *
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
#include "MuscleActiveFiberPowerProbe.h"
#include "Model.h"


using namespace std;
using namespace SimTK;
using namespace OpenSim;


//=============================================================================
// CONSTRUCTOR(S) AND SETUP
//=============================================================================
//_____________________________________________________________________________
// Default constructor.
MuscleActiveFiberPowerProbe::MuscleActiveFiberPowerProbe() 
{
    setNull();
    constructProperties();
}

//_____________________________________________________________________________
// Convenience constructor.
MuscleActiveFiberPowerProbe::
MuscleActiveFiberPowerProbe(const Array<string> muscle_names)
{
    setNull();
    constructProperties();
    set_muscle_names(muscle_names);
}


//_____________________________________________________________________________
// Set the data members of this MuscleActiveFiberPowerProbe to their null values.
void MuscleActiveFiberPowerProbe::setNull(void)
{
    setAuthors("Matthew Millard");
}

//_____________________________________________________________________________
// Connect properties to local pointers.
void MuscleActiveFiberPowerProbe::constructProperties(void)
{
    constructProperty_muscle_names();
}

//=============================================================================
// GET AND SET
//=============================================================================
//_____________________________________________________________________________
/**
 * Returns the names of the Actuators being probed.
 */
const Property<string>& MuscleActiveFiberPowerProbe::getMuscleNames() const
{
    return getProperty_muscle_names();
}

//_____________________________________________________________________________
/**
 * Sets the names of the Actuators being probed.
 */
void MuscleActiveFiberPowerProbe::setMuscleNames(const Array<string>& muscleNames)
{
    set_muscle_names(muscleNames);
}




//=============================================================================
// MODEL COMPONENT METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Perform some set up functions that happen after the
 * object has been deserialized or copied.
 *
 * @param aModel OpenSim model containing this MuscleActiveFiberPowerProbe.
 */
void MuscleActiveFiberPowerProbe::extendConnectToModel(Model& model)
{
    Super::extendConnectToModel(model);

    // check that each Muscle in the actuator_names array exists in the model
    int nA = getMuscleNames().size();
    for (int i=0; i<nA; i++) {
        string actName = getMuscleNames()[i];
        int k = model.getMuscles().getIndex(actName);
        if (k<0) {
            string errorMessage = getConcreteClassName() + ": Invalid Muscle '" + actName + "' specified in <muscle_names>.";
            log_warn("{} Probe will be disabled.", errorMessage);
            setEnabled(false);
        }
    }
}





//=============================================================================
// COMPUTATION
//=============================================================================
//_____________________________________________________________________________
/**
Compute the muscle fiber power upon which the Probe operation will be based on.
 */
SimTK::Vector MuscleActiveFiberPowerProbe::computeProbeInputs(const State& s) const
{
    int nA = getMuscleNames().size();
    SimTK::Vector TotalP(1, 0.0);       // Initialize at zero
 
    // Loop through each muscle in the list of actuator_names
    for (int i=0; i<nA; i++)
    {
        string actName = getMuscleNames()[i];
        int k = _model->getMuscles().getIndex(actName);

        // Get the fiber power from the Actuator object
        double fiberPower = _model->getMuscles().get(k).getFiberActivePower(s);
        
        // Append to total "Actuator" power
        TotalP(0) += fiberPower;
    }

    return TotalP;
}


//_____________________________________________________________________________
/** 
 * Returns the number of probe inputs in the vector returned by computeProbeInputs().
 */
int MuscleActiveFiberPowerProbe::getNumProbeInputs() const
{
    return 1;
}


//_____________________________________________________________________________
/** 
 * Provide labels for the probe values being reported.
 */
Array<string> MuscleActiveFiberPowerProbe::getProbeOutputLabels() const 
{
    Array<string> labels;
    labels.append(getName());
    return labels;
}
