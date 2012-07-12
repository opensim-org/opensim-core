// MuscleActiveFiberPowerProbe.cpp
// Author: Matt Millard
/*  
 * Permission is hereby granted, free of charge, to any person obtaining a    *
 * copy of this software and associated documentation files (the "Software"), *
 * to deal in the Software without restriction, including without limitation  *
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,   *
 * and/or sell copies of the Software, and to permit persons to whom the      *
 * Software is furnished to do so, subject to the following conditions:       *
 *                                                                            *
 * The above copyright notice and this permission notice shall be included in *
 * all copies or substantial portions of the Software.                        *
 *                                                                            *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR *
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,   *
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL    *
 * THE AUTHORS, CONTRIBUTORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,    *
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR      *
 * OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE  *
 * USE OR OTHER DEALINGS IN THE SOFTWARE.                                     *
 * -------------------------------------------------------------------------- */
//=============================================================================
// INCLUDES and STATICS
//=============================================================================
#include "MuscleActiveFiberPowerProbe.h"


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
    // No data members.
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
void MuscleActiveFiberPowerProbe::connectToModel(Model& model)
{
    Super::connectToModel(model);

    // check that each Muscle in the actuator_names array exists in the model
    int nA = getMuscleNames().size();
    for (int i=0; i<nA; i++) {
        string actName = getMuscleNames()[i];
        int k = model.getMuscles().getIndex(actName);
        if (k<0) {
            string errorMessage = getConcreteClassName() + ": Invalid Muscle '" + actName + "' specified in <muscle_names>.";
            throw (Exception(errorMessage.c_str()));
        }
    }
}





//=============================================================================
// COMPUTATION
//=============================================================================
//_____________________________________________________________________________
/**
Compute the muscle fiberpower upon which the Probe operation will be based on.
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

    return(TotalP);
}


//_____________________________________________________________________________
/** 
 * Provide labels for the probe values being reported.
 */
Array<string> MuscleActiveFiberPowerProbe::getProbeLabels() const 
{
    Array<string> labels;

    if (getScaleFactor() != 1.0) {
        char n[10];
        sprintf(n, "%f", getScaleFactor());
        labels.append(getName()+"_SCALED_BY_"+n+"X");
    }
    else
        labels.append(getName()+"_"+getOperation());

    return labels;
}
