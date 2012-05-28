// JointPowerProbe.cpp
// Authors: Frank C. Anderson, Ajay Seth, Tim Dorn
/*
 * Copyright (c)  2006-12, Stanford University. All rights reserved. 
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
// INCLUDE
//=============================================================================
#include "JointPowerProbe.h"

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
JointPowerProbe::JointPowerProbe() 
{
    setNull();
    constructProperties();
}

//_____________________________________________________________________________
// Convenience constructor.
JointPowerProbe::JointPowerProbe(const Array<string>& joint_names)
{
    setNull();
    constructProperties();
    set_joint_names(joint_names);
}

//_____________________________________________________________________________
// Set the data members of this JointPowerProbe to their null values.
void JointPowerProbe::setNull()
{
    // no data members
}

//_____________________________________________________________________________
// Allocate and initialize properties.
void JointPowerProbe::constructProperties()
{
    constructProperty_joint_names();
}


//=============================================================================
// GET AND SET
//=============================================================================
/**
 * Returns the names of the Joints being probed.
 */
const Property<string>& JointPowerProbe::getJointNames() const
{
    return getProperty_joint_names();
}

//_____________________________________________________________________________
/**
 * Sets the names of the Joints being probed.
 */
void JointPowerProbe::setJointNames(const Array<string>& aJointNames)
{
    set_joint_names(aJointNames);
}




//=============================================================================
// MODEL COMPONENT METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Perform some set up functions that happen after the
 * object has been deserialized or copied.
 *
 * @param aModel OpenSim model containing this JointPowerProbe.
 */
void JointPowerProbe::setup(Model& aModel)
{
    Super::setup(aModel);

    // check that each Joints in the joint_names array exists in the model
    int nA = getJointNames().size();
    for (int i=0; i<nA; i++) {
        string jointName = getJointNames()[i];
        int k = _model->getJointSet().getIndex(jointName);
        if (k<0) {
            string errorMessage = getConcreteClassName() + ": Invalid Joint '" 
                    + jointName + "' specified in <joint_names>.";
            throw OpenSim::Exception(errorMessage);
        }
    }
}





//=============================================================================
// COMPUTATION
//=============================================================================
//_____________________________________________________________________________
// Compute the Joint power upon which the Probe operation will be based on.
double JointPowerProbe::computeProbeValue(const State& s) const
{
    int nA = getJointNames().size();
    double TotalP = 0;				// Initialize at zero

    // Loop through each joint in the list of joint_names
    for (int i=0; i<nA; i++)
    {
        string jointName = getJointNames()[i];
        int k = _model->getJointSet().getIndex(jointName);

        // Get the "Joint" power from the Joint object
        double jointPower = _model->getJointSet().get(k).calcPower(s);
        
        // Append to total "Joint" power
        TotalP += jointPower;
    }

    return(TotalP);
}

