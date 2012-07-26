// ActuatorForceProbe.cpp
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


//=============================================================================
// INCLUDE
//=============================================================================
#include "ActuatorForceProbe.h"
#include "ForceSet.h"


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
    // no data members
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
const bool ActuatorForceProbe::getSumForcesTogether() const
{
    return get_sum_forces_together();
}

//_____________________________________________________________________________
/**
 * Returns the exponent to apply to each Actuator force.
 */
const double ActuatorForceProbe::getExponent() const
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
void ActuatorForceProbe::connectToModel(Model& model)
{
    Super::connectToModel(model);

    // check that each Actuator in the actuator_names array exists in the model
    int nF = getActuatorNames().size();
    for (int i=0; i<nF; i++) {
        string forceName = getActuatorNames()[i];
        int k = model.getForceSet().getIndex(forceName);
        if (k<0) {
            string errorMessage = getConcreteClassName() + ": Invalid Actuator '" 
                                  + forceName + "' specified in <actuator_names>.";
            throw OpenSim::Exception(errorMessage);
        }
    }
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
    int nF = getActuatorNames().size();
    SimTK::Vector TotalF;

    if (getSumForcesTogether()) {
        TotalF.resize(1);
        TotalF(0) = 0;       // Initialize to zero
    }
    else
        TotalF.resize(nF);

    // Loop through each actuator in the list of actuator_names
    for (int i=0; i<nF; ++i)
    {
        // Get the Actuator force
        int k = _model->getForceSet().getIndex(getActuatorNames()[i]);
        double Ftmp = _model->getActuators().get(k).getForce(s);

        // Append to output vector
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
        labels.append(getName()+"_Summed");

    // Report actuator forces individually
    else {
        for (int i=0; i<getActuatorNames().size(); ++i)
            labels.append(getName()+"_"+getActuatorNames()[i]);
    }

    return labels;
}