// ActuatorPowerProbe.cpp
// Authors: Frank C. Anderson, Ajay Seth, Tim Dorn
/*
 * Copyright (c)  2006, Stanford University. All rights reserved. 
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
/**
 * Default constructor.
 */
ActuatorPowerProbe::ActuatorPowerProbe() 
{
	setNull();
}

//_____________________________________________________________________________
/**
 * Convenience constructor.
 */
ActuatorPowerProbe::ActuatorPowerProbe(Array<string> actuator_names)
{
	setNull();
	setPropertyValue("actuator_names", actuator_names);
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
ActuatorPowerProbe::~ActuatorPowerProbe()
{
}


//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aActuatorPowerProbe ActuatorPowerProbe to be copied.
 */
ActuatorPowerProbe::ActuatorPowerProbe(const ActuatorPowerProbe &aActuatorPowerProbe) :
   Probe(aActuatorPowerProbe)
{
	setNull();
	copyData(aActuatorPowerProbe);
}


//_____________________________________________________________________________
/**
 * Copy data members from one ActuatorPowerProbe to another.
 *
 * @param aProbe ActuatorPowerProbe to be copied.
 */
void ActuatorPowerProbe::copyData(const ActuatorPowerProbe &aProbe)
{
	Super::copyData(aProbe);
	setPropertyValue("actuator_names", aProbe.getProperty<string>("actuator_names"));
}

//_____________________________________________________________________________
/**
 * Set the data members of this ActuatorPowerProbe to their null values.
 */
void ActuatorPowerProbe::setNull(void)
{
	setupProperties();
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void ActuatorPowerProbe::setupProperties(void)
{
	Array<string> tmp("");
	addListProperty<string>("actuator_names",
		"Specify a list of model Actuators whose work should be calculated. "
		"If multiple Actuators are given, the probe value will be the summation"
		" of all actuator powers.",
		tmp);
}


//=============================================================================
// OPERATORS
//=============================================================================
//_____________________________________________________________________________
/**
 * Assignment operator.
 *
 * @return Reference to this object.
 */
#ifndef SWIG
ActuatorPowerProbe& ActuatorPowerProbe::operator=(const ActuatorPowerProbe &aActuatorPowerProbe)
{
	// BASE CLASS
	Super::operator=(aActuatorPowerProbe);
	return(*this);
}
#endif


//=============================================================================
// GET AND SET
//=============================================================================
/**
 * Returns the names of the Actuators being probed.
 */
const Property<string>& ActuatorPowerProbe::getActuatorNames() const
{
	return getProperty<string>("actuator_names");
}

//_____________________________________________________________________________
/**
 * Sets the names of the Actuators being probed.
 */
void ActuatorPowerProbe::setActuatorNames(const Array<string>& aActuatorNames)
{
	setPropertyValue<string>("actuator_names", aActuatorNames);
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
void ActuatorPowerProbe::setup(Model& aModel)
{
	Super::setup(aModel);

	// check that each Actuator in the actuator_names array exists in the model
	int nA = getActuatorNames().size();
	for (int i=0; i<nA; i++) {
		string actName = getActuatorNames()[i];
		int k = _model->getActuators().getIndex(actName);
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
 * Compute the Actuator power upon which the Probe operation will be based on.
 */
Vector ActuatorPowerProbe::computeProbeValue(const State& s) const
{
	int nA = getActuatorNames().size();
	Vector TotalP(1);
	TotalP(0) = 0;				// Initialize at zero

	// Loop through each actuator in the list of actuator_names
	for (int i=0; i<nA; i++)
	{
		string actName = getActuatorNames()[i];
		int k = _model->getActuators().getIndex(actName);

		// Get the "Actuator" power from the Actuator object
		double actPower = _model->getActuators().get(k).getPower(s);
		
		// Append to total "Actuator" power
		TotalP(0) += actPower;
	}

	return(TotalP);
}

