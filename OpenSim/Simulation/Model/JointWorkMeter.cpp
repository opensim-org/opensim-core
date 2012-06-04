// JointWorkMeter.cpp
// Author: Ajay Seth
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

//==============================================================================
// INCLUDES
//==============================================================================
#include "JointWorkMeter.h"
#include <OpenSim/Simulation/Model/Model.h>

//==============================================================================
// STATICS
//==============================================================================
using namespace std;
using namespace OpenSim;

static const string WORK_STATE_NAME = "work";

//==============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//==============================================================================
// Uses default (compiler-generated) destructor, copy constructor, copy 
// assignment operator.

//_____________________________________________________________________________
// Default constructor.
JointWorkMeter::JointWorkMeter() 
{
	setNull();
	constructProperties();
}

//_____________________________________________________________________________
// Convenience constructor.
JointWorkMeter::JointWorkMeter(const Joint &joint, double initialWork)
{
	setNull();
	constructProperties();

	set_joint_name(joint.getName());
	set_initial_joint_work(initialWork);
}

//==============================================================================
// CONSTRUCTION METHODS
//==============================================================================

//_____________________________________________________________________________
// Set the data members of this JointWorkMeter to their null values.
void JointWorkMeter::setNull()
{
    // no data members
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void JointWorkMeter::constructProperties()
{
	constructProperty_joint_name("Unassigned");
	constructProperty_initial_joint_work(0.0);
}

//_____________________________________________________________________________
/**
 * Perform some set up functions that happen after the
 * object has been deserialized or copied.
 *
 * @param aModel OpenSim model containing this JointWorkMeter.
 */
void JointWorkMeter::setup(Model& aModel)
{
	Super::setup(aModel);

	const string& jointName = get_joint_name();
	int k = _model->getJointSet().getIndex(jointName);
	if( k >=0 )
		_joint = &_model->getJointSet().get(k);
	else{
		string errorMessage = "JointWorkMeter: Invalid joint '" + jointName 
                                + "' specified.";
		throw OpenSim::Exception(errorMessage);
	}
}

//==============================================================================
// Create the underlying system component(s)
//==============================================================================
void JointWorkMeter::createSystem(SimTK::MultibodySystem& system) const
{
	Super::createSystem(system);

	// Assign a name to the state variable to access the work value stored in the state
	string stateName = _joint->getName()+"."+WORK_STATE_NAME;

	// Add state variables to the underlying system
	addStateVariable(stateName);
}


//==============================================================================
// The state variable derivative (power) to be integrated
//==============================================================================
SimTK::Vector JointWorkMeter::
computeStateVariableDerivatives(const SimTK::State& s) const
{
	SimTK::Vector derivs(1, _joint->calcPower(s));
	double power = derivs[0];

	return derivs;
}

 void JointWorkMeter::initState( SimTK::State& s) const
{
    Super::initState(s);

	setStateVariable(s, getStateVariableNames()[0], 
        get_initial_joint_work());
}

void JointWorkMeter::setDefaultsFromState(const SimTK::State& state)
{
    Super::setDefaultsFromState(state);

    set_initial_joint_work(getWork(state));
}


//==============================================================================
// GET AND SET
//==============================================================================
//
// Computed work is part of the state
double JointWorkMeter::getWork(const SimTK::State& state) const
{
	return getStateVariable(state, _joint->getName()+"."+WORK_STATE_NAME);
}