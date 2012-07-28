// Controller.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c)  2010, Stanford University. All rights reserved. 
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

/* 
 * Author: Ajay Seth Frank C. Anderson, Chand T. John, Samuel R. Hamner, 
 */


//=============================================================================
// INCLUDES
//=============================================================================
#include <cstdio>
#include <OpenSim/Common/IO.h>
#include <OpenSim/Common/Object.h>
#include <OpenSim/Common/Set.h>
#include "Controller.h"
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/ModelComponent.h>
#include <OpenSim/Simulation/Model/Actuator.h>
#include "SimTKsimbody.h"


//=============================================================================
// STATICS
//=============================================================================

// This command indicates that any identifier (class, variable, method, etc.)
// defined within the OpenSim namespace can be used in this file without the
// "OpenSim::" prefix.
using namespace OpenSim;
using namespace std;


//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */
Controller::Controller() 
{
	constructProperties();
}


//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void Controller::constructProperties()
{
	constructProperty_isDisabled(false);
	constructProperty_actuator_list();

	// Set is only a reference list, not ownership
	_actuatorSet.setMemoryOwner(false);
}





//=============================================================================
// GET AND SET
//=============================================================================

//-----------------------------------------------------------------------------
// ON/OFF
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get whether or not this controller is disabled.
 */
bool Controller::isDisabled() const
{
    if( _model->getAllControllersEnabled() ) {
	   return( get_isDisabled() );
    } else {
       return( true );
    }
}
//_____________________________________________________________________________
/**
 * Turn this controller on or off.
 */
void Controller::setDisabled(bool aTrueFalse)
{
	upd_isDisabled()=aTrueFalse;
}

// for any post XML deseraialization intialization
void Controller:: connectToModel(Model& model)
{
	Super::connectToModel(model);

	if(getProperty_actuator_list().size() > 0){
		if(IO::Uppercase(get_actuator_list(0)) == "ALL"){
			setActuators(model.getActuators());
			// setup actuators to ensure actuators added by controllers are also setup properly
			// TODO: Adopt the controls (discrete state variables) of the Actuator
			return;
		}
		else{
			Set<Actuator> actuatorsByName;
			for(int i =0; i <  getProperty_actuator_list().size(); i++){
				if(model.getActuators().contains(get_actuator_list(i)))
					actuatorsByName.adoptAndAppend(&model.updActuators().get(get_actuator_list(i)));
				else
					cerr << "WARN: Controller::setup : Actuator " << get_actuator_list(i) << " was not found and will be ignored." << endl;
			}
			actuatorsByName.setMemoryOwner(false);
			setActuators(actuatorsByName);
		}
	}
}

/**
 * Create a Controller in the SimTK::System
 */
void Controller::addToSystem(SimTK::MultibodySystem& system) const
{
	Super::addToSystem(system);
}

// makes a request for which actuators a controller will control
void Controller::setActuators(const Set<Actuator>& actuators )
{
	//Rebuild consistent set of actuator lists
	_actuatorSet.setSize(0);
	updProperty_actuator_list().clear();
	for(int i=0; i< actuators.getSize(); i++){
		addActuator(actuators[i]);
	}
	// make sure controller does not take ownership
	_actuatorSet.setMemoryOwner(false);
}

void Controller::addActuator(const Actuator& actuator)
{
	// want to keep a reference not make a clone
	// but set interface does not take const pointer
	// just const ref that forces a copy
	// const_cast only to add to the private set of actuators
	Actuator* mutable_act = const_cast<Actuator *>(&actuator);
	_actuatorSet.adoptAndAppend(mutable_act);

	int found = updProperty_actuator_list().findIndex(actuator.getName());
	if(found < 0) //add if the actuator isn't already in the list
		updProperty_actuator_list().appendValue(actuator.getName());
}

Set<Actuator>& Controller::updActuators() { return _actuatorSet; }

const Set<Actuator>& Controller::getActuatorSet() const { return _actuatorSet; }
