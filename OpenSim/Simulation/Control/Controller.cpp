/* -------------------------------------------------------------------------- *
 *                          OpenSim:  Controller.cpp                          *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Ajay Seth, Frank C. Anderson, Chand T. John, Samuel R. Hamner   *
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
// INCLUDES
//=============================================================================
#include "Controller.h"
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/ModelComponent.h>
#include <OpenSim/Simulation/Model/Actuator.h>
#include <OpenSim/Common/IO.h>

//=============================================================================
// STATICS
//=============================================================================

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
	setAuthors("Ajay Seth, Frank Anderson, Chand John, Samuel Hamner");
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
    if( getModel().getAllControllersEnabled() ) {
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
void Controller::connectToModel(Model& model)
{
	Super::connectToModel(model);

	if (getProperty_actuator_list().size() > 0){
		if (IO::Uppercase(get_actuator_list(0)) == "ALL"){
			setActuators(model.getActuators());
			// setup actuators to ensure actuators added by controllers are also setup properly
			// TODO: Adopt the controls (discrete state variables) of the Actuator
			return;
		}
		else{
			Set<Actuator> actuatorsByName;
			for (int i = 0; i < getProperty_actuator_list().size(); i++){
				if (model.updActuators().contains(get_actuator_list(i)))
					actuatorsByName.adoptAndAppend(&model.updActuators().get(get_actuator_list(i)));
				else
					cerr << "WARN: Controller::connectToModel : Actuator " << get_actuator_list(i) << " was not found and will be ignored." << endl;
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
void Controller::setActuators(const Set<Actuator>& actuators)
{
	//Rebuild consistent set of actuator lists
	_actuatorSet.setSize(0);
	updProperty_actuator_list().clear();
	const Set<Actuator>& fSet = _model->getActuators();
	for (int i = 0, iact = 0; i<fSet.getSize(); i++) {
		ScalarActuator* act = dynamic_cast<ScalarActuator*>(&fSet[i]);
	//for(int i=0; i< actuators.getSize(); i++){
		addActuator(act[i]);
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
	if (found < 0) //add if the actuator isn't already in the list
		updProperty_actuator_list().appendValue(actuator.getName());
}

Set<Actuator>& Controller::updActuators() { return _actuatorSet; }

const Set<Actuator>& Controller::getActuatorSet() const { return _actuatorSet; }
