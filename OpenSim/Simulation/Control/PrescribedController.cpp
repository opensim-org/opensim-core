/* -------------------------------------------------------------------------- *
 *                     OpenSim:  PrescribedController.cpp                     *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Ajay Seth                                                       *
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

/*  
 * Author: Ajay Seth
 */


//=============================================================================
// INCLUDES
//=============================================================================
#include <cstdio>
#include <OpenSim/Common/Object.h>
#include <OpenSim/Common/Set.h>
#include "PrescribedController.h"
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
PrescribedController::PrescribedController() :
	Controller(),
	_prescribedControlFunctionsProp(PropertyObj("ControlFunctions", FunctionSet())),
    _prescribedControlFunctions((FunctionSet&)_prescribedControlFunctionsProp.getValueObj())
{
	setNull();
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 */
PrescribedController::PrescribedController(const PrescribedController &aPrescribedController) :
	Controller(aPrescribedController),
	_prescribedControlFunctionsProp(PropertyObj("ControlFunctions", FunctionSet())),
    _prescribedControlFunctions((FunctionSet&)_prescribedControlFunctionsProp.getValueObj())
{
	setNull();
	copyData(aPrescribedController);
}
//_____________________________________________________________________________
/**
 * Destructor.
 */
PrescribedController::~PrescribedController()
{

}


//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set NULL values for all member variables.
 */
void PrescribedController::setNull()
{
	setupProperties();
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void PrescribedController::setupProperties()
{
	string comment = "Functions (one per control) describing the controls for actuators"
		"specified for this controller.";
	_prescribedControlFunctionsProp.setComment(comment);
	_prescribedControlFunctionsProp.setName("ControlFunctions");
	_prescribedControlFunctions.setName("ControlFunctions");
	_prescribedControlFunctionsProp.setMatchName(true);
	_propertySet.append(&_prescribedControlFunctionsProp);
}
//_____________________________________________________________________________
/**
 * Copy the member variables of the specified controller.
 */
void PrescribedController::copyData(const PrescribedController &aController)
{
	_prescribedControlFunctions = aController._prescribedControlFunctions;
}


//=============================================================================
// OPERATORS
//=============================================================================
//-----------------------------------------------------------------------------
// ASSIGNMENT
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Assignment operator.
 */
PrescribedController& PrescribedController::operator=(const PrescribedController &aController)
{
	// BASE CLASS
	ModelComponent::operator=(aController);

	// DATA
	copyData(aController);

	return(*this);
}

// compute the control value for an actuator
void PrescribedController::computeControls(const SimTK::State& s, SimTK::Vector& controls) const
{
	SimTK::Vector actControls(1, 0.0);
	SimTK::Vector time(1, s.getTime());

	for(int i=0; i<getActuatorSet().getSize(); i++){
		actControls[0] = _prescribedControlFunctions[i].calcValue(time);
		getActuatorSet()[i].addInControls(actControls, controls);
	}  
}

//=============================================================================
// GET AND SET
//=============================================================================

void PrescribedController::prescribeControlForActuator(int index, OpenSim::Function *prescribedFunction)
{
	SimTK_ASSERT( index < getActuatorSet().getSize(), "PrescribedController::computeControl:  index > number of actuators" );
	SimTK_ASSERT( index >= 0,  "PrescribedController::computeControl:  index < 0" );
	if(index >= _prescribedControlFunctions.getSize())
		_prescribedControlFunctions.setSize(index+1);
	_prescribedControlFunctions.set(index, prescribedFunction);  
}

void PrescribedController::prescribeControlForActuator(const std::string actName, OpenSim::Function *prescribedFunction)
{
	int index = getProperty_actuator_list().findIndex(actName);
	if(index < 0 )
		throw Exception("PrescribedController does not have "+actName+" in its list of actuators to control.");
	prescribeControlForActuator(index, prescribedFunction);
}

