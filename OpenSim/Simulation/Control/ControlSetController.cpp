/* -------------------------------------------------------------------------- *
 *                     OpenSim:  ControlSetController.cpp                     *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Jack Middleton                                                  *
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
 * Author: Jack Middleton 
 */


//=============================================================================
// INCLUDES
//=============================================================================
#include "Controller.h"
#include "ControlSetController.h"
#include "ControlLinear.h"
#include "ControlSet.h"
#include <OpenSim/Simulation/Model/Model.h>


//=============================================================================
// STATICS
//=============================================================================

// This command indicates that any identifier (class, variable, method, etc.)
// defined within the OpenSim namespace can be used in this file without the
// "OpenSim::" prefix.
using namespace OpenSim;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
ControlSetController::~ControlSetController()
{
	delete _controlSet;
}
//_____________________________________________________________________________
/**
 * Default constructor
 */

ControlSetController::ControlSetController() :
    Controller(),
    _controlsFileName(_controlsFileNameProp.getValueStr() ) {
	setNull();
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 */
ControlSetController::ControlSetController(const ControlSetController &aController) :
	Controller(aController),
   _controlsFileName(_controlsFileNameProp.getValueStr())
{
	setNull();
	copyData(aController);
}


//_____________________________________________________________________________
/**
 * Set NULL values for all member variables.
 */
void ControlSetController::setNull()
{
    setupProperties();

	_model = NULL;
    _controlSet = NULL;


}
//_____________________________________________________________________________
/**
 * Set name of ControlSet file 
 */
//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void ControlSetController::
setControlSetFileName( const std::string&  controlSetFileName )
{
   _controlsFileName = controlSetFileName;
}
void ControlSetController::
setupProperties()
{
    std::string comment = "A Storage (.sto) or an XML control nodes file containing the controls for this controlSet.";
    _controlsFileNameProp.setComment(comment);
    _controlsFileNameProp.setName("controls_file");
    _propertySet.append( &_controlsFileNameProp );

}
//_____________________________________________________________________________
/**
 * Copy the member variables of the specified controller.
 */
void ControlSetController::copyData(const ControlSetController &aController)
{   
    _controlsFileName = aController._controlsFileName;
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
ControlSetController& ControlSetController::
operator=(const ControlSetController &aController)
{
	// BASE CLASS
	Object::operator=(aController);

	// DATA
	copyData(aController);

	return(*this);
}


//=============================================================================
// GET AND SET
//=============================================================================

//=============================================================================
// CONTROL
//=============================================================================

// compute the control value for all actuators this Controller is responsible for
void ControlSetController::computeControls(const SimTK::State& s, SimTK::Vector& controls)  const
{
	SimTK_ASSERT( _controlSet , "ControlSetController::computeControls controlSet is NULL");

	std::string actName = "";
	int index = -1;

	int na = getActuatorSet().getSize();

	for(int i=0; i< na; ++i){
		actName = getActuatorSet()[i].getName();
		index = _controlSet->getIndex(actName);
		if(index < 0){
			actName = actName + ".excitation";
			index = _controlSet->getIndex(actName);
		}

		if(index >= 0){
			SimTK::Vector actControls(1, _controlSet->get(index).getControlValue(s.getTime()));
			getActuatorSet()[i].addInControls(actControls, controls);
		}
	}
}

double ControlSetController::getFirstTime() const {
    Array<int> controlList;
   SimTK_ASSERT( _controlSet , "ControlSetController::getFirstTime controlSet is NULL");

//    std::cout << " ncontrols= "<< _controlSet->getSize() << std::endl<<std::endl;
    _controlSet->getControlList( "ControlLinear" , controlList );
    
    if( controlList.getSize() < 1 ) {
       return( -SimTK::Infinity );
    } else {
       ControlLinear& control = (ControlLinear&)_controlSet->get(controlList[0]);
       return( control.getFirstTime() );
    }
}

double ControlSetController::getLastTime() const {
    Array<int> controlList;
    _controlSet->getControlList( "ControlLinear" , controlList );
    
    if(controlList.getSize() < 1 ) {
       return( SimTK::Infinity );
    } else {
       ControlLinear& control = (ControlLinear&)_controlSet->get(controlList[0]);
       return( control.getLastTime() );
    }
}

void ControlSetController::finalizeFromProperties()
{
    SimTK_ASSERT(_controlsFileName != "",
        "ControlSetController::finalizeFromProperties controlsFileName is NULL");

    if (_controlsFileName != "Unassigned") {
        //        std::cout<<"\n\nControlSetController::connectToModel(): Loading controls from file "<<_controlsFileName<<"."<<std::endl;
        //        std::cout<<"ControlSetController::connectToModel(): Found "<<_controlSet->getSize()<<" controls."<<std::endl;
        delete  _controlSet;
        if (_controlsFileName.rfind(".sto") != std::string::npos)
            _controlSet = new ControlSet(Storage(_controlsFileName));
        else
            _controlSet = new ControlSet(_controlsFileName);
    }
    else if (_controlSet == NULL) {
        std::cout << " ControlSetController::finalizeFromProperties(): no Control Set Specified" << std::endl;
        setDisabled(true);
        return;  // no more wiring is needed
    }

    // Make sure that we are controlling all the actuators that the control set specifies
    std::string ext = ".excitation";
    for (int i = 0; _controlSet != NULL && i<_controlSet->getSize(); i++){
        std::string actName = _controlSet->get(i).getName();
        if (actName.length()>ext.length() && !(actName.compare(actName.length() - ext.length(), ext.length(), ".excitation"))){
            actName.erase(actName.length() - ext.length(), ext.length());
        }
        if (getProperty_actuator_list().findIndex(actName) < 0) // not already in the list of actuators for this controller
            updProperty_actuator_list().appendValue(actName);
    }

    Super::finalizeFromProperties();
}

// for any post XML deserialization intialization
void ControlSetController::connectToModel(Model& model)  
{
    // Controller::connectToModel() calls setActuators() with actuators in the
    // _actuatorNameList so call connectToModel() after the _controlSet 
    // constructor has been called
	Super::connectToModel(model);
}

// for adding any components to the model
void ControlSetController::addToSystem( SimTK::MultibodySystem& system ) const
{
	Super::addToSystem(system);
}

// for any intialization requiring a state or the complete system 
void ControlSetController::initStateFromProperties( SimTK::State& s)  const
{
	Super::initStateFromProperties(s);
}
