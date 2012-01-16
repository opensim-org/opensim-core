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
Controller::Controller() :
	ModelComponent(),
    _actuatorNameList(_actuatorNameListProp.getValueStrArray())
{
	setNull();
}
//_____________________________________________________________________________
/**
 * Constructor.
 */
Controller::Controller(Model& aModel) :
	ModelComponent(),
   _actuatorNameList(_actuatorNameListProp.getValueStrArray())
{
	setNull();
	_model = &aModel;
}
//_____________________________________________________________________________
/**
 * Constructor from an XML Document
  */
  Controller::Controller(const std::string &aFileName, bool aUpdateFromXMLNode) :
      ModelComponent(aFileName, false),
      _actuatorNameList(_actuatorNameListProp.getValueStrArray())
{
      setNull();
      if(aUpdateFromXMLNode) updateFromXMLNode(_document->getRootDataElement(), getDocument()->getDocumentVersion());
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 */
Controller::Controller(const Controller &aController) :
	ModelComponent(aController),
    _actuatorNameList(_actuatorNameListProp.getValueStrArray())
{
	setNull();
	copyData(aController);
}
//_____________________________________________________________________________
/**
 * Destructor.
 */
Controller::~Controller()
{

}


//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set NULL values for all member variables.
 */
void Controller::setNull()
{
	setupProperties();
	setType("Controller");
    _actuatorSet.setMemoryOwner(false);

	// MODEL
	_model = NULL;
}
//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void Controller::setupProperties()
{
     string comment;

    comment = "A list of actuators that this controller will control."
              "The keyword ALL indicates the controller will controll all the acuators in the model";
    _actuatorNameListProp.setComment(comment);
    _actuatorNameListProp.setName("actuator_list");
    _propertySet.append(&_actuatorNameListProp);

    comment = "Flag (true or false) indicating whether or not the controller is disabled.";
    _isDisabledProp.setComment(comment);
    _isDisabledProp.setName("isDisabled");
	_isDisabledProp.setValue(false);
    _propertySet.append( &_isDisabledProp );
}
//_____________________________________________________________________________
/**
 * Copy the member variables of the specified controller.
 */
void Controller::copyData(const Controller &aController)
{
	_isDisabledProp.setValue(aController._isDisabledProp.getValueBool());
    _actuatorNameList = aController._actuatorNameList;
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
Controller& Controller::
operator=(const Controller &aController)
{
	// BASE CLASS
	ModelComponent::operator=(aController);

	// DATA
	copyData(aController);

	return(*this);
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
	   return( _isDisabledProp.getValueBool() );
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
	_isDisabledProp.setValue(aTrueFalse);
}

// for any post XML deseraialization intialization
void Controller:: setup(Model& model)
{
	ModelComponent::setup(model);

	if(_actuatorNameList.getSize() > 0){
		if(IO::Uppercase(_actuatorNameList[0]) == "ALL"){
			setActuators(model.updActuators());
			// setup actuators to ensure actuators added by controllers are also setup properly
			// TODO: Adopt the controls (discrete state variables) of the Actuator
			return;
		}
		else{
			Set<Actuator> actuatorsByName;
			for(int i =0; i < _actuatorNameList.getSize(); i++){
				if(model.updActuators().contains(_actuatorNameList[i]))
					actuatorsByName.append(&model.updActuators().get(_actuatorNameList[i]));
				else
					cerr << "WARN: Controller::setup : Actuator " << _actuatorNameList[i] << " was not found and will be ignored.." << endl;
			}
			actuatorsByName.setMemoryOwner(false);
			setActuators(actuatorsByName);
		}
	}
}

/**
 * Create a Controller in the SimTK::System
 */
void Controller::createSystem(SimTK::MultibodySystem& system) const
{
	 // Beyond the const Component get the index so we can access the SimTK::Force later
	Controller* mutableThis = const_cast<Controller *>(this);

	// Keep track of the subystem the Controller is a part of in case subclasses want to 
	// extend by adding states, etc...
	// Since controllers affect how actuators produce force, it makes sense for a controller
	// to be a part of ForceSubsystem
	mutableThis->setIndexOfSubsystemForAllocations(_model->updForceSubsystem().getMySubsystemIndex());

	ModelComponent::createSystem(system);
}

// makes a request for which actuators a controller will control
void Controller::setActuators( Set<Actuator>& actuators ) {
	//Rebuild consistent set of actuator lists
	_actuatorSet.setSize(0);
	_actuatorNameList.setSize(0);
	for(int i=0; i< actuators.getSize(); i++){
		addActuator(&actuators[i]);
	}
	_actuatorSet.setMemoryOwner(false);
}

void Controller::addActuator(Actuator *actuator)
{
	_actuatorSet.append(actuator);
	_actuatorNameList.append(actuator->getName());
}


Set<Actuator>& Controller::updActuators() { return _actuatorSet; }

const Set<Actuator>& Controller::getActuatorSet() const { return _actuatorSet; }
