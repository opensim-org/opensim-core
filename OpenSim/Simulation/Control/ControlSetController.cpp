// ControlSetController.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c)  2009, Stanford University. All rights reserved. 
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
 * Constructor from an XML Document
 */
ControlSetController::ControlSetController(const std::string &aFileName, bool aUpdateFromXMLNode) :
	Controller(aFileName, false),
     _controlsFileName(_controlsFileNameProp.getValueStr())
{
	setNull();
	if(aUpdateFromXMLNode) updateFromXMLNode();
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
 * Copy this ControlSetController and return a pointer to the copy.
 * The copy constructor for this class is used.  This method is called
 * when a description of this controller is read in from an XML file.
 *
 * @return Pointer to a copy of this ControlSetController.
 */
Object* ControlSetController::copy() const
{
      ControlSetController *object = new ControlSetController(*this);
      return object;
}



//_____________________________________________________________________________
/**
 * Set NULL values for all member variables.
 */
void ControlSetController::
setNull()
{
    setupProperties();
	setType("ControlSetController");

	// MODEL
	_model = NULL;


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
    std::string comment = "XML file containing the controls for the controlSet.";
    _controlsFileNameProp.setComment(comment);
    _controlsFileNameProp.setName("controls_file");
    _propertySet.append( &_controlsFileNameProp );

}
//_____________________________________________________________________________
/**
 * Copy the member variables of the specified controller.
 */
void ControlSetController::
copyData(const ControlSetController &aController)
{
    // Copy parent class's members first.
    Controller::copyData(aController);
    
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

// set the pointer to the actuator's controller and the control index for 
// the actuator. First check that this the actuator is not being controlled
// by another contoller (likely a feedback controller )
void ControlSetController::setActuators( Set<Actuator>& as ) {

    SimTK_ASSERT( _controlSet , "ControlSetController::setActuators controlSet is NULL");

    for(int i=0; i<as.getSize(); i++ ) {
        Actuator& act = as.get(i);
        int index = _controlSet->getIndex(act.getName() );
		if(index == -1)
			index = _controlSet->getIndex(act.getName()+".excitation" );
        if( index != -1  ) { 
             act.setController(this);
             act.setControlIndex(index);
             act.setIsControlled(true);
        }
    }
}

//=============================================================================
// CONTROL
//=============================================================================

// compute the control value for an actuator
double ControlSetController::computeControl(const SimTK::State& s, int index )  const{
   
   SimTK_ASSERT( _controlSet , "ControlSetController::computeControl controlSet is NULL");

   SimTK_ASSERT( index < _controlSet->getSize(),
   "ControlSetController::computeControl:  index > size of actuator set" );

   SimTK_ASSERT( index >= 0,
   "ControlSetController::computeControl:  index < 0" );
//std::cout << "ControlSetController::computeControl " << _controlSet->get(index).getName() << "  t=" << s.getTime() << " control=" << _controlSet->get(index).getControlValue( s.getTime()) << std::endl; 
   return(_controlSet->get(index).getControlValue( s.getTime()) ) ;
}

double ControlSetController::
getFirstTime() const {
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

// for any post XML desereialization intialization
void ControlSetController::setup(Model& model)  
{

    SimTK_ASSERT( _controlsFileName!="" , "ControlSetController::setup controlsFileName is NULL");

    if(_controlsFileName!="Unassigned") {
//        std::cout<<"\n\nControlSetController::setup Loading controls from file "<<_controlsFileName<<"."<<std::endl;
//        std::cout<<"ControlSetController::setup Found "<<_controlSet->getSize()<<" controls."<<std::endl;
        _controlSet = new ControlSet(_controlsFileName);
    }
	else if (_controlSet == NULL) {
       std::cout << " ControlSetController:: no Control Set Specified" << std::endl;
    }

    // Controller::setup calls setActuators() which requires _controlSet to be valid
    // so call setup() after the _controlSet constructor has been called
	Controller::setup(model);

}
// for adding any components to the model
void ControlSetController::createSystem( SimTK::MultibodySystem& system ) const
{
}

// for any intialization requiring a state or the complete system 
void ControlSetController::initState( SimTK::State& s)  const
{
}
