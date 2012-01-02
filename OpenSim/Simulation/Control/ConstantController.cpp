// ConstantController.cpp
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
#include "ConstantController.h"
#include "ControlLinear.h"
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
ConstantController::~ConstantController()
{
}
//_____________________________________________________________________________
/**
 * Default constructor
 */

ConstantController::ConstantController() :
    Controller(),
    _controlConstant(_controlConstantProp.getValueDbl() ) {

	setNull();
    setupProperties();
}
//_____________________________________________________________________________
/**
 * Constructor from an XML Document
 */
ConstantController::ConstantController(const std::string &aFileName, bool aUpdateFromXMLNode) :
	Controller(aFileName, false),
    _controlConstant(_controlConstantProp.getValueDbl() )
{
	setNull();
    setupProperties();
	if(aUpdateFromXMLNode) updateFromXMLNode(_document->getRootDataElement(), _document->getDocumentVersion());
}
//_____________________________________________________________________________
/**
 * Copy constructor.
 */
ConstantController::ConstantController(const ConstantController &aController) :
	Controller(aController),
    _controlConstant(_controlConstantProp.getValueDbl() )
{
	setNull();
    setupProperties();
	copyData(aController);
}
//_____________________________________________________________________________
/**
 * Copy this ConstantController and return a pointer to the copy.
 * The copy constructor for this class is used.  This method is called
 * when a description of this controller is read in from an XML file.
 *
 * @return Pointer to a copy of this ConstantController.
 */
Object* ConstantController::copy() const
{
      ConstantController *object = new ConstantController(*this);
      return object;
}



//_____________________________________________________________________________
/**
 * Set NULL values for all member variables.
 */
void ConstantController::
setNull()
{
	setType("ConstantController");
    _controlConstant = 0.0;

	// MODEL
	_model = NULL;


}
//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void ConstantController::
setupProperties()
{
    std::string comment = "The control value the controller always sets for the actuator.";
    _controlConstantProp.setComment(comment);
    _controlConstantProp.setName("control_constant");
    _propertySet.append( &_controlConstantProp );



}
//_____________________________________________________________________________
/**
 * Copy the member variables of the specified controller.
 */
void ConstantController::
copyData(const ConstantController &aController)
{
    // Copy parent class's members first.
    Controller::copyData(aController);
    
    _controlConstant = aController._controlConstant;
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
ConstantController& ConstantController::
operator=(const ConstantController &aController)
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
void ConstantController::
setActuators( Set<Actuator>& as ) {

    int i,j;

    for(i=0,j=0; i<as.getSize(); i++ ) {
        Actuator& act = as.get(i);
     }
     _numControls = j;
}

//=============================================================================
// CONTROL
//=============================================================================

// compute the control value for an actuator
void ConstantController::computeControls(const SimTK::State& s, SimTK::Vector& controls) const
{
   	for(int i=0; i<_actuatorSet.getSize(); i++){
		SimTK::Vector actControls(1, _controlConstant);
		_actuatorSet[i].addInControls(actControls, controls);
	}
}



// for any post XML desereialization intialization
void ConstantController::setup(Model& model) 
{
     setActuators( _model->updActuators() );
}
// for adding any components to the model
void ConstantController::createSystem( SimTK::MultibodySystem& system ) const
{
}

// for any intialization requiring a state or the complete system 
void ConstantController::initState( SimTK::State& s)  const 
{
}
