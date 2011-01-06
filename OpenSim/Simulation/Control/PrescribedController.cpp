// PrescribedController.cpp
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
	Controller()
{
	setNull();
}

//_____________________________________________________________________________
/**
 * Constructor.
 */
PrescribedController::PrescribedController(Model& aModel) :
	Controller(aModel)
{
	setNull();
}
//_____________________________________________________________________________
/**
 * Constructor from an XML Document
  */
  PrescribedController::PrescribedController(const std::string &aFileName, bool aUpdateFromXMLNode) :
      Controller(aFileName, aUpdateFromXMLNode)
{
      setNull();
      if(aUpdateFromXMLNode) updateFromXMLNode();
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 */
PrescribedController::PrescribedController(const PrescribedController &aPrescribedController) :
	Controller(aPrescribedController)
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
	setType("PrescribedController");
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void PrescribedController::setupProperties()
{
}
//_____________________________________________________________________________
/**
 * Copy the member variables of the specified controller.
 */
void PrescribedController::copyData(const PrescribedController &aController)
{
	Controller::copyData(aController);
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

	for(int i=0; i<_actuatorSet.getSize(); i++){
		actControls[0] = _prescribedControlFunctions[i]->calcValue(time);
		_actuatorSet[i].addInControls(actControls, controls);
	}  
}

//=============================================================================
// GET AND SET
//=============================================================================

void PrescribedController::prescribeControlForActuator(int index, OpenSim::Function *prescribedFunction)
{
	SimTK_ASSERT( index < _actuatorSet.getSize(), "PrescribedController::computeControl:  index > number of actuators" );
	SimTK_ASSERT( index >= 0,  "PrescribedController::computeControl:  index < 0" );
	if(index >= _prescribedControlFunctions.getSize())
		_prescribedControlFunctions.setSize(index+1);
	_prescribedControlFunctions.set(index, prescribedFunction);  
}

void PrescribedController::prescribeControlForActuator(const std::string actName, OpenSim::Function *prescribedFunction)
{
	int index = _actuatorNameList.findIndex(actName);
	if(index < 0 )
		throw Exception("PrescribedController does not have "+actName+" in its list of actuators to control.");
	prescribeControlForActuator(index, prescribedFunction);
}

