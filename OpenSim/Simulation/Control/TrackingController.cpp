// TrackingController.cpp
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

/* Note: This code was originally developed by Realistic Dynamics Inc. 
 * Author: Frank C. Anderson, Chand T. John, Samuel R. Hamner, Ajay Seth
 */


//=============================================================================
// INCLUDES
//=============================================================================
#include <cstdio>
#include <OpenSim/Common/Object.h>
#include <OpenSim/Common/Set.h>
#include "TrackingController.h"
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
TrackingController::TrackingController() :
	Controller()
{
	setNull();
}
//_____________________________________________________________________________
/**
 * Constructor.
 */
TrackingController::TrackingController(Model& aModel) :
	Controller(aModel)
{
	setNull();
}
//_____________________________________________________________________________
/**
 * Constructor from an XML Document
  */
  TrackingController::TrackingController(const std::string &aFileName, bool aUpdateFromXMLNode) :
      Controller(aFileName, aUpdateFromXMLNode)
{
      setNull();
      if(aUpdateFromXMLNode) updateFromXMLNode();
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 */
TrackingController::TrackingController(const TrackingController &aTrackingController) :
	Controller(aTrackingController)
{
	setNull();
	copyData(aTrackingController);
}
//_____________________________________________________________________________
/**
 * Destructor.
 */
TrackingController::~TrackingController()
{

}


//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set NULL values for all member variables.
 */
void TrackingController::setNull()
{
	setupProperties();
	setType("TrackingController");
	_desiredStatesStorage = NULL;
	_trackingTasks = NULL;
}
//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void TrackingController::setupProperties()
{
    string comment;
	Controller::setupProperties();
}
//_____________________________________________________________________________
/**
 * Copy the member variables of the specified controller.
 */
void TrackingController::copyData(const TrackingController &aController)
{
	Controller::copyData(aController);
	_desiredStatesStorage = aController._desiredStatesStorage;
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
TrackingController& TrackingController::operator=(const TrackingController &aController)
{
	// BASE CLASS
	Controller::operator=(aController);

	// DATA
	copyData(aController);

	return(*this);
}


//=============================================================================
// GET AND SET
//=============================================================================
void TrackingController::setDesiredStatesStorage(const Storage* aYDesStore)
{
	_desiredStatesStorage = aYDesStore;
}

const Storage& TrackingController:: getDesiredStatesStorage() const
{
	return *_desiredStatesStorage;
}

