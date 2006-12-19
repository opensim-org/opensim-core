// Controller.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c) 2005, Stanford University. All rights reserved. 
* Redistribution and use in source and binary forms, with or without 
* modification, are permitted provided that the following conditions
* are met: 
*  - Redistributions of source code must retain the above copyright 
*    notice, this list of conditions and the following disclaimer. 
*  - Redistributions in binary form must reproduce the above copyright 
*    notice, this list of conditions and the following disclaimer in the 
*    documentation and/or other materials provided with the distribution. 
*  - Neither the name of the Stanford University nor the names of its 
*    contributors may be used to endorse or promote products derived 
*    from this software without specific prior written permission. 
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, 
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN 
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
* POSSIBILITY OF SUCH DAMAGE. 
*/

/* Note: This code was originally developed by Realistic Dynamics Inc. 
 * Author: Frank C. Anderson 
 */


//=============================================================================
// INCLUDES
//=============================================================================
#include "Controller.h"
#include <OpenSim/Simulation/SIMM/AbstractModel.h>


//=============================================================================
// STATICS
//=============================================================================


using namespace OpenSim;

const int Controller::NAME_LENGTH = ControllerNAME_LENGTH;	
// Remove _ before NAME_LENGTH to make SWIG happy since Controller::NAME_LENGTH gets
// wrapped as Controller_NAME_LENGTH. -Ayman 8/08
const int Controller::DESCRIP_LENGTH = ControllerDESCRIP_LENGTH;


//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 *
 * @param aModel AbstractModel that is to be controlled.
 */
Controller::Controller(AbstractModel *aModel)
{
	// NULL
	setNull();

	// MODEL
	_model = aModel;
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
void Controller::
setNull()
{
	// MODEL
	_model = NULL;

	// STATES
	_on = true;

	// NAME
	setName("UNKOWN");

	// DESCRIPTION
	setDescription("");

}


//=============================================================================
// GET AND SET
//=============================================================================
//-----------------------------------------------------------------------------
// MODEL
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get a pointer to the model on which this analysis is being performed.
 *
 * @return Pointer to the model.
 */
AbstractModel* Controller::
getModel()
{
	return(_model);
}

//-----------------------------------------------------------------------------
// ON/OFF
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Turn this analysis on or off.
 *
 * @param aTureFalse Turns analysis on if "true" and off if "false".
 */
void Controller::
setOn(bool aTrueFalse)
{
	_on = aTrueFalse;
}
//_____________________________________________________________________________
/**
 * Get whether or not this analysis is on.
 *
 * @return True if on, false if off.
 */
bool Controller::
getOn()
{
	return(_on);
}

//-----------------------------------------------------------------------------
// NAME
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the name of this controller.
 *
 * @param aName Name.
 */
void Controller::
setName(const char *aName)
{
	if(aName==NULL) {
		strcpy(_name,"");
	} else {
		strncpy(_name,aName,NAME_LENGTH-2);
	}
	_name[NAME_LENGTH-1] = 0;
}
//_____________________________________________________________________________
/**
 * Get the name of this controller.
 *
 * @return Name.
 */
const char* Controller::
getName()
{
	return(_name);
}

//-----------------------------------------------------------------------------
// DESCRIPTION
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the description of this controller.
 *
 * @param aDescrip Description.
 */
void Controller::
setDescription(const char *aDescrip)
{
	if(aDescrip==NULL) {
		strcpy(_descrip,"");
	} else {
		strncpy(_descrip,aDescrip,DESCRIP_LENGTH-2);
	}
	_descrip[DESCRIP_LENGTH-1] = 0;
}
//_____________________________________________________________________________
/**
 * Get the description of this controller.
 *
 * @return Description.
 */
const char* Controller::
getDescription()
{
	return(_descrip);
}


//=============================================================================
// CONTROL
//=============================================================================


