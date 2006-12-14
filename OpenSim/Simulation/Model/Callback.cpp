// Callback.cpp
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
#include <OpenSim/Tools/Object.h>
#include <OpenSim/Tools/rdMath.h>
#include <OpenSim/Tools/PropertyBool.h>
#include <OpenSim/Tools/PropertyDbl.h>

#include "Callback.h"
#include <OpenSim/Simulation/Simm/AbstractModel.h>

//=============================================================================
// STATICS
//=============================================================================


//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________


using namespace OpenSim;
/**
 * Destructor.
 */
Callback::~Callback()
{
}

//_____________________________________________________________________________
/**
 * Default constructor.
 *
 * @param aModel Model to which the callback mthods apply.
 */
Callback::Callback(AbstractModel *aModel):
	_on(_onProp.getValueBool()),
	_startTime(_startTimeProp.getValueDbl()),
	_endTime(_endTimeProp.getValueDbl())
{
	setNull();
	_model = aModel;
}
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aCallback Callback to copy.
 */
Callback::Callback(const Callback &aCallback):
	Object(aCallback),
	_on(_onProp.getValueBool()),
	_startTime(_startTimeProp.getValueDbl()),
	_endTime(_endTimeProp.getValueDbl())
{
	setNull();
	*this = aCallback;
}
//_____________________________________________________________________________
/**
* Construct an object from file.
*
* The object is constructed from the root element of the XML document.
* The type of object is the tag name of the XML root element.
*
* @param aFileName File name of the document.
*/
Callback::Callback(const std::string &aFileName):
	Object(aFileName),
	_on(_onProp.getValueBool()),
	_startTime(_startTimeProp.getValueDbl()),
	_endTime(_endTimeProp.getValueDbl())
{
	setNull();
	updateFromXMLNode();
}
//_____________________________________________________________________________
/**
* Construct an Callback from DOMElement.
*
* The object is constructed from the root element of the XML document.
* The type of object is the tag name of the XML root element.
*
* @param aFileName File name of the document.
*/
Callback::Callback(DOMElement *aElement):
	Object(aElement),
	_on(_onProp.getValueBool()),
	_startTime(_startTimeProp.getValueDbl()),
	_endTime(_endTimeProp.getValueDbl())
{
	setNull();
	updateFromXMLNode();
}
//_____________________________________________________________________________
/**
 * Virtual copy constructor.
 */
Object* Callback::
copy() const
{
	return(new Callback(*this));
}

//_____________________________________________________________________________
/**
 * Create and return a copy 
 */
Object* Callback::
copy(DOMElement *aElement) const
{
	Callback *c = new Callback(aElement);
	*c = *this;
	c->updateFromXMLNode();
	return(c);
}
//=============================================================================
// CONSTRUCTION AND DESTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set NULL values for member variables.
 */
void Callback::
setNull()
{
	setType("Callback");
	setupProperties();
	_on = true;
	_model = NULL;
	_startTime = rdMath::MINUS_INFINITY;
	_endTime = rdMath::PLUS_INFINITY;
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void Callback::
setupProperties()
{
	_onProp.setName("on");
	_propertySet.append(&_onProp);

	_startTimeProp.setName("start_time");
	_propertySet.append(&_startTimeProp );

	_endTimeProp.setName("end_time");
	_propertySet.append(&_endTimeProp );
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
 *
 * @return Reference to this object.
 */
Callback& Callback::
operator=(const Callback &aCallback)
{
	// BASE CLASS
	Object::operator=(aCallback);
	// Model
	_model = aCallback._model;
	// Status
	_on = aCallback._on;
	// Start/end times
	_startTime = aCallback._startTime;
	_endTime = aCallback._endTime;

	return(*this);
}

//=============================================================================
// GET AND SET
//=============================================================================
//-----------------------------------------------------------------------------
// MODEL
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get a pointer to the model for which this callback has been set.
 *
 * @return Pointer to the model.
 */
AbstractModel* Callback::
getModel() const
{
	return(_model);
}
//_____________________________________________________________________________
/**
 * Set a pointer to the model for which this callback has been set.
 *
 */
void Callback::
setModel(AbstractModel *aModel)
{
	_model=aModel;
}

//-----------------------------------------------------------------------------
// ON/OFF
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Turn this callback on or off.
 *
 * @param aTureFalse Turns analysis on if "true" and off if "false".
 */
void Callback::
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
bool Callback::
getOn() const
{
	return(_on);
}

//-----------------------------------------------------------------------------
// START TIME
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the time at which to begin executing the callback.  Note that the start
 * time should be specified in normalized time units, not in real time units.
 *
 * @param aStartTime Start time expressed in NORMALIZED time units.
 */
void Callback::
setStartTime(double aStartTime)
{
	_startTime = aStartTime;
}
//_____________________________________________________________________________
/**
 * Get the time at which to begin executing the callback, expressed in
 * normalized time units, not real time units.
 *
 * @return Start time expressed in NORMALIZED time units.
 */
double Callback::
getStartTime() const
{
	return(_startTime);
}

//-----------------------------------------------------------------------------
// END TIME
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the time at which to end executing the callback.  Note that the end time
 * should be specified in normalized time units, not in real time units.
 *
 * @param aEndTime Time at which the callback should end execution in
 * NORMALIZED time units.
 */
void Callback::
setEndTime(double aEndTime)
{
	_endTime = aEndTime;
}
//_____________________________________________________________________________
/**
 * Get the time at which to end executing the callback, expressed in
 * normalized time units, not real time units.
 *
 * @return End time expressed in NORMALIZED time units.
 */
double Callback::
getEndTime() const
{
	return(_endTime);
}

