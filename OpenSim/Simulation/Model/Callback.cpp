// Callback.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c)  2005, Stanford University. All rights reserved. 
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
 * Author: Frank C. Anderson 
 */


//=============================================================================
// INCLUDES
//=============================================================================
#include <OpenSim/Common/Object.h>
#include <OpenSim/Common/rdMath.h>
#include <OpenSim/Common/PropertyBool.h>
#include <OpenSim/Common/PropertyDbl.h>

#include "Callback.h"
#include "Model.h"

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
Callback::Callback(Model *aModel):
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
Callback::Callback(const std::string &aFileName, bool aUpdateFromXMLNode):
	Object(aFileName, false),
	_on(_onProp.getValueBool()),
	_startTime(_startTimeProp.getValueDbl()),
	_endTime(_endTimeProp.getValueDbl())
{
	setNull();
	if(aUpdateFromXMLNode) updateFromXMLNode();
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
	_onProp.setComment("Flag (true or false) specifying whether whether on. "
		"True by default.");
	_onProp.setName("on");
	_propertySet.append(&_onProp);

	_startTimeProp.setComment("Start time.");
	_startTimeProp.setName("start_time");
	_propertySet.append(&_startTimeProp );

	_endTimeProp.setComment("End time.");
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
Model* Callback::
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
setModel(Model *aModel)
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

