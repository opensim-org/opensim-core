// SimmMotionEvent.cpp
// Author: Peter Loan
/*
 * Copyright (c)  2006, Stanford University. All rights reserved. 
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

//=============================================================================
// INCLUDES
//=============================================================================
#include "SimmMotionEvent.h"

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace OpenSim;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */
SimmMotionEvent::SimmMotionEvent() :
	_time(_timeProp.getValueDbl()),
	_color(_colorProp.getValueDblVec())
{
	setNull();
	setupProperties();
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
SimmMotionEvent::~SimmMotionEvent()
{
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aEvent SimmMotionEvent to be copied.
 */
SimmMotionEvent::SimmMotionEvent(const SimmMotionEvent &aEvent) :
   Object(aEvent),
	_time(_timeProp.getValueDbl()),
	_color(_colorProp.getValueDblVec())
{
	setNull();
	setupProperties();
	copyData(aEvent);
}


//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the data members of this SimmMotionEvent to their null values.
 */
void SimmMotionEvent::setNull()
{
}

//_____________________________________________________________________________
/**
 * Copy data members from one SimmMotionEvent to another.
 *
 * @param aEvent SimmMotionEvent to be copied.
 */
void SimmMotionEvent::copyData(const SimmMotionEvent &aEvent)
{
	_time = aEvent._time;
	_color = aEvent._color;
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void SimmMotionEvent::setupProperties()
{
	_timeProp.setName("time");
	_propertySet.append(&_timeProp);

	_colorProp.setName("color");
	_propertySet.append(&_colorProp);
}

//=============================================================================
// OPERATORS
//=============================================================================
//_____________________________________________________________________________
/**
 * Assignment operator.
 *
 * @return Reference to this object.
 */
SimmMotionEvent& SimmMotionEvent::operator=(const SimmMotionEvent &aEvent)
{
	// BASE CLASS
	Object::operator=(aEvent);

	copyData(aEvent);

	return(*this);
}

//=============================================================================
// GET AND SET
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the color of the event.
 *
 * @param aColor array of RGB color values
 */
void SimmMotionEvent::setColor(double* aColor)
{
	for (int i = 0; i < 3; i++)
		_color[i] = aColor[i];
}
