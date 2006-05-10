// SimmMotionEvent.cpp
// Author: Peter Loan
/* Copyright (c) 2005, Stanford University and Peter Loan.
 * 
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including 
 * without limitation the rights to use, copy, modify, merge, publish, 
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject
 * to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included 
 * in all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */


//=============================================================================
// INCLUDES
//=============================================================================
#include "SimmMotionEvent.h"

//=============================================================================
// STATICS
//=============================================================================


using namespace OpenSim;
using namespace std;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */
SimmMotionEvent::SimmMotionEvent() :
	_time(_timeProp.getValueDbl()),
	_color(_colorProp.getValueDblArray())
{
	setNull();
}
//_____________________________________________________________________________
/**
 * Constructor from an XML node
 */
SimmMotionEvent::SimmMotionEvent(DOMElement *aElement) :
   Object(aElement),
	_time(_timeProp.getValueDbl()),
	_color(_colorProp.getValueDblArray())
{
	setNull();
	updateFromXMLNode();
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
	_color(_colorProp.getValueDblArray())
{
	setupProperties();
	copyData(aEvent);
}
//_____________________________________________________________________________
/**
 * Copy this SimmMotionEvent and return a pointer to the copy.
 * The copy constructor for this class is used.
 *
 * @return Pointer to a copy of this SimmMotionEvent.
 */
Object* SimmMotionEvent::copy() const
{
	SimmMotionEvent *event = new SimmMotionEvent(*this);
	return(event);
}
//_____________________________________________________________________________
/**
 * Copy this SimmMotionEvent and modify the copy so that it is consistent
 * with a specified XML element node.
 *
 * The copy is constructed by first using
 * SimmGenericModelParams::SimmGenericModelParams(DOMElement*) in order to establish the
 * relationship of the SimmGenericModelParams object with the XML node. Then, the
 * assignment operator is used to set all data members of the copy to the
 * values of this SimmGenericModelParams object. Finally, the data members of the copy are
 * updated using SimmGenericModelParams::updateFromXMLNode().
 *
 * @param aElement XML element. 
 * @return Pointer to a copy of this SimmMotionEvent.
 */
Object* SimmMotionEvent::copy(DOMElement *aElement) const
{
	SimmMotionEvent *event = new SimmMotionEvent(aElement);
	*event = *this;
	event->updateFromXMLNode();
	return(event);
}

void SimmMotionEvent::copyData(const SimmMotionEvent &aEvent)
{
	_time = aEvent._time;
	_color = aEvent._color;
}


//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the data members of this SimmMotionEvent to their null values.
 */
void SimmMotionEvent::setNull()
{
	setupProperties();
	setType("SimmMotionEvent");
	setName("");
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

SimmMotionEvent& SimmMotionEvent::operator=(const SimmMotionEvent &aEvent)
{
	// BASE CLASS
	Object::operator=(aEvent);

	copyData(aEvent);

	return(*this);
}

void SimmMotionEvent::setColor(double* aColor)
{
	for (int i = 0; i < 3; i++)
		_color[i] = aColor[i];
}

void SimmMotionEvent::peteTest() const
{
	cout << "   Event: " << getName() << endl;
	cout << "      time: " << _time << endl;
	cout << "      color: " << _color << endl;
}

