// SimmMarkerPair.cpp
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
#include "SimmMarkerPair.h"

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
SimmMarkerPair::SimmMarkerPair() :
	_markerNames(_markerNamesProp.getValueStrArray())
{
	setNull();
}
//_____________________________________________________________________________
/**
 * Constructor from an XML node
 */
SimmMarkerPair::SimmMarkerPair(DOMElement *aElement) :
   Object(aElement),
	_markerNames(_markerNamesProp.getValueStrArray())
{
	setNull();

	updateFromXMLNode();
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
SimmMarkerPair::~SimmMarkerPair()
{
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aMarkerPair SimmMarkerPair to be copied.
 */
SimmMarkerPair::SimmMarkerPair(const SimmMarkerPair &aMarkerPair) :
   Object(aMarkerPair),
	_markerNames(_markerNamesProp.getValueStrArray())
{
	setupProperties();
	copyData(aMarkerPair);
}
//_____________________________________________________________________________
/**
 * Copy this SimmMarkerPair and return a pointer to the copy.
 * The copy constructor for this class is used.
 *
 * @return Pointer to a copy of this SimmMarkerPair.
 */
Object* SimmMarkerPair::copy() const
{
	SimmMarkerPair *markerPair = new SimmMarkerPair(*this);
	return(markerPair);
}
//_____________________________________________________________________________
/**
 * Copy this SimmMarkerPair and modify the copy so that it is consistent
 * with a specified XML element node.
 *
 * The copy is constructed by first using
 * SimmMarkerPair::SimmMarkerPair(DOMElement*) in order to establish the
 * relationship of the SimmMarkerPair object with the XML node. Then, the
 * assignment operator is used to set all data members of the copy to the
 * values of this SimmMarkerPair object. Finally, the data members of the copy are
 * updated using SimmMarkerPair::updateFromXMLNode().
 *
 * @param aElement XML element. 
 * @return Pointer to a copy of this SimmMarkerPair.
 */
Object* SimmMarkerPair::copy(DOMElement *aElement) const
{
	SimmMarkerPair *markerPair = new SimmMarkerPair(aElement);
	*markerPair = *this;
	markerPair->updateFromXMLNode();
	return(markerPair);
}

void SimmMarkerPair::copyData(const SimmMarkerPair &aMarkerPair)
{
	_markerNames = aMarkerPair._markerNames;
}


//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the data members of this SimmMarkerPair to their null values.
 */
void SimmMarkerPair::setNull()
{
	setupProperties();
	setType("SimmMarkerPair");
	setName("");
}
//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void SimmMarkerPair::setupProperties()
{
	_markerNamesProp.setName("markers");
	_propertySet.append(&_markerNamesProp);
}

SimmMarkerPair& SimmMarkerPair::operator=(const SimmMarkerPair &aMarkerPair)
{
	// BASE CLASS
	Object::operator=(aMarkerPair);

	copyData(aMarkerPair);

	return(*this);
}

void SimmMarkerPair::getMarkerNames(const string*& aName1, const string*& aName2) const
{
	aName1 = &_markerNames[0];
	aName2 = &_markerNames[1];
}

void SimmMarkerPair::peteTest() const
{
	cout << "         MarkerPair: " << getName() << endl;
	cout << "            _markerNames: " << _markerNames << endl;
}

