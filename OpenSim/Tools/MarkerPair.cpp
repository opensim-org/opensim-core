// MarkerPair.cpp
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
#include "MarkerPair.h"

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
MarkerPair::MarkerPair() :
	_markerNames(_markerNamesProp.getValueStrArray())
{
	setNull();
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
MarkerPair::~MarkerPair()
{
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aMarkerPair MarkerPair to be copied.
 */
MarkerPair::MarkerPair(const MarkerPair &aMarkerPair) :
   Object(aMarkerPair),
	_markerNames(_markerNamesProp.getValueStrArray())
{
	setupProperties();
	copyData(aMarkerPair);
}
//_____________________________________________________________________________
/**
 * Copy this MarkerPair and return a pointer to the copy.
 * The copy constructor for this class is used.
 *
 * @return Pointer to a copy of this MarkerPair.
 */
Object* MarkerPair::copy() const
{
	MarkerPair *markerPair = new MarkerPair(*this);
	return(markerPair);
}

void MarkerPair::copyData(const MarkerPair &aMarkerPair)
{
	_markerNames = aMarkerPair._markerNames;
}


//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the data members of this MarkerPair to their null values.
 */
void MarkerPair::setNull()
{
	setupProperties();
	setType("MarkerPair");
	setName("");
}
//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void MarkerPair::setupProperties()
{
	_markerNamesProp.setComment("Names of two markers, the distance between which is used to compute a body scale factor.");
	_markerNamesProp.setName("markers");
	_propertySet.append(&_markerNamesProp);
}

MarkerPair& MarkerPair::operator=(const MarkerPair &aMarkerPair)
{
	// BASE CLASS
	Object::operator=(aMarkerPair);

	copyData(aMarkerPair);

	return(*this);
}

void MarkerPair::getMarkerNames(string& aName1, string& aName2) const
{
	aName1 = _markerNames[0];
	aName2 = _markerNames[1];
}

void MarkerPair::peteTest() const
{
	cout << "         MarkerPair: " << getName() << endl;
	cout << "            _markerNames: " << _markerNames << endl;
}

