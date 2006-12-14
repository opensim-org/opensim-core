// MarkerSet.cpp
// Author: Ayman Habib, Peter Loan
/*
 * Copyright (c) 2006, Stanford University. All rights reserved. 
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

#include "MarkerSet.h"
#include <OpenSim/Tools/ScaleSet.h>

using namespace std;
using namespace OpenSim;

//=============================================================================
// DESTRUCTOR AND CONSTRUCTORS
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
MarkerSet::~MarkerSet(void)
{
}

//_____________________________________________________________________________
/**
 * Constructor of a markerSet from a file.
 */
MarkerSet::MarkerSet(const string& aMarkersFileName) :
	Set<AbstractMarker>(aMarkersFileName)
{
	setNull();
	updateFromXMLNode();
}

//_____________________________________________________________________________
/**
 * Default constructor of a markerSet.
 */
MarkerSet::MarkerSet() :
	Set<AbstractMarker>()
{
	setNull();
}

//_____________________________________________________________________________
/**
 * Copy constructor of a markerSet.
 */
MarkerSet::MarkerSet(const MarkerSet& aMarkerSet):
Set<AbstractMarker>(aMarkerSet)
{
	setNull();
	*this = aMarkerSet;
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
/**
 * Set the data members of this MarkerSet to their null values.
 */
void MarkerSet::setNull()
{
	setType("MarkerSet");
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
#ifndef SWIG
MarkerSet& MarkerSet::operator=(const MarkerSet &aMarkerSet)
{
	Set<AbstractMarker>::operator=(aMarkerSet);
	return (*this);
}
#endif

//=============================================================================
// UTILITY
//=============================================================================
//_____________________________________________________________________________
/**
 * Check if the marker set uses weights 
 */
bool MarkerSet::usesWeights() const
{
	bool weighted = false;

	for (int i = 0; i < getSize(); i++)
	{
		AbstractMarker* nextMarker = get(i);
		if (nextMarker->getWeight()!= 1.0)
		{
			weighted = true;
			break;
		}
	}

	return weighted;
}

//_____________________________________________________________________________
/**
 * Get names of markers in the marker set
 */
void MarkerSet::getMarkerNames(Array<string>& aMarkerNamesArray)
{
	for (int i = 0; i < getSize(); i++)
	{
		AbstractMarker* nextMarker = get(i);
		aMarkerNamesArray.append(nextMarker->getName());
	}
}

//_____________________________________________________________________________
/**
 * Scale marker set by a set of scale factors
 */
void MarkerSet::scale(const ScaleSet& scaleSet)
{
	Array<double>	scaleFactors(1.0, 3);

	for (int i = 0; i < getSize(); i++)
	{
		AbstractMarker* nextMarker = get(i);
		const string* refBodyName = nextMarker->getBodyName();
		assert(refBodyName);
		bool found = false;
		for (int j = 0; j < scaleSet.getSize() && !found; j++)
		{
			Scale* nextScale = scaleSet.get(j);
			if (nextScale->getSegmentName() == *refBodyName)
			{
				found = true;
				nextScale->getScaleFactors(scaleFactors);
				nextMarker->scale(scaleFactors);
			}
		}
	}
}

//_____________________________________________________________________________
/**
 * Add name prefix.
 */
void MarkerSet::addNamePrefix(const string& prefix)
{
	int i;

	// Cycle thru set and add prefix
	for (i = 0; i < getSize(); i++)
		get(i)->setName(prefix + get(i)->getName());
}
