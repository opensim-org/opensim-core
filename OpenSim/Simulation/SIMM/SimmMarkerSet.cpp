// SimmMarkerSet.cpp
// Author: Ayman Habib, Peter Loan
/* Copyright (c) 2005, Stanford University, Ayman Habib, and Peter Loan.
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

#include "SimmMarkerSet.h"



using namespace OpenSim;
using namespace std;

//=============================================================================
// DESTRUCTOR AND CONSTRUCTORS
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
SimmMarkerSet::~SimmMarkerSet(void)
{
}

//_____________________________________________________________________________
/**
 * Constructor of a markerSet from a file.
 */
SimmMarkerSet::SimmMarkerSet(const string& aMarkersFileName) :
	Set<SimmMarker>(aMarkersFileName)
{
	setNull();
	updateFromXMLNode();
}

//_____________________________________________________________________________
/**
 * Default constructor of a markerSet.
 */
SimmMarkerSet::SimmMarkerSet() :
	Set<SimmMarker>()
{
	setNull();
}

void SimmMarkerSet::setNull()
{
	setType("SimmMarkerSet");
	setName("");
}
//_____________________________________________________________________________
/**
 * Copy constructor of a aSimmMarkerSet.
 */
SimmMarkerSet::SimmMarkerSet(const SimmMarkerSet& aSimmMarkerSet):
	Set<SimmMarker>(aSimmMarkerSet)
{
	setNull();
	*this = aSimmMarkerSet;
}
//_____________________________________________________________________________
/**
 * Add name prefix.
 */
void SimmMarkerSet::addNamePrefix(const std::string& prefix)
{
	// Cycle thru set and add prefix
	for(int i=0; i <getSize(); i++){
		get(i)->setName(prefix + get(i)->getName());
	}
}