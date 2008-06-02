// MarkerSet.cpp
// Author: Ayman Habib, Peter Loan
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

#include "MarkerSet.h"
#include "Marker.h"
#include <OpenSim/Common/ScaleSet.h>

using namespace std;
using namespace OpenSim;
using SimTK::Vec3;

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
	Set<AbstractMarker>(aMarkersFileName, false)
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

//_____________________________________________________________________________
/**
 * Copy this MarkerSet and return a pointer to the copy.
 * The copy constructor for this class is used.
 *
 * @return Pointer to a copy of this MarkerSet.
 */
Object* MarkerSet::copy() const
{
	return new MarkerSet(*this);
}

/**
 * Post construction initialization.
 */
void MarkerSet::setup(AbstractDynamicsEngine* aAbstractDynamicsEngine)
{
	// Base class
	Set<AbstractMarker>::setup();

	// Do members
	for (int i = 0; i < getSize(); i++)
		get(i)->setup(aAbstractDynamicsEngine);

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
	Vec3	scaleFactors(1.0);

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

//_____________________________________________________________________________
/**
 * Create a new marker and add it to the set.
 */
AbstractMarker* MarkerSet::addMarker(const string& aName, const double aOffset[3], AbstractBody& aBody)
{
	// If a marker by this name already exists, do nothing.
	if (get(aName) != NULL)
		return NULL;

	// Create a marker and add it to the set.
	Marker* m = new Marker();
	m->setName(aName);
	m->setOffset(aOffset);
	m->setBody(aBody, false);
	append(m);

	return m;
}
