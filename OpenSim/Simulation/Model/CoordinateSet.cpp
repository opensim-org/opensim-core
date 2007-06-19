// CoordinateSet.cpp
// Author: Peter Loan
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

#include "CoordinateSet.h"

using namespace std;
using namespace OpenSim;

//=============================================================================
// DESTRUCTOR AND CONSTRUCTORS
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
CoordinateSet::~CoordinateSet(void)
{
}

//_____________________________________________________________________________
/**
 * Default constructor of a CoordinateSet.
 */
CoordinateSet::CoordinateSet() :
	Set<AbstractCoordinate>()
{
	setNull();
}

//_____________________________________________________________________________
/**
 * Copy constructor of a CoordinateSet.
 */
CoordinateSet::CoordinateSet(const CoordinateSet& aCoordinateSet):
	Set<AbstractCoordinate>(aCoordinateSet)
{
	setNull();
	*this = aCoordinateSet;
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
/**
 * Set the data members of this CoordinateSet to their null values.
 */
void CoordinateSet::setNull()
{
	setType("CoordinateSet");
}
/**
 * Post construction initialization.
 */
void CoordinateSet::setup(AbstractDynamicsEngine* aAbstractDynamicsEngine)
{
	// Base class
	Set<AbstractCoordinate>::setup();

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
CoordinateSet& CoordinateSet::operator=(const CoordinateSet &aCoordinateSet)
{
	Set<AbstractCoordinate>::operator=(aCoordinateSet);
	return (*this);
}
#endif
