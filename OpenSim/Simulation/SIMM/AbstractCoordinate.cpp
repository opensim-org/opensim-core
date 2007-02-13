// AbstractCoordinate.cpp
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

//=============================================================================
// INCLUDES
//=============================================================================
#include "AbstractCoordinate.h"

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
AbstractCoordinate::AbstractCoordinate() :
	_dynamicsEngine(NULL)
{
	setNull();
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
AbstractCoordinate::~AbstractCoordinate()
{
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aCoordinate AbstractCoordinate to be copied.
 */
AbstractCoordinate::AbstractCoordinate(const AbstractCoordinate &aCoordinate) :
   Object(aCoordinate),
	_dynamicsEngine(NULL)
{
	setNull();
	copyData(aCoordinate);
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Copy data members from one AbstractCoordinate to another.
 *
 * @param aCoordinate AbstractCoordinate to be copied.
 */
void AbstractCoordinate::copyData(const AbstractCoordinate &aCoordinate)
{
	_dynamicsEngine = aCoordinate._dynamicsEngine;
}

//_____________________________________________________________________________
/**
 * Set the data members of this AbstractCoordinate to their null values.
 */
void AbstractCoordinate::setNull(void)
{
	setType("AbstractCoordinate");
}

//_____________________________________________________________________________
/**
 * Perform set up functions after model has been deserialized or copied.
 *
 * @param aEngine dynamics engine containing this coordinate.
 */
void AbstractCoordinate::setup(AbstractDynamicsEngine* aEngine)
{
	_dynamicsEngine = aEngine;
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
AbstractCoordinate& AbstractCoordinate::operator=(const AbstractCoordinate &aCoordinate)
{
	// BASE CLASS
	Object::operator=(aCoordinate);

	copyData(aCoordinate);

	return(*this);
}
