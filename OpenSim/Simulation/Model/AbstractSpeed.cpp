// AbstractSpeed.cpp
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
#include "AbstractSpeed.h"

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
AbstractSpeed::AbstractSpeed() :
	Object()
{
	setNull();
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
AbstractSpeed::~AbstractSpeed()
{
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aSpeed AbstractSpeed to be copied.
 */
AbstractSpeed::AbstractSpeed(const AbstractSpeed &aSpeed) :
	Object(aSpeed)
{
	setNull();
	copyData(aSpeed);
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Copy data members from one AbstractSpeed to another.
 *
 * @param aSpeed AbstractSpeed to be copied.
 */
void AbstractSpeed::copyData(const AbstractSpeed &aSpeed)
{
	_dynamicsEngine = aSpeed._dynamicsEngine;
}

//_____________________________________________________________________________
/**
 * Set the data members of this AbstractSpeed to their null values.
 */
void AbstractSpeed::setNull()
{
	setType("AbstractSpeed");
}

//_____________________________________________________________________________
/**
 * Perform set up functions after model has been deserialized or copied.
 *
 * @param aEngine dynamics engine containing this coordinate.
 */
void AbstractSpeed::setup(AbstractDynamicsEngine* aEngine)
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
AbstractSpeed& AbstractSpeed::operator=(const AbstractSpeed &aSpeed)
{
	// BASE CLASS
	Object::operator=(aSpeed);

	copyData(aSpeed);

	return *this;
}
//=============================================================================
// STATIC METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Get speed name from a coordinate name
 *
 * @param aCoordinateName Coordinate name.
 * @return Speed name.
 */
std::string AbstractSpeed::getSpeedName(const std::string &aCoordinateName)
{
	return aCoordinateName + "_u";
}
//_____________________________________________________________________________
/**
 * Get coordinate name from a speed name
 *
 * @param aSpeedName Speed name
 * @return Coordinate name.
 */
std::string AbstractSpeed::getCoordinateName(const std::string &aSpeedName)
{
	if(aSpeedName.length()>=2 && aSpeedName.substr(aSpeedName.length()-2)=="_u")
		return aSpeedName.substr(0,aSpeedName.length()-2);
	else
		return aSpeedName;
}
