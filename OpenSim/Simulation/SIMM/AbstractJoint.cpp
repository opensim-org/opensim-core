// AbstractJoint.cpp
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
#include "AbstractJoint.h"

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
AbstractJoint::AbstractJoint() :
	Object(),
	_dynamicsEngine(NULL),
	_transformsValid(false)
{
	setNull();
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
AbstractJoint::~AbstractJoint()
{
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aJoint AbstractJoint to be copied.
 */
AbstractJoint::AbstractJoint(const AbstractJoint &aJoint) :
	Object(aJoint),
	_dynamicsEngine(NULL),
	_transformsValid(false)
{
	copyData(aJoint);
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Copy data members from one AbstractJoint to another.
 *
 * @param aJoint AbstractJoint to be copied.
 */
void AbstractJoint::copyData(const AbstractJoint &aJoint)
{
	_dynamicsEngine = aJoint._dynamicsEngine;
	_transformsValid = false;
}

//_____________________________________________________________________________
/**
 * Set the data members of this AbstractJoint to their null values.
 */
void AbstractJoint::setNull()
{
	setType("AbstractJoint");
}

//_____________________________________________________________________________
/**
 * Perform set up functions after model has been deserialized or copied.
 *
 * @param aEngine dynamics engine containing this joint.
 */
void AbstractJoint::setup(AbstractDynamicsEngine* aEngine)
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
AbstractJoint& AbstractJoint::operator=(const AbstractJoint &aJoint)
{
	// BASE CLASS
	Object::operator=(aJoint);

	copyData(aJoint);

	return(*this);
}
