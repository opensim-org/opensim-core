// AbstractJoint.cpp
// Author: Peter Loan
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

void AbstractJoint::setDynamicsEngine(AbstractDynamicsEngine* aEngine)
{
	_dynamicsEngine = aEngine;
}
