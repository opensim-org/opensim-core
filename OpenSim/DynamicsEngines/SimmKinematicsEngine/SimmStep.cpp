// SimmStep.cpp
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
#include <OpenSim/Common/rdMath.h>
#include <OpenSim/Common/Mtx.h>
#include "SimmStep.h"
#include <OpenSim/Simulation/Model/AbstractJoint.h>

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
SimmStep::SimmStep() :
	_joint(NULL),
	_direction(forward)
{
}

//_____________________________________________________________________________
/**
 * Constructor from a joint and a direction.
 *
 * @param aJoint the joint for this step.
 * @param aDirection the direction the joint is traversed in this step.
 */
SimmStep::SimmStep(AbstractJoint* aJoint, Direction aDirection) :
	_joint(aJoint),
	_direction(aDirection)
{
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
SimmStep::~SimmStep()
{
}

//=============================================================================
// GET AND SET
//=============================================================================
//_____________________________________________________________________________
/**
 * Get the joint transform.
 *
 * @return Reference to the joint transform in the step.
 */
Transform& SimmStep::getJointTransform()
{
	if (_direction == forward)
		_transformCache = _joint->getForwardTransform();
	else
		_transformCache = _joint->getInverseTransform();

	return _transformCache;
}
