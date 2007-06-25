// SimmPath.cpp
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
#include "SimmPath.h"
#include <OpenSim/Simulation/Model/AbstractBody.h>
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
SimmPath::SimmPath() :
   _from(NULL),
	_to(NULL),
	_transformsValid(false)
{
}

//_____________________________________________________________________________
/**
 * Constructor from a path, starting body, and ending body
 *
 * @param aPath the path from aFromBody to aToBody
 * @param aFromBody the first body in aPath
 * @param aToBody the last body in aPath
 */
SimmPath::SimmPath(JointPath aPath, const AbstractBody* aFromBody, const AbstractBody* aToBody) :
   _path(aPath),
   _from(aFromBody),
	_to(aToBody),
	_transformsValid(false)
{
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
SimmPath::~SimmPath()
{
}

//=============================================================================
// TRANSFORMS
//=============================================================================
//_____________________________________________________________________________
/**
 * Get the forward transform for the path, recalculating it if necessary.
 *
 * @return Reference to the transform
 */
Transform& SimmPath::getForwardTransform()
{
	if (!_transformsValid)
		calcTransforms();

	return _forwardTransform;
}

//_____________________________________________________________________________
/**
 * Get the inverse transform for the path, recalculating it if necessary.
 *
 * @return Reference to the transform
 */
Transform& SimmPath::getInverseTransform()
{
	if (!_transformsValid)
		calcTransforms();

	return _inverseTransform;
}

//_____________________________________________________________________________
/**
 * Calculate the transform for the path by concatenating the transforms for
 * all of the joints in the path.
 *
 * @return Reference to the transform
 */
void SimmPath::calcTransforms()
{
	if (_path.size() >= 1)
	{
		/* Get the transform for the first joint in the path. Copying it right into
		 * _forwardTransform saves the step of multiplying it by the identity matrix
		 * as the first iteration in the 'for' loop that follows.
		 */
		_forwardTransform = _path[0].getJointTransform();

		/* Now get the remaining joint transforms and append them onto the current stack,
		 * stored in _forwardTransform.
		 */
		for (unsigned int i = 1; i < _path.size(); i++)
		{
			Mtx::Multiply(4, 4, 4, _path[i].getJointTransform().getMatrix(),
				_forwardTransform.getMatrix(), _forwardTransform.getMatrix());
		}

		/* Invert the forward transform to get the inverse. */
		Mtx::Invert(4, _forwardTransform.getMatrix(), _inverseTransform.getMatrix());
	}

   _transformsValid = true;
}
