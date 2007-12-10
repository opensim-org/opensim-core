// SimmPath.cpp
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
