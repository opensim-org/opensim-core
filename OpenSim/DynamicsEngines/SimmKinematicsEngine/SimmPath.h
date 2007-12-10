#ifndef __SimmPath_h__
#define __SimmPath_h__

// SimmPath.h
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


// INCLUDE
#include <iostream>
#include <math.h>
#include <vector>
#include "osimSimmKinematicsEngineDLL.h"
#include <OpenSim/Common/Transform.h>
#include <OpenSim/Common/Storage.h>
#include "SimmStep.h"

namespace OpenSim {

class AbstractBody;

typedef std::vector<SimmStep> JointPath;

//=============================================================================
//=============================================================================
/**
 * A class implementing a path of SIMM joints, representing the transform[s]
 * between one SIMM body and another.
 *
 * @author Peter Loan
 * @version 1.0
 */
class SimmPath
{

//=============================================================================
// DATA
//=============================================================================
private:
	JointPath _path;
	const AbstractBody* _from;
	const AbstractBody* _to;
	Transform _forwardTransform;
	Transform _inverseTransform;
	bool _transformsValid;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	SimmPath();
	SimmPath(JointPath aPath, const AbstractBody* aFromBody, const AbstractBody* aToBody);
	virtual ~SimmPath();

	void invalidate() { _transformsValid = false; }
	const AbstractBody* getFromBody() { return _from; }
	const AbstractBody* getToBody() { return _to; }
	const JointPath& getPath() const { return _path; }
	Transform& getForwardTransform();
	Transform& getInverseTransform();

private:
	void calcTransforms();

//=============================================================================
};	// END of class SimmPath
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __SimmPath_h__


