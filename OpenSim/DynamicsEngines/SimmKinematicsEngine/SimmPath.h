#ifndef __SimmPath_h__
#define __SimmPath_h__

// SimmPath.h
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


