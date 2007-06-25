#ifndef __SimmPathMatrix_h__
#define __SimmPathMatrix_h__

// SimmPathMatrix.h
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
#include <string>
#include <iostream>
#include <math.h>
#include <vector>
#include "osimSimmKinematicsEngineDLL.h"
#include "SimmPath.h"

namespace OpenSim {

class AbstractBody;

//=============================================================================
//=============================================================================
/**
 * A class implementing the matrix of paths for a SIMM model. The matrix contains
 * paths for going from any SIMM body to any other SIMM body. The paths are
 * stored in a closed hash table.
 *
 * @author Peter Loan
 * @version 1.0
 */
class SimmPathMatrix
{

//=============================================================================
// DATA
//=============================================================================
private:
	int _size;
	int _factor;
	static const int cSizeFactor;
	static const int cHash1; // determined experimentally using a model
	static const int cHash2;  // with regularly-spaced from/to pointers
	std::vector<SimmPath*> _hashTable;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	SimmPathMatrix(int size);
	virtual ~SimmPathMatrix();

	void initTable(int size);
	void invalidate();
	SimmPath* getSimmPath(const AbstractBody* aFromBody, const AbstractBody* aToBody) const;
	const JointPath* getPath(const AbstractBody* aFromBody, const AbstractBody* aToBody) const;
	void setPath(const AbstractBody* aFromBody, const AbstractBody* aToBody, JointPath aPath);

private:
	int hash(const AbstractBody* aFromBody, const AbstractBody* aToBody) const;
	void deletePaths();

//=============================================================================
};	// END of class SimmPathMatrix
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __SimmPathMatrix_h__


