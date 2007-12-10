#ifndef __SimmPathMatrix_h__
#define __SimmPathMatrix_h__

// SimmPathMatrix.h
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


