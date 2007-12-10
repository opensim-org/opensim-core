// SimmPathMatrix.cpp
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
#include "SimmPathMatrix.h"
#include <OpenSim/Simulation/Model/AbstractBody.h>
#include <ostream>

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace OpenSim;

const int SimmPathMatrix::cSizeFactor = 2;
const int SimmPathMatrix::cHash1 = 27;
const int SimmPathMatrix::cHash2 = 7;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */
SimmPathMatrix::SimmPathMatrix(int size)
{
	if (size > 0)
	{
		_size = size * size * 2;
		_hashTable.resize(_size);
		_factor = size;
		for (int i = 0; i < _size; i++)
			_hashTable[i] = NULL;
	}
	else
		_size = 0;
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
SimmPathMatrix::~SimmPathMatrix()
{
	deletePaths();
}

//=============================================================================
// GET AND SET
//=============================================================================
//_____________________________________________________________________________
/**
 * Get a SimmPath between two bodies.
 *
 * @param aFromBody the first body
 * @param aToBody the second body
 * @return Pointer to the SimmPath from aFromBody to aToBody
 */
SimmPath* SimmPathMatrix::getSimmPath(const AbstractBody* aFromBody, const AbstractBody* aToBody) const
{
	int index = hash(aFromBody, aToBody);

	assert(index >= 0);

	return _hashTable[index];
}

//_____________________________________________________________________________
/**
 * Get a JointPath between two bodies.
 *
 * @param aFromBody the first body
 * @param aToBody the second body
 * @return Pointer to the JointPath from aFromBody to aToBody
 */
const JointPath* SimmPathMatrix::getPath(const AbstractBody* aFromBody, const AbstractBody* aToBody) const
{
	int index = hash(aFromBody, aToBody);

	assert(index >= 0);

	if (_hashTable[index])
		return &_hashTable[index]->getPath();

	return NULL;
}

//_____________________________________________________________________________
/**
 * Set the path between two bodies.
 *
 * @param aFromBody the first body
 * @param aToBody the second body
 * @param Pointer to the JointPath from aFromBody to aToBody
 */
void SimmPathMatrix::setPath(const AbstractBody* aFromBody, const AbstractBody* aToBody, JointPath aPath)
{
	int index = hash(aFromBody, aToBody);

	assert(index >= 0);

	_hashTable[index] = new SimmPath(aPath, aFromBody, aToBody);
}

//=============================================================================
// UTILITY
//=============================================================================
//_____________________________________________________________________________
/**
 * Delete all paths in the SimmPathMatrix.
 */
void SimmPathMatrix::deletePaths()
{
	for (unsigned int i = 0; i < _hashTable.size(); i++)
	{
		if (_hashTable[i])
			delete _hashTable[i];
	}

	_size = 0;
}

//_____________________________________________________________________________
/**
 * Initialize the has table which holds the paths
 */
void SimmPathMatrix::initTable(int size)
{
	/* If there are already some paths stored in the hash table, delete them. */
	if (_size > 0)
		deletePaths();

	/* Make room for the new paths. */
	if (size > 0)
	{
		_size = size * size * cSizeFactor;
		_hashTable.resize(_size);
		_factor = size;
		for (int i = 0; i < _size; i++)
			_hashTable[i] = NULL;
	}
}

void SimmPathMatrix::invalidate()
{
	for (int i = 0; i < _size; i++)
		if (_hashTable[i])
			_hashTable[i]->invalidate();
}

//_____________________________________________________________________________
/**
 * Hash a pair of bodies
 *
 * @param aFromBody the first body
 * @param aToBody the second body
 * @return The integer hash value
 */
int SimmPathMatrix::hash(const AbstractBody* aFromBody, const AbstractBody* aToBody) const
{
	unsigned int hash_value = ((unsigned int)(uintptr_t)aFromBody / cHash1 + (unsigned int)(uintptr_t)aToBody) / cHash2 % _size;

	SimmPath* hashEntry;
	for (int i = 0; i < _size; i++)
	{
		hashEntry = _hashTable[hash_value];
		if ( hashEntry == NULL ||
			(hashEntry->getFromBody() == aFromBody && hashEntry->getToBody() == aToBody))
		{
			return hash_value;
		}
		hash_value++;
		if (hash_value >= (unsigned int)_size)
			hash_value = 0;
	}

	return -1;
}
