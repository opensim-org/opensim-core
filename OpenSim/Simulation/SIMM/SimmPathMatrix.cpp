// SimmPathMatrix.cpp
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
#include "SimmPathMatrix.h"
#include "AbstractBody.h"
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
		_hashTable.reserve(_size);
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
		_hashTable.reserve(_size);
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
	int hash_value = ((int)(intptr_t)aFromBody / cHash1 + (int)(intptr_t)aToBody) / cHash2 % _size;

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
		if (hash_value >= _size)
			hash_value = 0;
	}

	return -1;
}

void SimmPathMatrix::peteTest() const
{
	cout << "SimmPathMatrix:" << endl;
	for (int i = 0; i < _size; i++)
	{
		if (_hashTable[i])
			_hashTable[i]->peteTest();
		else
			cout << "slot " << i << " is empty." << endl;
	}
}
