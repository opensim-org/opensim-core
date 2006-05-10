// SimmPathMatrix.cpp
// Author: Peter Loan
/* Copyright (c) 2005, Stanford University and Peter Loan.
 * 
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
#include "SimmBody.h"
#include <ostream>

//=============================================================================
// STATICS
//=============================================================================


using namespace OpenSim;
const int SimmPathMatrix::cSizeFactor = 2;
const int SimmPathMatrix::cHash1 = 27;
const int SimmPathMatrix::cHash2 = 7;


using namespace std;



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

void SimmPathMatrix::deletePaths()
{
	for (unsigned int i = 0; i < _hashTable.size(); i++)
	{
		if (_hashTable[i])
			delete _hashTable[i];
	}
}

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

int SimmPathMatrix::hash(const SimmBody* aFromBody, const SimmBody* aToBody) const
{
	int hash_value = ((int)aFromBody / cHash1 + (int)aToBody) / cHash2 % _size;
	int foo = hash_value;

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

SimmPath* SimmPathMatrix::getSimmPath(const SimmBody* aFromBody, const SimmBody* aToBody) const
{
	int index = hash(aFromBody, aToBody);

	assert(index >= 0);

	return _hashTable[index];
}

const JointPath* SimmPathMatrix::getPath(const SimmBody* aFromBody, const SimmBody* aToBody) const
{
	int index = hash(aFromBody, aToBody);

	assert(index >= 0);

	if (_hashTable[index])
		return &_hashTable[index]->getPath();

	return NULL;
}

void SimmPathMatrix::setPath(const SimmBody* aFromBody, const SimmBody* aToBody, JointPath p)
{
	int index = hash(aFromBody, aToBody);

	assert(index >= 0);

	_hashTable[index] = new SimmPath(p, aFromBody, aToBody);
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
