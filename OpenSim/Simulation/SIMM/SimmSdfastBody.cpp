// SimmSdfastBody.cpp
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
#include "SimmSdfastBody.h"

//=============================================================================
// STATICS
//=============================================================================


using namespace OpenSim;
using namespace std;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */
SimmSdfastBody::SimmSdfastBody(void) :
	_SimmBody(NULL)
{
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aSdfastBody SimmSdfastBody to be copied.
 */
SimmSdfastBody::SimmSdfastBody(const SimmSdfastBody &aSdfastBody) :
	_SimmBody(NULL)
{
	copyData(aSdfastBody);
}

void SimmSdfastBody::copyData(const SimmSdfastBody &aSdfastBody)
{
	_name = aSdfastBody._name;
	_mass = aSdfastBody._mass;
	_SimmBody = aSdfastBody._SimmBody;

	for (int i = 0; i < 3; i++)
	{
		_massCenter[i] = aSdfastBody._massCenter[i];
		_bodyToJoint[i] = aSdfastBody._bodyToJoint[i];
		_inboardToJoint[i] = aSdfastBody._inboardToJoint[i];
		for (int j = 0; j < 3; j++)
			_inertia[i][j] = aSdfastBody._inertia[i][j];
	}
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
SimmSdfastBody::~SimmSdfastBody(void)
{
	_name.erase(_name.begin(),_name.end());
}

SimmSdfastBody* SimmSdfastBody::copy(void) const
{
	SimmSdfastBody *sdfastBody = new SimmSdfastBody(*this);
	return sdfastBody;
}

SimmSdfastBody& SimmSdfastBody::operator=(const SimmSdfastBody &aSdfastBody)
{
	copyData(aSdfastBody);

	return *this;
}
