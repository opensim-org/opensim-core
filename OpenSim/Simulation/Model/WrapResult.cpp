// WrapResult.cpp
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
#include "WrapResult.h"

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
WrapResult::WrapResult()
{
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
WrapResult::~WrapResult()
{
}

//_____________________________________________________________________________
/**
 * Copy data members from one WrapResult to another.
 *
 * @param aWrapResult WrapResult to be copied.
 */
void WrapResult::copyData(const WrapResult& aWrapResult)
{
	wrap_pts = aWrapResult.wrap_pts;
	wrap_path_length = aWrapResult.wrap_path_length;

	startPoint = aWrapResult.startPoint;
	endPoint = aWrapResult.endPoint;

	int i;
	for (i = 0; i < 3; i++) {
		r1[i] = aWrapResult.r1[i];
		r2[i] = aWrapResult.r2[i];
		c1[i] = aWrapResult.c1[i];
		sv[i] = aWrapResult.sv[i];
	}
}

//=============================================================================
// OPERATORS
//=============================================================================
//_____________________________________________________________________________
/**
 * Assignment operator.
 *
 * @return Reference to this object.
 */
WrapResult& WrapResult::operator=(const WrapResult& aWrapResult)
{
	copyData(aWrapResult);

	return(*this);
}
