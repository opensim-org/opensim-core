// SimmMuscleSet.cpp
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

#include "SimmMuscleSet.h"

using namespace std;
using namespace OpenSim;

//=============================================================================
// DESTRUCTOR AND CONSTRUCTORS
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
SimmMuscleSet::~SimmMuscleSet(void)
{
}

//_____________________________________________________________________________
/**
 * Default constructor of a SimmMuscleSet.
 */
SimmMuscleSet::SimmMuscleSet() :
	Set<SimmMuscle>()
{
	setNull();
}

//_____________________________________________________________________________
/**
 * Copy constructor of a SimmMuscleSet.
 */
SimmMuscleSet::SimmMuscleSet(const SimmMuscleSet& aSimmMuscleSet):
	Set<SimmMuscle>(aSimmMuscleSet)
{
	setNull();
	*this = aSimmMuscleSet;
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
/**
 * Set the data members of this SimmMuscleSet to their null values.
 */
void SimmMuscleSet::setNull()
{
	setType("SimmMuscleSet");
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
#ifndef SWIG
SimmMuscleSet& SimmMuscleSet::operator=(const SimmMuscleSet &aSimmMuscleSet)
{
	Set<SimmMuscle>::operator=(aSimmMuscleSet);
	return (*this);
}
#endif
