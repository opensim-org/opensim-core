// MeasurementSet.cpp
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

#include "MeasurementSet.h"

using namespace std;
using namespace OpenSim;

//=============================================================================
// DESTRUCTOR AND CONSTRUCTORS
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
MeasurementSet::~MeasurementSet(void)
{
}

//_____________________________________________________________________________
/**
 * Default constructor of a MeasurementSet.
 */
MeasurementSet::MeasurementSet() :
	Set<Measurement>()
{
	setNull();
}

//_____________________________________________________________________________
/**
 * Copy constructor of a MeasurementSet.
 */
MeasurementSet::MeasurementSet(const MeasurementSet& aSimmMeasurementSet):
	Set<Measurement>(aSimmMeasurementSet)
{
	setNull();
	*this = aSimmMeasurementSet;
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
/**
 * Set the data members of this MeasurementSet to their null values.
 */
void MeasurementSet::setNull()
{
	setType("MeasurementSet");
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
MeasurementSet& MeasurementSet::operator=(const MeasurementSet &aSimmMeasurementSet)
{
	Set<Measurement>::operator=(aSimmMeasurementSet);
	return (*this);
}
#endif
