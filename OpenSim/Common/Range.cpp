/*
 * Copyright (c) 2007, Stanford University. All rights reserved. 
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions
 * are met: 
 *  - Redistributions of source code must retain the above copyright 
 *    notice, this list of conditions and the following disclaimer. 
 *  - Redistributions in binary form must reproduce the above copyright 
 *    notice, this list of conditions and the following disclaimer in the 
 *    documentation and/or other materials provided with the distribution. 
 *  - Neither the name of the Stanford University nor the names of its 
 *    contributors may be used to endorse or promote products derived 
 *    from this software without specific prior written permission. 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, 
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN 
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
 * POSSIBILITY OF SUCH DAMAGE. 
 */
#include "Range.h"



using namespace OpenSim;
using namespace std;
//=============================================================================
// DESTRUCTOR AND CONSTRUCTORS
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
Range::~Range(void)
{
}
//_____________________________________________________________________________
/**
 * Default constructor of an Range
 */
Range::Range():
_min(_propMin.getValueDbl()),
_max(_propMax.getValueDbl())
{
	setNull();
}
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aRange range to copy.
 */
Range::Range(const Range &aRange) :
Object(aRange),
_min(_propMin.getValueDbl()),
_max(_propMax.getValueDbl())
{
	setNull();

	// ASSIGN
	*this = aRange;
}

//=============================================================================
// OPERATORS
//=============================================================================
//-----------------------------------------------------------------------------
// ASSIGNMENT
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Assign this object to the values of another.
 *
 * @return Reference to this object.
 */
Range& Range::
operator=(const Range &aRange)
{
	// BASE CLASS
	Object::operator=(aRange);
	_min = aRange._min;
	_max = aRange._max;

	return(*this);
}

//=============================================================================
// GET AND SET
//=============================================================================
//_____________________________________________________________________________
/**
 * Range min. get
 */
const double Range::getMin() const
{
	return _min;
}
//_____________________________________________________________________________
/**
 * Range max. get
 */
const double Range::getMax() const
{
	return _max;
}

//_____________________________________________________________________________
/**
 * Range min. set
 */
void Range::setMin(const double aMin)
{
	_min = aMin;
}
//_____________________________________________________________________________
/**
 * Range max. set
 */
void Range::setMax(const double aMax)
{
	_max = aMax;
}


void Range::setNull()
{
	setName("");
	setupProperties();
}
//_____________________________________________________________________________
/**
 * Set up the serialized member variables.  
 */
void Range::
setupProperties()
{
	// _min
	_propMin.setName("min");
	_propMin.setValue(0.0);
	_propertySet.append( &_propMin );
	// _max
	_propMax.setName("max");
	_propMax.setValue(1.0);
	_propertySet.append( &_propMax );


}
