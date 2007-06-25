// Constant.cpp
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
#include "Constant.h"

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace OpenSim;

//=============================================================================
// DESTRUCTOR AND CONSTRUCTORS
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
Constant::~Constant()
{
}

//_____________________________________________________________________________
/**
 * Default constructor.
 */
Constant::Constant() :
	_value(_valueProp.getValueDbl())
{
	setNull();
	setupProperties();
}

//_____________________________________________________________________________
/**
 */
Constant::Constant(int aN,const double *aX,const double *aY,	const string &aName) :
	_value(_valueProp.getValueDbl())
{
	setNull();
	setupProperties();
	// OBJECT TYPE AND NAME
	setName(aName);
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 * All data members of the specified Constant are copied.
 *
 * @param aConstant Constant object to be copied.
 */
Constant::Constant(const Constant &aConstant) :
	Function(aConstant),
	_value(_valueProp.getValueDbl())
{
	setNull();
	setupProperties();
	copyData(aConstant);
}

//_____________________________________________________________________________
/**
 * Copy this object.
 *
 * @return Pointer to a copy of this object.
 */
Object* Constant::copy() const
{
	Constant *aConstant = new Constant(*this);
	return(aConstant);
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Set all member variables to their NULL or default values.
 */
void Constant::setNull()
{
	setType("Constant");
}

//_____________________________________________________________________________
/**
 * Set up the serialized member variables.  This involves both generating
 * the properties and connecting them to the local pointers used to access
 * the serialized member variables.
 */
void Constant::setupProperties()
{
	_valueProp.setName("value");
	_valueProp.setValue(0);
	_propertySet.append(&_valueProp);
}

//_____________________________________________________________________________
/**
 * Set all member variables equal to the members of another object.
 * Note that this method is private.  It is only meant for copying the data
 * members defined in this class.  It does not, for example, make any changes
 * to data members of base classes.
 *
 * @param aConstant Constant to be copied.
 */
void Constant::copyData(const Constant &aConstant)
{
	_value = aConstant._value;
}

//=============================================================================
// OPERATORS
//=============================================================================
//_____________________________________________________________________________
/**
 * Assignment operator.
 * Note that data members of the base class are also assigned.
 *
 * @param aConstant Constant to be copied.
 * @return Reference to this object.
 */
Constant& Constant::operator=(const Constant &aConstant)
{
	// BASE CLASS
	Function::operator=(aConstant);

	// DATA
	copyData(aConstant);

	return(*this);
}

//=============================================================================
// UTILITY
//=============================================================================
//_____________________________________________________________________________
/**
 * Update the bounding box for this function.
 *
 * @see Function
 */
void Constant::updateBoundingBox()
{
	setMinX(0.0);
	setMinY(0.0);
	setMinZ(0.0);
	setMaxX(0.0);
	setMaxY(0.0);
	setMaxZ(0.0);
}
