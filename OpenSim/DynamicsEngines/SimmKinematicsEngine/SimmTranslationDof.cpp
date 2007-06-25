// SimmTranslationDof.cpp
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
#include "SimmTranslationDof.h"
#include <OpenSim/Simulation/Model/AbstractCoordinate.h>

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
SimmTranslationDof::SimmTranslationDof()
{
	setNull();

	_axis[0] = _axis[1] = _axis[2] = 0.0;
	_axisIndex = xTranslation;
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
SimmTranslationDof::~SimmTranslationDof()
{
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aDof SimmTranslationDof to be copied.
 */
SimmTranslationDof::SimmTranslationDof(const SimmTranslationDof &aDof) :
   AbstractDof(aDof)
{
	setNull();
	copyData(aDof);
}

//_____________________________________________________________________________
/**
 * Copy this dof and return a pointer to the copy.
 * The copy constructor for this class is used.
 *
 * @return Pointer to a copy of this SimmTranslationDof.
 */
Object* SimmTranslationDof::copy() const
{
	SimmTranslationDof *dof = new SimmTranslationDof(*this);
	return(dof);
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Perform some set up functions that need to be done each time the
 * SimmTranslationDof is deserialized.
 */
void SimmTranslationDof::updateFromXMLNode()
{
	AbstractDof::updateFromXMLNode();

	if (_name == TX_NAME)
	{
		_axis[0] = 1.0;
		_axisIndex = xTranslation;
	}
	else if (_name == TY_NAME)
	{
		_axis[1] = 1.0;
		_axisIndex = yTranslation;
	}
	else if (_name == TZ_NAME)
	{
		_axis[2] = 1.0;
		_axisIndex = zTranslation;
	}
}

//_____________________________________________________________________________
/**
 * Copy data members from one SimmTranslationDof to another.
 *
 * @param aDof SimmTranslationDof to be copied.
 */
void SimmTranslationDof::copyData(const SimmTranslationDof &aDof)
{
	aDof.getAxis(_axis);
	_axisIndex = aDof._axisIndex;
}

//_____________________________________________________________________________
/**
 * Set the data members of this SimmTranslationDof to their null values.
 */
void SimmTranslationDof::setNull()
{
	setType("SimmTranslationDof");
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
SimmTranslationDof& SimmTranslationDof::operator=(const SimmTranslationDof &aDof)
{
	// BASE CLASS
	AbstractDof::operator=(aDof);

	copyData(aDof);

	return(*this);
}

//=============================================================================
// GET AND SET
//=============================================================================
//_____________________________________________________________________________
/**
 * Get the current value of the translation dof
 *
 * @return The current value of the dof.
 */
double SimmTranslationDof::getValue()
{
	if (_coordinate)
		return _function->evaluate(0, _coordinate->getValue(), 0.0, 0.0);
	else
		return _function->evaluate(0, 0.0, 0.0, 0.0);
}

//_____________________________________________________________________________
/**
 * Set the translation axis.
 *
 * @param aAxis Translation axis.
 */
void SimmTranslationDof::setAxis(const double aAxis[3])
{
	_axis[0] = aAxis[0];
	_axis[1] = aAxis[1];
	_axis[2] = aAxis[2];
}
//_____________________________________________________________________________
/**
 * Get the translation axis.
 *
 * @param rAxis the translation axis is returned here.
 */
void SimmTranslationDof::getAxis(double rAxis[3]) const
{
	rAxis[0] = _axis[0];
	rAxis[1] = _axis[1];
	rAxis[2] = _axis[2];
}

//_____________________________________________________________________________
/**
 * Get the translation.
 *
 * @param rVec the translation is returned here.
 */
void SimmTranslationDof::getTranslation(double rVec[4])
{
	double value = getValue();

	rVec[0] = _axis[0] * value;
	rVec[1] = _axis[1] * value;
	rVec[2] = _axis[2] * value;
	rVec[3] = 1.0;
}
