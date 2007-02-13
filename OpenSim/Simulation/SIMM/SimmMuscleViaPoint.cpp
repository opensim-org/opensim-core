// SimmMuscleViaPoint.cpp
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
#include "SimmMuscleViaPoint.h"
#include "AbstractModel.h"
#include "AbstractSimmMuscle.h"
#include "AbstractBody.h"
#include "AbstractCoordinate.h"
#include "CoordinateSet.h"
#include "AbstractDynamicsEngine.h"

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
SimmMuscleViaPoint::SimmMuscleViaPoint() :
   _range(_rangeProp.getValueDblArray()),
	_coordinateName(_coordinateNameProp.getValueStr()),
	_coordinate(NULL)
{
	setNull();
	setupProperties();
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
SimmMuscleViaPoint::~SimmMuscleViaPoint()
{
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aPoint SimmMuscleViaPoint to be copied.
 */
SimmMuscleViaPoint::SimmMuscleViaPoint(const SimmMuscleViaPoint &aPoint) :
   SimmMusclePoint(aPoint),
   _range(_rangeProp.getValueDblArray()),
	_coordinateName(_coordinateNameProp.getValueStr()),
	_coordinate(NULL)
{
	setNull();
	setupProperties();
	copyData(aPoint);
}

//_____________________________________________________________________________
/**
 * Copy this muscle via point and return a pointer to the copy.
 * The copy constructor for this class is used.
 *
 * @return Pointer to a copy of this SimmMuscleViaPoint.
 */
Object* SimmMuscleViaPoint::copy() const
{
	SimmMuscleViaPoint *pt = new SimmMuscleViaPoint(*this);
	return(pt);
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Copy data members from one SimmMuscleViaPoint to another.
 *
 * @param aPoint SimmMuscleViaPoint to be copied.
 */
void SimmMuscleViaPoint::copyData(const SimmMuscleViaPoint &aPoint)
{
	_range = aPoint._range;
	_coordinateName = aPoint._coordinateName;
	_coordinate = aPoint._coordinate;
}

//_____________________________________________________________________________
/**
 * Set the data members of this SimmMuscleViaPoint to their null values.
 */
void SimmMuscleViaPoint::setNull()
{
	setType("SimmMuscleViaPoint");
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void SimmMuscleViaPoint::setupProperties()
{
	const double defaultRange[] = {0.0, 0.0};
	_rangeProp.setName("range");
	_rangeProp.setValue(2, defaultRange);
	_propertySet.append(&_rangeProp);

	_coordinateNameProp.setName("coordinate");
	_propertySet.append(&_coordinateNameProp);
}

//_____________________________________________________________________________
/**
 * Determine if this point is active by checking the value of the
 * coordinate that it is linked to.
 *
 * @return Whether or not this point is active.
 */
bool SimmMuscleViaPoint::isActive() const
{
	if (_coordinate)
	{
		double value = _coordinate->getValue();
		if (value >= _range[0] && value <= _range[1])
			return true;
	}

	return false;
}

//_____________________________________________________________________________
/**
 * Perform some set up functions that happen after the
 * object has been deserialized or copied.
 *
 * @param aModel model containing this SimmMuscleViaPoint.
 */
void SimmMuscleViaPoint::setup(AbstractModel* aModel, AbstractSimmMuscle* aMuscle)
{
	// base class
	SimmMusclePoint::setup(aModel, aMuscle);

	/* Look up the coordinate by name in the dynamics engine and
	 * store a pointer to it.
	 */
	_coordinate = aModel->getDynamicsEngine().getCoordinateSet()->get(_coordinateName);
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
SimmMuscleViaPoint& SimmMuscleViaPoint::operator=(const SimmMuscleViaPoint &aPoint)
{
	// BASE CLASS
	SimmMusclePoint::operator=(aPoint);

	copyData(aPoint);

	return(*this);
}

void SimmMuscleViaPoint::peteTest() const
{
	cout << "   MuscleViaPoint: " << getName() << endl;
	cout << "      point: " << getAttachment() << endl;
	cout << "      body: " << _body->getName() << endl;
	cout << "      range: " << getRange() << endl;
	cout << "      coordinate: " << _coordinate->getName() << endl;
}
