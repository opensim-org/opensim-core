// MuscleViaPoint.cpp
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
#include "MuscleViaPoint.h"
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/AbstractMuscle.h>
#include <OpenSim/Simulation/Model/AbstractBody.h>
#include <OpenSim/Simulation/Model/AbstractCoordinate.h>
#include <OpenSim/Simulation/Model/CoordinateSet.h>
#include <OpenSim/Simulation/Model/AbstractDynamicsEngine.h>

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
MuscleViaPoint::MuscleViaPoint() :
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
MuscleViaPoint::~MuscleViaPoint()
{
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aPoint MuscleViaPoint to be copied.
 */
MuscleViaPoint::MuscleViaPoint(const MuscleViaPoint &aPoint) :
   MusclePoint(aPoint),
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
 * @return Pointer to a copy of this MuscleViaPoint.
 */
Object* MuscleViaPoint::copy() const
{
	MuscleViaPoint *pt = new MuscleViaPoint(*this);
	return(pt);
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Copy data members from one MuscleViaPoint to another.
 *
 * @param aPoint MuscleViaPoint to be copied.
 */
void MuscleViaPoint::copyData(const MuscleViaPoint &aPoint)
{
	_range = aPoint._range;
	_coordinateName = aPoint._coordinateName;
	_coordinate = aPoint._coordinate;
}

//_____________________________________________________________________________
/**
 * Set the data members of this MuscleViaPoint to their null values.
 */
void MuscleViaPoint::setNull()
{
	setType("MuscleViaPoint");
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void MuscleViaPoint::setupProperties()
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
 * Set the coordinate that this point is linked to.
 *
 * @return Whether or not this point is active.
 */
void MuscleViaPoint::setCoordinate(AbstractCoordinate& aCoordinate)
{
	if (&aCoordinate != _coordinate)
	{
	   _coordinate = &aCoordinate;
	   _coordinateName = _coordinate->getName();
		_muscle->invalidatePath();
	}
}

//_____________________________________________________________________________
/**
 * Set the range min.
 *
 * @param aRange range min to change to.
 */
void MuscleViaPoint::setRangeMin(double aMin)
{
	if (aMin <= _range[1])
	{
		_range[0] = aMin;
		_muscle->invalidatePath();
	}
}

//_____________________________________________________________________________
/**
 * Set the range max.
 *
 * @param aRange range max to change to.
 */
void MuscleViaPoint::setRangeMax(double aMax)
{
	if (aMax >= _range[0])
	{
		_range[1] = aMax;
		_muscle->invalidatePath();
	}
}

//_____________________________________________________________________________
/**
 * Determine if this point is active by checking the value of the
 * coordinate that it is linked to.
 *
 * @return Whether or not this point is active.
 */
bool MuscleViaPoint::isActive() const
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
 * @param aModel model containing this MuscleViaPoint.
 */
void MuscleViaPoint::setup(Model* aModel, AbstractMuscle* aMuscle)
{
	// base class
	MusclePoint::setup(aModel, aMuscle);

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
MuscleViaPoint& MuscleViaPoint::operator=(const MuscleViaPoint &aPoint)
{
	// BASE CLASS
	MusclePoint::operator=(aPoint);

	copyData(aPoint);

	return(*this);
}

void MuscleViaPoint::peteTest() const
{
	cout << "   MuscleViaPoint: " << getName() << endl;
	cout << "      point: " << getAttachment() << endl;
	cout << "      body: " << _body->getName() << endl;
	cout << "      range: " << getRange() << endl;
	cout << "      coordinate: " << _coordinate->getName() << endl;
}
