// MovingMusclePoint.cpp
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
#include "MovingMusclePoint.h"
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/AbstractMuscle.h>
#include <OpenSim/Simulation/Model/AbstractBody.h>
#include <OpenSim/Simulation/Model/AbstractCoordinate.h>
#include <OpenSim/Simulation/Model/CoordinateSet.h>
#include <OpenSim/Simulation/Model/SpeedSet.h>
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
MovingMusclePoint::MovingMusclePoint() :
   _XAttachment(_XAttachmentProp.getValueObjPtrRef()),
	_XCoordinateName(_XCoordinateNameProp.getValueStr()),
   _YAttachment(_YAttachmentProp.getValueObjPtrRef()),
	_YCoordinateName(_YCoordinateNameProp.getValueStr()),
   _ZAttachment(_ZAttachmentProp.getValueObjPtrRef()),
	_ZCoordinateName(_ZCoordinateNameProp.getValueStr())
{
	setNull();
	setupProperties();
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
MovingMusclePoint::~MovingMusclePoint()
{
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aPoint MovingMusclePoint to be copied.
 */
MovingMusclePoint::MovingMusclePoint(const MovingMusclePoint &aPoint) :
   MusclePoint(aPoint),
   _XAttachment(_XAttachmentProp.getValueObjPtrRef()),
	_XCoordinateName(_XCoordinateNameProp.getValueStr()),
   _YAttachment(_YAttachmentProp.getValueObjPtrRef()),
	_YCoordinateName(_YCoordinateNameProp.getValueStr()),
   _ZAttachment(_ZAttachmentProp.getValueObjPtrRef()),
	_ZCoordinateName(_ZCoordinateNameProp.getValueStr())
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
 * @return Pointer to a copy of this MovingMusclePoint.
 */
Object* MovingMusclePoint::copy() const
{
	MovingMusclePoint *pt = new MovingMusclePoint(*this);
	return(pt);
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Copy data members from one MovingMusclePoint to another.
 *
 * @param aPoint MovingMusclePoint to be copied.
 */
void MovingMusclePoint::copyData(const MovingMusclePoint &aPoint)
{
	_XAttachment = aPoint._XAttachment;
	_XCoordinateName = aPoint._XCoordinateName;
	_YAttachment = aPoint._YAttachment;
	_YCoordinateName = aPoint._YCoordinateName;
	_ZAttachment = aPoint._ZAttachment;
	_ZCoordinateName = aPoint._ZCoordinateName;
}

//_____________________________________________________________________________
/**
 * Set the data members of this MovingMusclePoint to their null values.
 */
void MovingMusclePoint::setNull()
{
	setType("MovingMusclePoint");

	_XCoordinate = NULL;
	_YCoordinate = NULL;
	_ZCoordinate = NULL;
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void MovingMusclePoint::setupProperties()
{
	_XAttachmentProp.setName("X_attachment");
	_propertySet.append(&_XAttachmentProp);

	_XCoordinateNameProp.setName("X_coordinate");
	_propertySet.append(&_XCoordinateNameProp);

	_YAttachmentProp.setName("Y_attachment");
	_propertySet.append(&_YAttachmentProp);

	_YCoordinateNameProp.setName("Y_coordinate");
	_propertySet.append(&_YCoordinateNameProp);

	_ZAttachmentProp.setName("Z_attachment");
	_propertySet.append(&_ZAttachmentProp);

	_ZCoordinateNameProp.setName("Z_coordinate");
	_propertySet.append(&_ZCoordinateNameProp);
}

//_____________________________________________________________________________
/**
 * Set the coordinate used for the X attachment function.
 *
 * @param The coordinate to set to.
 */
void MovingMusclePoint::setXCoordinate(AbstractCoordinate& aCoordinate)
{
	if (&aCoordinate != _XCoordinate)
	{
	   _XCoordinate = &aCoordinate;
	   _XCoordinateName = _XCoordinate->getName();
		_muscle->invalidatePath();
	}
}

//_____________________________________________________________________________
/**
 * Set the coordinate used for the Y attachment function.
 *
 * @param The coordinate to set to.
 */
void MovingMusclePoint::setYCoordinate(AbstractCoordinate& aCoordinate)
{
	if (&aCoordinate != _YCoordinate)
	{
	   _YCoordinate = &aCoordinate;
	   _YCoordinateName = _YCoordinate->getName();
		_muscle->invalidatePath();
	}
}

//_____________________________________________________________________________
/**
 * Set the coordinate used for the Z attachment function.
 *
 * @param The coordinate to set to.
 */
void MovingMusclePoint::setZCoordinate(AbstractCoordinate& aCoordinate)
{
	if (&aCoordinate != _ZCoordinate)
	{
	   _ZCoordinate = &aCoordinate;
	   _ZCoordinateName = _ZCoordinate->getName();
		_muscle->invalidatePath();
	}
}

//_____________________________________________________________________________
/**
 * Perform some set up functions that happen after the
 * object has been deserialized or copied.
 *
 * @param aModel model containing this MovingMusclePoint.
 */
void MovingMusclePoint::setup(Model* aModel, AbstractMuscle* aMuscle)
{
	// base class
	MusclePoint::setup(aModel, aMuscle);

	// Look up the coordinates by name in the dynamics engine and
	// store pointers to them.
	_XCoordinate = aModel->getDynamicsEngine().getCoordinateSet()->get(_XCoordinateName);
	_YCoordinate = aModel->getDynamicsEngine().getCoordinateSet()->get(_YCoordinateName);
	_ZCoordinate = aModel->getDynamicsEngine().getCoordinateSet()->get(_ZCoordinateName);

	// Update the XYZ location of the point (stored in _attachment).
	update();
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
MovingMusclePoint& MovingMusclePoint::operator=(const MovingMusclePoint &aPoint)
{
	// BASE CLASS
	MusclePoint::operator=(aPoint);

	copyData(aPoint);

	return(*this);
}

//_____________________________________________________________________________
/**
 * Update the point's location.
 *
 */
void MovingMusclePoint::update()
{
	if (_XCoordinate)
		_attachment[0] = _XAttachment->evaluate(0, _XCoordinate->getValue(), 0.0, 0.0);

	if (_YCoordinate)
		_attachment[1] = _YAttachment->evaluate(0, _YCoordinate->getValue(), 0.0, 0.0);

	if (_ZCoordinate)
		_attachment[2] = _ZAttachment->evaluate(0, _ZCoordinate->getValue(), 0.0, 0.0);
}

//_____________________________________________________________________________
/**
 * Get the velocity of the point in the body's local reference frame.
 *
 * @param aVelocity The velocity.
 */

void MovingMusclePoint::getVelocity(double aVelocity[3])
{
    // Get the generalized speed associated with moving muscle point
	double speed;
	
	SpeedSet *speeds = _body->getDynamicsEngine()->getSpeedSet();

	if (_XCoordinate){
		speed = speeds->get(AbstractSpeed::getSpeedName(_XCoordinateName))->getValue();
		//Multiply the partial (derivative of point coordinate w.r.t. gencoord) by genspeed
		aVelocity[0] = _XAttachment->evaluate(1, _XCoordinate->getValue(), 0.0, 0.0)*speed;
	}
	else
		aVelocity[0] = 0.0;

	if (_YCoordinate){
		speed = speeds->get(AbstractSpeed::getSpeedName(_YCoordinateName))->getValue();
		//Multiply the partial (derivative of point coordinate w.r.t. gencoord) by genspeed
		aVelocity[1] = _YAttachment->evaluate(1, _YCoordinate->getValue(), 0.0, 0.0)*speed;
	}
	else
		aVelocity[1] = 0.0;

	if (_ZCoordinate){
		speed = speeds->get(AbstractSpeed::getSpeedName(_ZCoordinateName))->getValue();
		//Multiply the partial (derivative of point coordinate w.r.t. gencoord) by genspeed
		aVelocity[2] = _ZAttachment->evaluate(1, _ZCoordinate->getValue(), 0.0, 0.0);
	}
	else
		aVelocity[2] = 0.0;
}

void MovingMusclePoint::scale(Array<double>& aScaleFactors)
{
	if (_XCoordinate)
		_XAttachment->scaleY(aScaleFactors[0]);

	if (_YCoordinate)
		_YAttachment->scaleY(aScaleFactors[1]);

	if (_ZCoordinate)
		 _ZAttachment->scaleY(aScaleFactors[2]);

	updateGeometry();
}