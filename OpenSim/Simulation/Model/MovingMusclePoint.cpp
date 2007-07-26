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
   _xAttachment(_xAttachmentProp.getValueObjPtrRef()),
	_xCoordinateName(_xCoordinateNameProp.getValueStr()),
   _yAttachment(_yAttachmentProp.getValueObjPtrRef()),
	_yCoordinateName(_yCoordinateNameProp.getValueStr()),
   _zAttachment(_zAttachmentProp.getValueObjPtrRef()),
	_zCoordinateName(_zCoordinateNameProp.getValueStr())
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
   _xAttachment(_xAttachmentProp.getValueObjPtrRef()),
	_xCoordinateName(_xCoordinateNameProp.getValueStr()),
   _yAttachment(_yAttachmentProp.getValueObjPtrRef()),
	_yCoordinateName(_yCoordinateNameProp.getValueStr()),
   _zAttachment(_zAttachmentProp.getValueObjPtrRef()),
	_zCoordinateName(_zCoordinateNameProp.getValueStr())
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
	_xAttachment = (Function*)Object::SafeCopy(aPoint._xAttachment);
	_xCoordinateName = aPoint._xCoordinateName;
	_yAttachment = (Function*)Object::SafeCopy(aPoint._yAttachment);
	_yCoordinateName = aPoint._yCoordinateName;
	_zAttachment = (Function*)Object::SafeCopy(aPoint._zAttachment);
	_zCoordinateName = aPoint._zCoordinateName;
}

//_____________________________________________________________________________
/**
 * Set the data members of this MovingMusclePoint to their null values.
 */
void MovingMusclePoint::setNull()
{
	setType("MovingMusclePoint");

	_xCoordinate = NULL;
	_yCoordinate = NULL;
	_zCoordinate = NULL;
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void MovingMusclePoint::setupProperties()
{
	_xAttachmentProp.setName("XAttachment");
	_propertySet.append(&_xAttachmentProp);

	_xCoordinateNameProp.setName("x_coordinate");
	_propertySet.append(&_xCoordinateNameProp);

	_yAttachmentProp.setName("YAttachment");
	_propertySet.append(&_yAttachmentProp);

	_yCoordinateNameProp.setName("y_coordinate");
	_propertySet.append(&_yCoordinateNameProp);

	_zAttachmentProp.setName("ZAttachment");
	_propertySet.append(&_zAttachmentProp);

	_zCoordinateNameProp.setName("z_coordinate");
	_propertySet.append(&_zCoordinateNameProp);
}

//_____________________________________________________________________________
/**
 * Set the coordinate used for the X attachment function.
 *
 * @param The coordinate to set to.
 */
void MovingMusclePoint::setXCoordinate(AbstractCoordinate& aCoordinate)
{
	if (&aCoordinate != _xCoordinate) {
	   _xCoordinate = &aCoordinate;
	   _xCoordinateName = _xCoordinate->getName();
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
	if (&aCoordinate != _yCoordinate) {
	   _yCoordinate = &aCoordinate;
	   _yCoordinateName = _yCoordinate->getName();
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
	if (&aCoordinate != _zCoordinate) {
	   _zCoordinate = &aCoordinate;
	   _zCoordinateName = _zCoordinate->getName();
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
	_xCoordinate = aModel->getDynamicsEngine().getCoordinateSet()->get(_xCoordinateName);
	_yCoordinate = aModel->getDynamicsEngine().getCoordinateSet()->get(_yCoordinateName);
	_zCoordinate = aModel->getDynamicsEngine().getCoordinateSet()->get(_zCoordinateName);

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
	if (_xCoordinate)
		_attachment[0] = _xAttachment->evaluate(0, _xCoordinate->getValue(), 0.0, 0.0);

	if (_yCoordinate)
		_attachment[1] = _yAttachment->evaluate(0, _yCoordinate->getValue(), 0.0, 0.0);

	if (_zCoordinate)
		_attachment[2] = _zAttachment->evaluate(0, _zCoordinate->getValue(), 0.0, 0.0);
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

	if (_xCoordinate){
		speed = speeds->get(AbstractSpeed::getSpeedName(_xCoordinateName))->getValue();
		//Multiply the partial (derivative of point coordinate w.r.t. gencoord) by genspeed
		aVelocity[0] = _xAttachment->evaluate(1, _xCoordinate->getValue(), 0.0, 0.0)*speed;
	}
	else
		aVelocity[0] = 0.0;

	if (_yCoordinate){
		speed = speeds->get(AbstractSpeed::getSpeedName(_yCoordinateName))->getValue();
		//Multiply the partial (derivative of point coordinate w.r.t. gencoord) by genspeed
		aVelocity[1] = _yAttachment->evaluate(1, _yCoordinate->getValue(), 0.0, 0.0)*speed;
	}
	else
		aVelocity[1] = 0.0;

	if (_zCoordinate){
		speed = speeds->get(AbstractSpeed::getSpeedName(_zCoordinateName))->getValue();
		//Multiply the partial (derivative of point coordinate w.r.t. gencoord) by genspeed
		aVelocity[2] = _zAttachment->evaluate(1, _zCoordinate->getValue(), 0.0, 0.0);
	}
	else
		aVelocity[2] = 0.0;
}

void MovingMusclePoint::scale(Array<double>& aScaleFactors)
{
	if (_xCoordinate)
		_xAttachment->scaleY(aScaleFactors[0]);

	if (_yCoordinate)
		_yAttachment->scaleY(aScaleFactors[1]);

	if (_zCoordinate)
		 _zAttachment->scaleY(aScaleFactors[2]);

	updateGeometry();
}