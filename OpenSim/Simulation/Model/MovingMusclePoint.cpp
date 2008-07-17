// MovingMusclePoint.cpp
// Author: Peter Loan
/*
 * Copyright (c)  2006, Stanford University. All rights reserved. 
* Use of the OpenSim software in source form is permitted provided that the following
* conditions are met:
* 	1. The software is used only for non-commercial research and education. It may not
*     be used in relation to any commercial activity.
* 	2. The software is not distributed or redistributed.  Software distribution is allowed 
*     only through https://simtk.org/home/opensim.
* 	3. Use of the OpenSim software or derivatives must be acknowledged in all publications,
*      presentations, or documents describing work in which OpenSim or derivatives are used.
* 	4. Credits to developers may not be removed from executables
*     created from modifications of the source.
* 	5. Modifications of source code must retain the above copyright notice, this list of
*     conditions and the following disclaimer. 
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
*  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
*  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
*  SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
*  TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
*  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR BUSINESS INTERRUPTION) OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
*  WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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
#include <OpenSim/Common/NatCubicSpline.h>

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace OpenSim;
using SimTK::Vec3;

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
 * Initialize a MovingMusclePoint with data from a MusclePoint.
 *
 * @param aPoint MusclePoint to be copied.
 */
void MovingMusclePoint::init(const MusclePoint& aPoint)
{
	MusclePoint::copyData(aPoint);

	// If aPoint is a MovingMusclePoint, then you can copy all of its members over.
	// Otherwise, create new functions for X, Y, and Z so that the point starts
	// out in a reasonable state.
   const MovingMusclePoint* mmp = dynamic_cast<const MovingMusclePoint*>(&aPoint);
	if (mmp) {
		copyData(*mmp);
	} else {
		double x[2], y[2];
		x[0] = 0.0;
		x[1] = 1.0;
		y[0] = y[1] = aPoint.getAttachment()[0];
		_xAttachment = new NatCubicSpline(2, x, y);
		y[0] = y[1] = aPoint.getAttachment()[1];
		_yAttachment = new NatCubicSpline(2, x, y);
		y[0] = y[1] = aPoint.getAttachment()[2];
		_zAttachment = new NatCubicSpline(2, x, y);
	}
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
 * @param aCoordinate the coordinate to set to.
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
 * @param aCoordinate the coordinate to set to.
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
 * @param aCoordinate the coordinate to set to.
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
 * Set the function used for the X attachment.
 *
 * @param aFunction the function to set to.
 */
void MovingMusclePoint::setXFunction(Function& aFunction)
{
	_xAttachment = &aFunction;

	// Invalidate the path
	_muscle->invalidatePath();
}

//_____________________________________________________________________________
/**
 * Set the function used for the Y attachment.
 *
 * @param aFunction the function to set to.
 */
void MovingMusclePoint::setYFunction(Function& aFunction)
{
	_yAttachment = &aFunction;

	// Invalidate the path
	_muscle->invalidatePath();
}

//_____________________________________________________________________________
/**
 * Set the function used for the Z attachment.
 *
 * @param aFunction the function to set to.
 */
void MovingMusclePoint::setZFunction(Function& aFunction)
{
	_zAttachment = &aFunction;

	// Invalidate the path
	_muscle->invalidatePath();
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
	else // type == Constant
		_attachment[0] = _xAttachment->evaluate(0, 0.0, 0.0, 0.0);

	if (_yCoordinate)
		_attachment[1] = _yAttachment->evaluate(0, _yCoordinate->getValue(), 0.0, 0.0);
	else // type == Constant
		_attachment[1] = _yAttachment->evaluate(0, 0.0, 0.0, 0.0);

	if (_zCoordinate)
		_attachment[2] = _zAttachment->evaluate(0, _zCoordinate->getValue(), 0.0, 0.0);
	else // type == Constant
		_attachment[2] = _zAttachment->evaluate(0, 0.0, 0.0, 0.0);
}

//_____________________________________________________________________________
/**
 * Get the velocity of the point in the body's local reference frame.
 *
 * @param aVelocity The velocity.
 */

void MovingMusclePoint::getVelocity(SimTK::Vec3& aVelocity)
{
    // Get the generalized speed associated with moving muscle point
	AbstractSpeed* speed;
	double speedValue;
	
	SpeedSet *speeds = _body->getDynamicsEngine()->getSpeedSet();

	if (_xCoordinate){
		speed = speeds->get(AbstractSpeed::getSpeedName(_xCoordinateName));
		if (speed) speedValue = speed->getValue();
		else speedValue = 0.0;
		//Multiply the partial (derivative of point coordinate w.r.t. gencoord) by genspeed
		aVelocity[0] = _xAttachment->evaluate(1, _xCoordinate->getValue(), 0.0, 0.0)*speedValue;
	}
	else
		aVelocity[0] = 0.0;

	if (_yCoordinate){
		speed = speeds->get(AbstractSpeed::getSpeedName(_yCoordinateName));
		if (speed) speedValue = speed->getValue();
		else speedValue = 0.0;
		//Multiply the partial (derivative of point coordinate w.r.t. gencoord) by genspeed
		aVelocity[1] = _yAttachment->evaluate(1, _yCoordinate->getValue(), 0.0, 0.0)*speedValue;
	}
	else
		aVelocity[1] = 0.0;

	if (_zCoordinate){
		speed = speeds->get(AbstractSpeed::getSpeedName(_zCoordinateName));
		if (speed) speedValue = speed->getValue();
		else speedValue = 0.0;
		//Multiply the partial (derivative of point coordinate w.r.t. gencoord) by genspeed
		aVelocity[2] = _zAttachment->evaluate(1, _zCoordinate->getValue(), 0.0, 0.0)*speedValue;
	}
	else
		aVelocity[2] = 0.0;
}

void MovingMusclePoint::scale(const SimTK::Vec3& aScaleFactors)
{
	if (_xCoordinate)
		_xAttachment->scaleY(aScaleFactors[0]);

	if (_yCoordinate)
		_yAttachment->scaleY(aScaleFactors[1]);

	if (_zCoordinate)
		 _zAttachment->scaleY(aScaleFactors[2]);

	updateGeometry();
}