// MusclePoint.cpp
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
#include "MusclePoint.h"
#include "BodySet.h"
#include "AbstractModel.h"
#include "AbstractMuscle.h"
#include "AbstractDynamicsEngine.h"
#include <OpenSim/Simulation/Wrap/AbstractWrapObject.h>

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace OpenSim;

Geometry *MusclePoint::_defaultGeometry= AnalyticSphere::createSphere(0.005);

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */
MusclePoint::MusclePoint() :
   _attachment(_attachmentProp.getValueDblArray()),
	_bodyName(_bodyNameProp.getValueStr()),
	_displayerProp(PropertyObj("", VisibleObject())),
   _displayer((VisibleObject&)_displayerProp.getValueObj())
{
	setNull();
	setupProperties();
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
MusclePoint::~MusclePoint()
{
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aPoint MusclePoint to be copied.
 */
MusclePoint::MusclePoint(const MusclePoint &aPoint) :
   Object(aPoint),
   _attachment(_attachmentProp.getValueDblArray()),
	_bodyName(_bodyNameProp.getValueStr()),
	_displayerProp(PropertyObj("", VisibleObject())),
   _displayer((VisibleObject&)_displayerProp.getValueObj())
{
	setNull();
	setupProperties();
	copyData(aPoint);
}

//_____________________________________________________________________________
/**
 * Copy this muscle point and return a pointer to the copy.
 * The copy constructor for this class is used.
 *
 * @return Pointer to a copy of this MusclePoint.
 */
Object* MusclePoint::copy() const
{
	MusclePoint *pt = new MusclePoint(*this);
	return(pt);
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Copy data members from one MusclePoint to another.
 *
 * @param aPoint MusclePoint to be copied.
 */
void MusclePoint::copyData(const MusclePoint &aPoint)
{
	_attachment = aPoint._attachment;
	_displayer = aPoint._displayer;
	_bodyName = aPoint._bodyName;
	_body = aPoint._body;
}

//_____________________________________________________________________________
/**
 * Set the data members of this MusclePoint to their null values.
 */
void MusclePoint::setNull()
{
	setType("MusclePoint");

	_body = NULL;
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void MusclePoint::setupProperties()
{
	const double defaultAttachment[] = {0.0, 0.0, 0.0};
	_attachmentProp.setName("location");
	_attachmentProp.setValue(3, defaultAttachment);
	_propertySet.append(&_attachmentProp);

	_displayerProp.setName("display");
	_propertySet.append(&_displayerProp);

	_bodyNameProp.setName("body");
	_propertySet.append(&_bodyNameProp);
}

//_____________________________________________________________________________
/**
 * Perform some set up functions that happen after the
 * object has been deserialized or copied.
 *
 * @param aModel model containing this MusclePoint.
 */
void MusclePoint::setup(AbstractModel* aModel, AbstractMuscle* aMuscle)
{
	// Look up the body by name in the kinematics engine and
	// store a pointer to it.
	_body = aModel->getDynamicsEngine().getBodySet()->get(_bodyName);

	if (!_body)
	{
		string errorMessage = "Body " + _bodyName + " referenced in muscle " + aMuscle->getName() + " not found in model " + aModel->getName();
		throw Exception(errorMessage);
	}

	// _displayer.setOwner(this);
	// Muscle points depend on body
	// A displayer may not be needed altogether.
	// - Removing the dependency since muscle points now display as part of the
	// - muscle itself, extracted directly from the set of line segments
	// - representing the muscle path. -Ayman 02/07
	//
	//
	//_body->getDisplayer()->addDependent(&_displayer);
	//_displayer.addGeometry(_defaultGeometry);
	// 
	// Transform position;
	// position.translate(_attachment.get());
	// getDisplayer()->setTransform(position);
	// double defaultColor[3] = { 1.0, 0.0, 0.0 };
	// _displayer.getVisibleProperties().setColor(defaultColor);
}
//_____________________________________________________________________________
/**
 * Update geometry of the muscle point.
 *
 */
void MusclePoint::updateGeometry()
{
	Transform position;
	position.translate(_attachment.get());
	getDisplayer()->setTransform(position);
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
MusclePoint& MusclePoint::operator=(const MusclePoint &aPoint)
{
	// BASE CLASS
	Object::operator=(aPoint);

	copyData(aPoint);

	return(*this);
}

void MusclePoint::setBody(AbstractBody& aBody)
{
	_body = &aBody;
	_bodyName = aBody.getName();
}

void MusclePoint::setAttachment(double aAttachment[3])
{
	_attachment[0] = aAttachment[0];
	_attachment[1] = aAttachment[1];
	_attachment[2] = aAttachment[2];
}

//=============================================================================
// SCALING
//=============================================================================
//_____________________________________________________________________________
/**
 * Scale the muscle point.
 *
 * @param aScaleFactors the XYZ scale factors to scale the point by.
 */
void MusclePoint::scale(Array<double>& aScaleFactors)
{
	for (int i = 0; i < 3; i++)
		_attachment[i] *= aScaleFactors[i];

	updateGeometry();
}

void MusclePoint::peteTest() const
{
	cout << "   MusclePoint: " << getName() << endl;
	cout << "      point: " << getAttachment() << endl;
	if (_body)
		cout << "      body: " << _body->getName() << endl;
}
