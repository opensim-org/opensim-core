// MusclePoint.cpp
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
#include "MusclePoint.h"
#include "BodySet.h"
#include "Model.h"
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
	_muscle = aPoint._muscle;
}

//_____________________________________________________________________________
/**
 * Set the data members of this MusclePoint to their null values.
 */
void MusclePoint::setNull()
{
	setType("MusclePoint");

	_body = NULL;
	_muscle = NULL;
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
	_attachmentProp.setAllowableArraySize(3);
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
void MusclePoint::setup(Model* aModel, AbstractMuscle* aMuscle)
{
	_muscle = aMuscle;

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

//_____________________________________________________________________________
/**
 * Set the body that this point is fixed to.
 *
 * @param aBody Reference to the body.
 */
void MusclePoint::setBody(AbstractBody& aBody, bool preserveLocation)
{
	if (&aBody == _body)
		return;

	// Preserve location means to switch bodies without changing
	// the location of the point in the inertial reference frame.
	if (preserveLocation) {
	   AbstractDynamicsEngine* engine = aBody.getDynamicsEngine();
	   if (engine)
		   engine->transformPosition(*_body, _attachment, aBody, _attachment);
	}

	_body = &aBody;
	_bodyName = aBody.getName();

	// Invalidate the path if the point and muscle have already
	// been set up.
	if (_muscle != NULL)
		_muscle->invalidatePath();
}

//_____________________________________________________________________________
/**
 * Set the XYZ attachment of the point.
 *
 * @param aAttachment The XYZ coordinates.
 */
void MusclePoint::setAttachment(double aAttachment[3])
{
	_attachment[0] = aAttachment[0];
	_attachment[1] = aAttachment[1];
	_attachment[2] = aAttachment[2];

	// Invalidate the path
	_muscle->invalidatePath();
}

//_____________________________________________________________________________
/**
 * Set the X, Y, or Z attachment of the point.
 *
 * @param aCoordIndex The coordinate to change (0=X, 1=Y, 2=Z).
 * @param aAttachment The X, Y, or Z coordinate.
 */
void MusclePoint::setAttachment(int aCoordIndex, double aAttachment)
{
	if (aCoordIndex >= 0 && aCoordIndex <= 2) {
		_attachment[aCoordIndex] = aAttachment;

		// Invalidate the path
		_muscle->invalidatePath();
	}
}

//_____________________________________________________________________________
/**
 * Get the velocity of the point in the body's local reference frame.
 *
 * @param aVelocity The velocity.
 */

void MusclePoint::getVelocity(double aVelocity[3])
{
	aVelocity[0] = aVelocity[1] = aVelocity[2] = 0.0;
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
