// SimbodyJoint.cpp
// Author: Frank C. Anderson, Peter Loan
/*
 * Copyright (c) 2007, Stanford University. All rights reserved. 
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
#include <iostream>
#include <math.h>
#include <OpenSim/Common/rdMath.h>
#include <OpenSim/Common/Mtx.h>
#include <OpenSim/Common/Function.h>
#include <OpenSim/Common/Constant.h>
#include "SimbodyJoint.h"
#include "SimbodyEngine.h"
#include <OpenSim/Common/SimmMacros.h>
#include <OpenSim/Simulation/Model/BodySet.h>

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace SimTK;
using namespace OpenSim;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */
SimbodyJoint::SimbodyJoint() :
	AbstractJoint(),
	_bodies(_bodiesProp.getValueStrArray()),
	_locationInParent(_locationInParentProp.getValueDblArray()),
	_locationInChild(_locationInChildProp.getValueDblArray()),
	_dofSetProp(PropertyObj("", DofSet())),
	_dofSet((DofSet&)_dofSetProp.getValueObj())

{
	setNull();
	setupProperties();
	updateSimbody();
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
SimbodyJoint::~SimbodyJoint()
{
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aJoint SimbodyJoint to be copied.
 */
SimbodyJoint::SimbodyJoint(const SimbodyJoint &aJoint) :
   AbstractJoint(aJoint),
	_bodies(_bodiesProp.getValueStrArray()),
	_locationInParent(_locationInParentProp.getValueDblArray()),
	_locationInChild(_locationInChildProp.getValueDblArray()),
	_dofSetProp(PropertyObj("", DofSet())),
	_dofSet((DofSet&)_dofSetProp.getValueObj())
{
	setNull();
	setupProperties();
	copyData(aJoint);
}

//_____________________________________________________________________________
/**
 * Copy this joint and return a pointer to the copy.
 * The copy constructor for this class is used.
 *
 * @return Pointer to a copy of this SimbodyJoint.
 */
Object* SimbodyJoint::copy() const
{
	SimbodyJoint *joint = new SimbodyJoint(*this);
	return(joint);
}

//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Copy data members from one SimbodyJoint to another.
 *
 * @param aJoint SimbodyJoint to be copied.
 */
void SimbodyJoint::copyData(const SimbodyJoint &aJoint)
{
	_bodies = aJoint._bodies;
	setLocationInParent(&(aJoint._locationInParent[0]));
	setLocationInChild(&(aJoint._locationInChild[0]));
	_childBody = aJoint._childBody;
	_parentBody = aJoint._parentBody;
}

//_____________________________________________________________________________
/**
 * Set the data members of this SimbodyJoint to their null values.
 */
void SimbodyJoint::setNull()
{
	setType("SimbodyJoint");
	_parentBody = NULL;
	_childBody = NULL;
	_engine = NULL;
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void SimbodyJoint::setupProperties()
{
	_bodiesProp.setName("bodies");
	_propertySet.append(&_bodiesProp);

	double origin[] = {0.0, 0.0, 0.0};
	_locationInParentProp.setName("location_in_parent");
	_locationInParentProp.setValue(3,origin);
	_propertySet.append(&_locationInParentProp);

	_locationInChildProp.setName("location_in_child");
	_locationInChildProp.setValue(3,origin);
	_propertySet.append(&_locationInChildProp);
}

//_____________________________________________________________________________
/**
 * Update the underlying SDFast parameters, such as the inboard to joint and
 * body to joint vectors.
 *
 * @return True if the new inboard to joint was set; false otherwise.
 */
void SimbodyJoint::updateSimbody()
{
	setLocationInParent(&_locationInParent[0]);
	setLocationInChild(&_locationInChild[0]);
}

//_____________________________________________________________________________
/**
 * Perform some set up functions that happen after the
 * object has been deserialized or copied.
 *
 * @param aEngine dynamics engine containing this SimbodyJoint.
 */
void SimbodyJoint::setup(AbstractDynamicsEngine* aEngine)
{
	string errorMessage;

	// Base class
	AbstractJoint::setup(aEngine);

	// Look up the parent and child bodies by name in the
	// dynamics engine and store pointers to them.
	_parentBody = dynamic_cast<SimbodyBody*>(aEngine->getBodySet()->get(_bodies[0]));
	if (!_parentBody) {
		errorMessage += "Invalid parent body (" + _bodies[0] + ") specified in joint " + getName();
		throw (Exception(errorMessage.c_str()));
	}

	_childBody = dynamic_cast<SimbodyBody*>(aEngine->getBodySet()->get(_bodies[1]));
	if (!_childBody) {
		errorMessage += "Invalid child body (" + _bodies[0] + ") specified in joint " + getName();
		throw (Exception(errorMessage.c_str()));
	}

	_engine = dynamic_cast<SimbodyEngine*>(aEngine);
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
SimbodyJoint& SimbodyJoint::operator=(const SimbodyJoint &aJoint)
{
	AbstractJoint::operator=(aJoint);
	copyData(aJoint);
	return(*this);
}


//=============================================================================
// GET AND SET
//=============================================================================
//-----------------------------------------------------------------------------
// LOCATION IN PARENT
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the location of this joint in its parent body.  This method
 * updates the underlying SDFast inboard to joint vector.
 *
 * @param aLocation New location specified in the parent body frame.
 */
void SimbodyJoint::setLocationInParent(const double aLocation[3])
{
	// Update Simbody
	if(_parentBody!=NULL) {
		// TODO:  Find out what needs to be done to update Simbody.
	}

	// Update property
	for(int i=0; i<3; i++) _locationInParent[i] = aLocation[i];
}
//_____________________________________________________________________________
/**
 * Get the location of this joint in its parent body.
 *
 * @param rLocation Currnt location specified in the parent body frame.
 */
void SimbodyJoint::getLocationInParent(double rLocation[3]) const
{
	Mtx::Assign(1,3,&_locationInParent[0],rLocation);
}

//-----------------------------------------------------------------------------
// LOCATION IN CHILD
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the location of this joint in its child body.  This method
 * updates the underlying SDFast body to joint vector.
 *
 * @param aLocation New location specified in the child body frame.
 */
void SimbodyJoint::setLocationInChild(const double aLocation[3])
{
	if(_childBody!=NULL) {
		// TODO:  Find out what needs to be done to update Simbody.
	}

	// UPDATE PROPERTY
	for(int i=0; i<3; i++) _locationInChild[i] = aLocation[i];
}
//_____________________________________________________________________________
/**
 * Get the location of this joint in its child body.
 *
 * @param rLocation Current location specified in the child body frame.
 */
void SimbodyJoint::getLocationInChild(double rLocation[3]) const
{
	Mtx::Assign(1,3,&_locationInChild[0],rLocation);
}


//_____________________________________________________________________________
/**
 * Get the SimbodyJoint's forward transform.
 *
 * @return Reference to the forward transform.
 */
const OpenSim::Transform& SimbodyJoint::getForwardTransform()
{

	return _forwardTransform;
}

//_____________________________________________________________________________
/**
 * Get the SimbodyJoint's inverse transform.
 *
 * @return Reference to the inverse transform.
 */
const OpenSim::Transform& SimbodyJoint::getInverseTransform()
{

	return _inverseTransform;
}

//_____________________________________________________________________________
/**
 * Set the name of the joint's parent body
 *
 * @param Name of the parent body.
 */
void SimbodyJoint::setParentBodyName(const string& aName)
{
	_bodies.set(0, aName);
}

//_____________________________________________________________________________
/**
 * Set the name of the joint's child body
 *
 * @param Name of the child body.
 */
void SimbodyJoint::setChildBodyName(const string& aName)
{
	_bodies.set(1, aName);
}



//=============================================================================
// UTILITY
//=============================================================================
bool SimbodyJoint::hasXYZAxes() const
{
   return true;
}

bool SimbodyJoint::isTreeJoint() const
{
	return true;
}


//=============================================================================
// SCALING
//=============================================================================
//_____________________________________________________________________________
/**
 * Scale a joint based on XYZ scale factors for the bodies.
 *
 * @param aScaleSet Set of XYZ scale factors for the bodies.
 */
void SimbodyJoint::scale(const ScaleSet& aScaleSet)
{
	Array<double> scaleFactors(1.0, 3);

	// SCALING TO DO WITH THE PARENT BODY -----
	// Joint kinematics are scaled by the scale factors for the
	// parent body, so get those body's factors
	const string& parentName = getParentBody()->getName();
	// Get scale factors
	for (int i=0; i<aScaleSet.getSize(); i++) {
		Scale *scale = aScaleSet.get(i);
		if (scale->getSegmentName() == parentName) {
			scale->getScaleFactors(scaleFactors);
			break;
		}
	}

	// If all three factors are equal to 1.0, do nothing.
	if (EQUAL_WITHIN_ERROR(scaleFactors[0], 1.0) &&
		 EQUAL_WITHIN_ERROR(scaleFactors[1], 1.0) &&
		 EQUAL_WITHIN_ERROR(scaleFactors[2], 1.0))
		 return;

	// Location in parent
	double scaledLocationInParent[3];
	for(int i=0; i<3; i++) scaledLocationInParent[i] = scaleFactors[i] * _locationInParent[i];
	setLocationInParent(scaledLocationInParent);


	// SCALING TO DO WITH THE CHILD BODY -----
	const string& childName = getChildBody()->getName();
	// Get scale factors
	for (int i=0; i<aScaleSet.getSize(); i++) {
		Scale *scale = aScaleSet.get(i);
		if (scale->getSegmentName() == childName) {
			scale->getScaleFactors(scaleFactors);
			break;
		}
	}

	// If all three factors are equal to 1.0, do nothing.
	if (EQUAL_WITHIN_ERROR(scaleFactors[0], 1.0) &&
		 EQUAL_WITHIN_ERROR(scaleFactors[1], 1.0) &&
		 EQUAL_WITHIN_ERROR(scaleFactors[2], 1.0))
		 return;

	// Location in child
	double scaledLocationInChild[3];
	for(int i=0; i<3; i++) scaledLocationInChild[i] = scaleFactors[i] * _locationInChild[i];
	setLocationInChild(scaledLocationInChild);
}

