// SdfastJoint.cpp
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
#include <iostream>
#include <math.h>
#include <OpenSim/Tools/rdMath.h>
#include <OpenSim/Tools/Mtx.h>
#include <OpenSim/Tools/Function.h>
#include <OpenSim/Tools/Constant.h>
#include "SdfastJoint.h"
#include "SdfastEngine.h"
#include <OpenSim/Simulation/SIMM/SimmMacros.h>
#include <OpenSim/Simulation/SIMM/BodySet.h>

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
SdfastJoint::SdfastJoint() :
	AbstractJoint(),
	_bodies(_bodiesProp.getValueStrArray()),
	_locationInParent(_locationInParentProp.getValueDblArray()),
	_locationInChild(_locationInChildProp.getValueDblArray()),
	_childBody(NULL),
	_parentBody(NULL),
	_index(_indexProp.getValueInt()),
	_SdfastTypeName(_SdfastTypeNameProp.getValueStr())
{
	setNull();
	setupProperties();
	updateSdfast();
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
SdfastJoint::~SdfastJoint()
{
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aJoint SdfastJoint to be copied.
 */
SdfastJoint::SdfastJoint(const SdfastJoint &aJoint) :
   AbstractJoint(aJoint),
	_bodies(_bodiesProp.getValueStrArray()),
	_locationInParent(_locationInParentProp.getValueDblArray()),
	_locationInChild(_locationInChildProp.getValueDblArray()),
	_childBody(NULL),
	_parentBody(NULL),
	_index(_indexProp.getValueInt()),
	_SdfastTypeName(_SdfastTypeNameProp.getValueStr())
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
 * @return Pointer to a copy of this SdfastJoint.
 */
Object* SdfastJoint::copy() const
{
	SdfastJoint *joint = new SdfastJoint(*this);
	return(joint);
}

//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Copy data members from one SdfastJoint to another.
 *
 * @param aJoint SdfastJoint to be copied.
 */
void SdfastJoint::copyData(const SdfastJoint &aJoint)
{
	_bodies = aJoint._bodies;
	setLocationInParent(&(aJoint._locationInParent[0]));
	setLocationInChild(&(aJoint._locationInChild[0]));
	_childBody = aJoint._childBody;
	_parentBody = aJoint._parentBody;
	_index = aJoint._index;
	_SdfastTypeName = aJoint._SdfastTypeName;
}

//_____________________________________________________________________________
/**
 * Set the data members of this SdfastJoint to their null values.
 */
void SdfastJoint::setNull()
{
	setType("SdfastJoint");
	_parentBody = NULL;
	_childBody = NULL;
	_SdfastEngine = NULL;
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void SdfastJoint::setupProperties()
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

	_indexProp.setName("index");
	_propertySet.append(&_indexProp);

	_SdfastTypeNameProp.setName("sdfast_type");
	_propertySet.append(&_SdfastTypeNameProp);
}

//_____________________________________________________________________________
/**
 * Update the underlying SDFast parameters, such as the inboard to joint and
 * body to joint vectors.
 *
 * @return True if the new inboard to joint was set; false otherwise.
 */
void SdfastJoint::updateSdfast()
{
	setLocationInParent(&_locationInParent[0]);
	setLocationInChild(&_locationInChild[0]);
}

//_____________________________________________________________________________
/**
 * Perform some set up functions that happen after the
 * object has been deserialized or copied.
 *
 * @param aEngine dynamics engine containing this SdfastJoint.
 */
void SdfastJoint::setup(AbstractDynamicsEngine* aEngine)
{
	string errorMessage;

	// Base class
	AbstractJoint::setup(aEngine);

	/* Look up the parent and child bodies by name in the
	 * dynamics engine and store pointers to them.
	 */
	_parentBody = dynamic_cast<SdfastBody*>(aEngine->getBodySet()->get(_bodies[0]));
	if (!_parentBody)
	{
		errorMessage += "Invalid parent body (" + _bodies[0] + ") specified in joint " + getName();
		throw (Exception(errorMessage.c_str()));
	}

	_childBody = dynamic_cast<SdfastBody*>(aEngine->getBodySet()->get(_bodies[1]));
	if (!_childBody)
	{
		errorMessage += "Invalid child body (" + _bodies[0] + ") specified in joint " + getName();
		throw (Exception(errorMessage.c_str()));
	}

	_SdfastEngine = dynamic_cast<SdfastEngine*>(aEngine);
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
SdfastJoint& SdfastJoint::operator=(const SdfastJoint &aJoint)
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
void SdfastJoint::setLocationInParent(const double aLocation[3])
{
	if(_parentBody!=NULL) {
		// COMPUTE NEW InboardToJoint (itj)
		double com[3],itjNew[3];
		_parentBody->getMassCenter(com);
		Mtx::Subtract(1,3,&aLocation[0],com,itjNew);

		// COMPARE TO CURRENT
		bool different = false;
		double itjCurrent[3];
		sdgetitj(_index,itjCurrent);
		for(int i=0; i<3; i++) {
			if(!rdMath::IsEqual(itjNew[i],itjCurrent[i],rdMath::ZERO)) different = true;
		}
		if(!different) return;

		// SET NEW ITJ
		sditj(_index,itjNew);
		sdinit();

		// TODO: check for SDFast errors.
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
void SdfastJoint::getLocationInParent(double rLocation[3]) const
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
void SdfastJoint::setLocationInChild(const double aLocation[3])
{
	if(_childBody!=NULL) {
		// COMPUTE NEW BodyToJoint (btj)
		double com[3],btjNew[3];
		_childBody->getMassCenter(com);
		Mtx::Subtract(1,3,aLocation,com,btjNew);

		// COMPARE TO CURRENT
		double btjCurrent[3];
		sdgetbtj(_index,btjCurrent);
		bool different = false;
		for(int i=0; i<3; i++) {
			if(!rdMath::IsEqual(btjNew[i],btjCurrent[i],rdMath::ZERO)) different=true;
		}
		if(!different) return;

		// SET NEW ITJ
		sdbtj(_index,btjNew);
		sdinit();

		// TODO: check for SDFast errors.
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
void SdfastJoint::getLocationInChild(double rLocation[3]) const
{
	Mtx::Assign(1,3,&_locationInChild[0],rLocation);
}



//_____________________________________________________________________________
/**
 * Get the SdfastJoint's forward transform.
 *
 * @return Reference to the forward transform.
 */
const Transform& SdfastJoint::getForwardTransform()
{
	int i;
	double parentDirCos[3][3], parentOrigin[3], parentOriginGrnd[3];
	double childDirCos[3][3], childOrigin[3], childOriginGrnd[3];

	// Get the mass centers of the parent and child bodies, and negate them.
	// This gives you the coordinates of the OpenSim body's origin in the
	// reference frame of the SD/FAST body.
	// Q(from Clay): Do you want the coordinates of the body's origin or
	// the coordinates of the joint?
	_parentBody->getMassCenter(parentOrigin);
	_childBody->getMassCenter(childOrigin);
	for (i = 0; i < 3; i++)
	{
		parentOrigin[i] = -parentOrigin[i];
		childOrigin[i] = -childOrigin[i];
	}
	sdpos(_parentBody->getSdfastIndex(), parentOrigin, parentOriginGrnd);
	sdpos(_childBody->getSdfastIndex(), childOrigin, childOriginGrnd);

	// Now get the direction cosine matrices for the bodies.
	sdorient(_parentBody->getSdfastIndex(), parentDirCos);
	sdorient(_childBody->getSdfastIndex(), childDirCos);

	// Now copy the translations and orientations into transforms.
	// 'parent' is the transform from ground to the parent frame.
	// 'child' is the transform from ground to the child frame.
	Transform parent;
	double* parentMatrix = parent.getMatrix();
	parentMatrix[0] = parentDirCos[0][0];
	parentMatrix[1] = parentDirCos[0][1];
	parentMatrix[2] = parentDirCos[0][2];
	parentMatrix[4] = parentDirCos[1][0];
	parentMatrix[5] = parentDirCos[1][1];
	parentMatrix[6] = parentDirCos[1][2];
	parentMatrix[8] = parentDirCos[2][0];
	parentMatrix[9] = parentDirCos[2][1];
	parentMatrix[10] = parentDirCos[2][2];
	parentMatrix[12] = parentOriginGrnd[0];
	parentMatrix[13] = parentOriginGrnd[1];
	parentMatrix[14] = parentOriginGrnd[2];

	Transform child;
	double* childMatrix = child.getMatrix();
	childMatrix[0] = childDirCos[0][0];
	childMatrix[1] = childDirCos[0][1];
	childMatrix[2] = childDirCos[0][2];
	childMatrix[4] = childDirCos[1][0];
	childMatrix[5] = childDirCos[1][1];
	childMatrix[6] = childDirCos[1][2];
	childMatrix[8] = childDirCos[2][0];
	childMatrix[9] = childDirCos[2][1];
	childMatrix[10] = childDirCos[2][2];
	childMatrix[12] = childOriginGrnd[0];
	childMatrix[13] = childOriginGrnd[1];
	childMatrix[14] = childOriginGrnd[2];

	// The joint's forward transform is the inverse of the parent transform
	// times the child transform.
	Mtx::Invert(4, parent.getMatrix(), parent.getMatrix());
	Mtx::Multiply(4, 4, 4, parent.getMatrix(), child.getMatrix(), _forwardTransform.getMatrix());

	return _forwardTransform;
}

//_____________________________________________________________________________
/**
 * Get the SdfastJoint's inverse transform.
 *
 * @return Reference to the inverse transform.
 */
const Transform& SdfastJoint::getInverseTransform()
{
	int i;
	double parentDirCos[3][3], parentOrigin[3], parentOriginGrnd[3];
	double childDirCos[3][3], childOrigin[3], childOriginGrnd[3];

	// Get the mass centers of the parent and child bodies, and negate them.
	// This gives you the coordinates of the OpenSim body's origin in the
	// reference frame of the SD/FAST body.
	_parentBody->getMassCenter(parentOrigin);
	_childBody->getMassCenter(childOrigin);
	for(i=0; i<3; i++) {
		parentOrigin[i] = -parentOrigin[i];
		childOrigin[i] = -childOrigin[i];
	}
	sdpos(_parentBody->getSdfastIndex(),parentOrigin,parentOriginGrnd);
	sdpos(_childBody->getSdfastIndex(),childOrigin,childOriginGrnd);

	// Now get the direction cosine matrices for the bodies.
	sdorient(_parentBody->getSdfastIndex(), parentDirCos);
	sdorient(_childBody->getSdfastIndex(), childDirCos);

	// Now copy the translations and orientations into transforms.
	// 'parent' is the transform from ground to the parent frame.
	// 'child' is the transform from ground to the child frame.
	Transform parent;
	double* parentMatrix = parent.getMatrix();
	parentMatrix[0] = parentDirCos[0][0];
	parentMatrix[1] = parentDirCos[0][1];
	parentMatrix[2] = parentDirCos[0][2];
	parentMatrix[4] = parentDirCos[1][0];
	parentMatrix[5] = parentDirCos[1][1];
	parentMatrix[6] = parentDirCos[1][2];
	parentMatrix[8] = parentDirCos[2][0];
	parentMatrix[9] = parentDirCos[2][1];
	parentMatrix[10] = parentDirCos[2][2];
	parentMatrix[12] = parentOriginGrnd[0];
	parentMatrix[13] = parentOriginGrnd[1];
	parentMatrix[14] = parentOriginGrnd[2];

	Transform child;
	double* childMatrix = child.getMatrix();
	childMatrix[0] = childDirCos[0][0];
	childMatrix[1] = childDirCos[0][1];
	childMatrix[2] = childDirCos[0][2];
	childMatrix[4] = childDirCos[1][0];
	childMatrix[5] = childDirCos[1][1];
	childMatrix[6] = childDirCos[1][2];
	childMatrix[8] = childDirCos[2][0];
	childMatrix[9] = childDirCos[2][1];
	childMatrix[10] = childDirCos[2][2];
	childMatrix[12] = childOriginGrnd[0];
	childMatrix[13] = childOriginGrnd[1];
	childMatrix[14] = childOriginGrnd[2];

	// The joint's forward transform is the inverse of the child transform
	// times the parent transform.
	Mtx::Invert(4, child.getMatrix(), child.getMatrix());
	Mtx::Multiply(4, 4, 4, child.getMatrix(), parent.getMatrix(), _inverseTransform.getMatrix());

	return _inverseTransform;
}

//_____________________________________________________________________________
/**
 * Set the name of the joint's parent body
 *
 * @param Name of the parent body.
 */
void SdfastJoint::setParentBodyName(const string& aName)
{
	_bodies.set(0, aName);
}

//_____________________________________________________________________________
/**
 * Set the name of the joint's child body
 *
 * @param Name of the child body.
 */
void SdfastJoint::setChildBodyName(const string& aName)
{
	_bodies.set(1, aName);
}

//_____________________________________________________________________________
/**
 * Set the [Sdfast] type of the joint
 *
 * @param Name of the joint type.
 */
void SdfastJoint::setSdfastType(const char* aName)
{
	_SdfastTypeName = string(aName);
}


//=============================================================================
// UTILITY
//=============================================================================
bool SdfastJoint::hasXYZAxes() const
{
   return true;
}

bool SdfastJoint::isTreeJoint() const
{
	int info[50], slider[6];
	sdjnt(getSdfastIndex(), info, slider);
	return info[1] == 0;
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
void SdfastJoint::scale(const ScaleSet& aScaleSet)
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

void SdfastJoint::peteTest()
{
	cout << "Joint: " << getName() << endl;
	cout << "   bodies: " << _bodies << endl;

	_forwardTransform.printMatrix();
}
