// SimbodyJoint01_05.cpp
// Author: Frank C. Anderson, Peter Loan
/*
 * Copyright (c)  2007, Stanford University. All rights reserved. 
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
#include <iostream>
#include <math.h>
#include <OpenSim/Common/rdMath.h>
#include <OpenSim/Common/Mtx.h>
#include <OpenSim/Common/Function.h>
#include <OpenSim/Common/Constant.h>
#include "SimbodyJoint01_05.h"
#include "SimbodyRotationDof01_05.h"
#include "SimbodyTranslationDof01_05.h"
#include "SimbodyEngine01_05.h"
#include <OpenSim/Common/SimmMacros.h>
#include <OpenSim/Simulation/Model/BodySet.h>

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace SimTK;
using namespace OpenSim;

const std::string SimbodyJoint01_05::_NewType = "Joint";

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
SimbodyJoint01_05::~SimbodyJoint01_05()
{
}
//_____________________________________________________________________________
/**
 * Default constructor.
 */
SimbodyJoint01_05::SimbodyJoint01_05() :
	AbstractJoint(),
	_bodies(_bodiesProp.getValueStrArray()),
	_locationInParent(_locationInParentProp.getValueDblVec3()),
	_locationInChild(_locationInChildProp.getValueDblVec3()),
	_dofSetProp(PropertyObj("", DofSet01_05())),
	_dofSet((DofSet01_05&)_dofSetProp.getValueObj())

{
	setNull();
	setupProperties();
	updateSimbody();
}
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aJoint SimbodyJoint01_05 to be copied.
 */
SimbodyJoint01_05::SimbodyJoint01_05(const SimbodyJoint01_05 &aJoint) :
   AbstractJoint(aJoint),
	_bodies(_bodiesProp.getValueStrArray()),
	_locationInParent(_locationInParentProp.getValueDblVec3()),
	_locationInChild(_locationInChildProp.getValueDblVec3()),
	_dofSetProp(PropertyObj("", DofSet01_05())),
	_dofSet((DofSet01_05&)_dofSetProp.getValueObj())
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
 * @return Pointer to a copy of this SimbodyJoint01_05.
 */
Object* SimbodyJoint01_05::copy() const
{
	SimbodyJoint01_05 *joint = new SimbodyJoint01_05(*this);
	return(joint);
}

//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Copy data members from one SimbodyJoint01_05 to another.
 *
 * @param aJoint SimbodyJoint01_05 to be copied.
 */
void SimbodyJoint01_05::copyData(const SimbodyJoint01_05 &aJoint)
{
	_bodies = aJoint._bodies;
	_dofSet = aJoint._dofSet;
	setLocationInParent(aJoint._locationInParent);
	setLocationInChild(aJoint._locationInChild);
	_childBody = aJoint._childBody;
	_parentBody = aJoint._parentBody;
}

//_____________________________________________________________________________
/**
 * Set the data members of this SimbodyJoint01_05 to their null values.
 */
void SimbodyJoint01_05::setNull()
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
void SimbodyJoint01_05::setupProperties()
{
	_bodiesProp.setName("bodies");
	_propertySet.append(&_bodiesProp);

	_dofSetProp.setName("DofSet");
	_propertySet.append(&_dofSetProp);

	SimTK::Vec3 origin(0.0, 0.0, 0.0);
	_locationInParentProp.setName("location_in_parent");
	_locationInParentProp.setValue(origin);
	_propertySet.append(&_locationInParentProp);

	_locationInChildProp.setName("location_in_child");
	_locationInChildProp.setValue(origin);
	_propertySet.append(&_locationInChildProp);
}

//_____________________________________________________________________________
/**
 * Update the underlying SDFast parameters, such as the inboard to joint and
 * body to joint vectors.
 *
 * @return True if the new inboard to joint was set; false otherwise.
 */
void SimbodyJoint01_05::updateSimbody()
{
	setLocationInParent(_locationInParent);
	setLocationInChild(_locationInChild);
}

//_____________________________________________________________________________
/**
 * Perform some set up functions that happen after the
 * object has been deserialized or copied.
 *
 * @param aEngine dynamics engine containing this SimbodyJoint01_05.
 */
void SimbodyJoint01_05::setup(AbstractDynamicsEngine* aEngine)
{
/*	string errorMessage;

	// Base class
	AbstractJoint::setup(aEngine);
	_engine = dynamic_cast<SimbodyEngine01_05*>(aEngine);

	// Look up the parent and child bodies by name in the
	// dynamics engine and store pointers to them.
	_parentBody = dynamic_cast<SimbodyBody01_05*>(aEngine->getBodySet()->get(_bodies[0]));
	if (!_parentBody) {
		errorMessage += "Invalid parent body (" + _bodies[0] + ") specified in joint " + getName();
		throw (Exception(errorMessage.c_str()));
	}

	_childBody = dynamic_cast<SimbodyBody01_05*>(aEngine->getBodySet()->get(_bodies[1]));
	if (!_childBody) {
		errorMessage += "Invalid child body (" + _bodies[0] + ") specified in joint " + getName();
		throw (Exception(errorMessage.c_str()));
	}

	/* Set up each of the dofs. */
	int i;
   for (i = 0; i < _dofSet.getSize(); i++)
		_dofSet.get(i)->setup(aEngine, this);
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
SimbodyJoint01_05& SimbodyJoint01_05::operator=(const SimbodyJoint01_05 &aJoint)
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
void SimbodyJoint01_05::setLocationInParent(const SimTK::Vec3& aLocation)
{
	// Update Simbody
	if(_parentBody!=NULL) {
		// TODO:  Find out what needs to be done to update Simbody.
	}

	// Update property
	_locationInParent = aLocation;
}
//_____________________________________________________________________________
/**
 * Get the location of this joint in its parent body.
 *
 * @param rLocation Currnt location specified in the parent body frame.
 */
void SimbodyJoint01_05::getLocationInParent(SimTK::Vec3& rLocation) const
{
	rLocation=_locationInParent;
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
void SimbodyJoint01_05::setLocationInChild(const SimTK::Vec3& aLocation)
{
	if(_childBody!=NULL) {
		// TODO:  Find out what needs to be done to update Simbody.
	}

	// UPDATE PROPERTY
	_locationInChild = aLocation;
}
//_____________________________________________________________________________
/**
 * Get the location of this joint in its child body.
 *
 * @param rLocation Current location specified in the child body frame.
 */
void SimbodyJoint01_05::getLocationInChild(SimTK::Vec3& rLocation) const
{
	rLocation=_locationInChild;
}


//_____________________________________________________________________________
/**
 * Get the SimbodyJoint01_05's forward transform.
 *
 * @return Reference to the forward transform.
 */
const OpenSim::Transform& SimbodyJoint01_05::getForwardTransform()
{

	return _forwardTransform;
}

//_____________________________________________________________________________
/**
 * Get the SimbodyJoint01_05's inverse transform.
 *
 * @return Reference to the inverse transform.
 */
const OpenSim::Transform& SimbodyJoint01_05::getInverseTransform()
{

	return _inverseTransform;
}

//_____________________________________________________________________________
/**
 * Set the name of the joint's parent body.
 *
 * @param Name of the parent body.
 */
void SimbodyJoint01_05::setParentBodyName(const string& aName)
{
	_bodies.set(0, aName);
}
//_____________________________________________________________________________
/**
 * Get the name of the joint's parent body.
 *
 * @return Name of the parent body.
 */
string SimbodyJoint01_05::getParentBodyName()
{
	if(_bodies.getSize()<1) return "";
	return _bodies[0];
}

//_____________________________________________________________________________
/**
 * Set the name of the joint's child body
 *
 * @param Name of the child body.
 */
void SimbodyJoint01_05::setBodyName(const string& aName)
{
	_bodies.set(1, aName);
}
//_____________________________________________________________________________
/**
 * Get the name of the joint's child body.
 *
 * @return Name of the child body.
 */
string SimbodyJoint01_05::getBodyName()
{
	if(_bodies.getSize()<2) return "";
	return _bodies[1];
}



//=============================================================================
// UTILITY
//=============================================================================
bool SimbodyJoint01_05::hasXYZAxes() const
{
   return true;
}

bool SimbodyJoint01_05::isTreeJoint() const
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
void SimbodyJoint01_05::scale(const ScaleSet& aScaleSet)
{
	Vec3 scaleFactors(1.0);

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


	/* This code assumes that if the DOF is a function with 2 points, then it
	 * acts as the independent gencoord, and should not be scaled. It
	 * also assumes that if the function has 3 or more points, it should be
	 * scaled.
	 */
   for (int i = 0; i < _dofSet.getSize(); i++)
   {
		if (_dofSet.get(i)->getMotionType() == AbstractTransformAxis::Translational)
		{
			SimbodyTranslationDof01_05* transDof = dynamic_cast<SimbodyTranslationDof01_05*>(_dofSet.get(i));
			Function* function = transDof->getFunction();
			int axis = transDof->getAxisIndex();

			if (transDof->getCoordinate() == NULL)
			{
				/* If the DOF has no coordinate, then it is a constant, so it should
				 * cast to a Constant.
				 */
				Constant* cons = dynamic_cast<Constant*>(function);
				if (cons)
					cons->setValue(transDof->getValue() * scaleFactors[axis]);
			}
			else
			{
				bool scaleIt = false;
				if (function->getNumberOfPoints() > 2)
				{
					scaleIt = true;
				}
				else if (function->getNumberOfPoints() == 2)
				{
					// If the function does not pass through 0,0 or its slope
					// at 0,0 is not 1 or -1, then scale the function.
					double valueAtZero = function->evaluate(0, 0.0, 0.0, 0.0);
					double slopeAtZero = function->evaluate(1, 0.0, 0.0, 0.0);

					if (NOT_EQUAL_WITHIN_ERROR(valueAtZero, 0.0) ||
						 (NOT_EQUAL_WITHIN_ERROR(slopeAtZero, 1.0) && NOT_EQUAL_WITHIN_ERROR(slopeAtZero, -1.0)))
						scaleIt = true;
				}

				if (scaleIt)
					function->scaleY(scaleFactors[axis]);
			}
		}
	}

}

