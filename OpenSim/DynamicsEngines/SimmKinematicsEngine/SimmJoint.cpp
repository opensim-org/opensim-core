// SimmJoint.cpp
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
#include <OpenSim/Common/rdMath.h>
#include <OpenSim/Common/Mtx.h>
#include <OpenSim/Common/Function.h>
#include <OpenSim/Common/Constant.h>
#include "SimmJoint.h"
#include <OpenSim/Simulation/Model/AbstractDynamicsEngine.h>
#include <OpenSim/Common/SimmMacros.h>
#include <OpenSim/Simulation/Model/DofSet.h>
#include "SimmRotationDof.h"
#include "SimmTranslationDof.h"
#include <OpenSim/Simulation/Model/BodySet.h>

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
SimmJoint::SimmJoint() :
	AbstractJoint(),
	_bodies(_bodiesProp.getValueStrArray()),
	_dofSetProp(PropertyObj("", DofSet())),
	_dofSet((DofSet&)_dofSetProp.getValueObj()),
	_childBody(NULL),
	_parentBody(NULL)
{
	setNull();
	setupProperties();
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
SimmJoint::~SimmJoint()
{
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aJoint SimmJoint to be copied.
 */
SimmJoint::SimmJoint(const SimmJoint &aJoint) :
   AbstractJoint(aJoint),
	_bodies(_bodiesProp.getValueStrArray()),
	_dofSetProp(PropertyObj("", DofSet())),
	_dofSet((DofSet&)_dofSetProp.getValueObj()),
	_childBody(NULL),
	_parentBody(NULL)
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
 * @return Pointer to a copy of this SimmJoint.
 */
Object* SimmJoint::copy() const
{
	SimmJoint *joint = new SimmJoint(*this);
	return(joint);
}

//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Copy data members from one SimmJoint to another.
 *
 * @param aJoint SimmJoint to be copied.
 */
void SimmJoint::copyData(const SimmJoint &aJoint)
{
	_bodies = aJoint._bodies;
	_dofSet = aJoint._dofSet;

	_childBody = aJoint._childBody;
	_parentBody = aJoint._parentBody;
}

//_____________________________________________________________________________
/**
 * Set the data members of this SimmJoint to their null values.
 */
void SimmJoint::setNull()
{
	setType("SimmJoint");
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void SimmJoint::setupProperties()
{
	_bodiesProp.setName("bodies");
	_propertySet.append(&_bodiesProp);

	_dofSetProp.setName("DofSet");
	_propertySet.append(&_dofSetProp);
}

//_____________________________________________________________________________
/**
 * Perform some set up functions that happen after the
 * object has been deserialized or copied.
 *
 * @param aEngine dynamics engine containing this SimmJoint.
 */
void SimmJoint::setup(AbstractDynamicsEngine* aEngine)
{
	string errorMessage;

	AbstractJoint::setup(aEngine);

	_forwardTransform.setIdentity();
	_inverseTransform.setIdentity();
	_transformsValid=false;

	/* Look up the parent and child bodies by name in the
	 * dynamics engine and store pointers to them.
	 */
	_parentBody = aEngine->getBodySet()->get(_bodies[0]);
	if (!_parentBody)
	{
		errorMessage += "Invalid parent body (" + _bodies[0] + ") specified in joint " + getName();
		throw (Exception(errorMessage.c_str()));
	}

	_childBody = aEngine->getBodySet()->get(_bodies[1]);
	if (!_childBody)
	{
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
SimmJoint& SimmJoint::operator=(const SimmJoint &aJoint)
{
	// BASE CLASS
	AbstractJoint::operator=(aJoint);

	copyData(aJoint);

	return(*this);
}

//=============================================================================
// GET AND SET
//=============================================================================
//_____________________________________________________________________________
/**
 * Get the SimmJoint's forward transform.
 *
 * @return Reference to the forward transform.
 */
const Transform& SimmJoint::getForwardTransform()
{
	if (!_transformsValid)
		calcTransforms();

	return _forwardTransform;
}

//_____________________________________________________________________________
/**
 * Get the SimmJoint's inverse transform.
 *
 * @return Reference to the inverse transform.
 */
const Transform& SimmJoint::getInverseTransform()
{
	if (!_transformsValid)
		calcTransforms();

	return _inverseTransform;
}

//=============================================================================
// TRANSFORMS
//=============================================================================
//_____________________________________________________________________________
/**
 * Calculate the SimmJoint's forward and inverse transforms.
 */
void SimmJoint::calcTransforms()
{
	int i;
	Transform mat;
	double t[4], translation[4];

	for (i = 0; i < 4; i++)
		translation[i] = 0.0;

   for (i = 0; i < _dofSet.getSize(); i++)
   {
		if (_dofSet.get(i)->getMotionType() == AbstractDof::Translational)
		{
			SimmTranslationDof* td = dynamic_cast<SimmTranslationDof*>(_dofSet.get(i));
			td->getTranslation(t);
			Mtx::Multiply(1, 4, 4, t, mat.getMatrix(), t);
			Mtx::Add(4, 1, translation, t, translation);
		}
		else if (_dofSet.get(i)->getMotionType() == AbstractDof::Rotational)
		{
			SimmRotationDof* rd = dynamic_cast<SimmRotationDof*>(_dofSet.get(i));
			Transform m;
			m.rotateAxis(rd->getValue(), Transform::Radians, rd->getAxisPtr());
			Mtx::Multiply(4, 4, 4, m.getMatrix(), mat.getMatrix(), mat.getMatrix());
		}
   }

   /* now apply the translation that you calculated earlier */
   mat.translate(translation);

   _forwardTransform = mat;
	Mtx::Invert(4, mat.getMatrix(), _inverseTransform.getMatrix());

   _transformsValid = true;
}

//=============================================================================
// UTILITY
//=============================================================================
//_____________________________________________________________________________
/**
 * Check is a coordinate is used in the SimmJoint.
 *
 * @param aCoordinate coordinate to look for in joint.
 * @return Whether or not the coordinate is used in the joint.
 */
bool SimmJoint::isCoordinateUsed(AbstractCoordinate* aCoordinate) const
{
	int i;
	for (i = 0; i < _dofSet.getSize(); i++)
	{
		if (_dofSet.get(i)->getCoordinate() == aCoordinate)
			return true;
	}

	return false;
}

bool SimmJoint::hasXYZAxes() const
{
	int i;
   bool xTaken = false, yTaken = false, zTaken = false;

   for (i = 0; i < _dofSet.getSize(); i++)
   {
		if (_dofSet.get(i)->getMotionType() == AbstractDof::Rotational)
		{
			const double* axis = _dofSet.get(i)->getAxisPtr();

			if (!xTaken && EQUAL_WITHIN_ERROR(axis[0], 1.0) &&
				 EQUAL_WITHIN_ERROR(axis[1], 0.0) && EQUAL_WITHIN_ERROR(axis[2], 0.0))
			{
				xTaken = true;
				break;
			}
			else if (!yTaken && EQUAL_WITHIN_ERROR(axis[0], 0.0) &&
				      EQUAL_WITHIN_ERROR(axis[1], 1.0) && EQUAL_WITHIN_ERROR(axis[2], 0.0))
			{
				yTaken = true;
				break;
			}
			else if (!zTaken && EQUAL_WITHIN_ERROR(axis[0], 0.0) &&
				      EQUAL_WITHIN_ERROR(axis[1], 0.0) && EQUAL_WITHIN_ERROR(axis[2], 1.0))
			{
				zTaken = true;
				break;
			}
			return false;
		}
   }

   return true;
}

//=============================================================================
// SCALING
//=============================================================================
//_____________________________________________________________________________
/**
 * Scale a joint based on XYZ scale factors for the bodies.
 *
 * @param aScaleSet set of XYZ scale factors for the bodies.
 */
void SimmJoint::scale(const ScaleSet &aScaleSet)
{
	Array<double> scaleFactors(1.0, 3);

	// Joint kinematics are scaled by the scale factors for the
	// parent body, so get those body's factors
	const string& parentName = getParentBody()->getName();

	for (int i = 0; i < aScaleSet.getSize(); i++)
	{
		Scale *aScale = aScaleSet.get(i);
		if (aScale->getSegmentName() == parentName)
		{
			aScale->getScaleFactors(scaleFactors);
			break;
		}
	}

	scale(scaleFactors);
}
//_____________________________________________________________________________
/**
 * Scale a joint based on XYZ scale factors for the parent body.
 *
 * @param aScaleSet set of XYZ scale factors for the parent body.
 */
void SimmJoint::scale(const Array<double> &aScaleFactors)
{
	// If all three factors are equal to 1.0, do nothing.
	if (EQUAL_WITHIN_ERROR(aScaleFactors[0], 1.0) &&
		 EQUAL_WITHIN_ERROR(aScaleFactors[1], 1.0) &&
		 EQUAL_WITHIN_ERROR(aScaleFactors[2], 1.0))
		 return;

	// Scaling will happen, so invalidate the transforms.
	_transformsValid = false;

	/* This code assumes that if the DOF is a function with 2 points, then it
	 * acts as the independent gencoord, and should not be scaled. It
	 * also assumes that if the function has 3 or more points, it should be
	 * scaled.
	 */
   for (int i = 0; i < _dofSet.getSize(); i++)
   {
		if (_dofSet.get(i)->getMotionType() == AbstractDof::Translational)
		{
			SimmTranslationDof* transDof = dynamic_cast<SimmTranslationDof*>(_dofSet.get(i));
			Function* function = transDof->getFunction();
			int axis = transDof->getAxisIndex();

			if (transDof->getCoordinate() == NULL)
			{
				/* If the DOF has no coordinate, then it is a constant, so it should
				 * cast to a Constant.
				 */
				Constant* cons = dynamic_cast<Constant*>(function);
				if (cons)
					cons->setValue(transDof->getValue() * aScaleFactors[axis]);
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
					function->scaleY(aScaleFactors[axis]);
			}
		}
	}
}
