// SimmJoint.cpp
// Author: Peter Loan
/* Copyright (c) 2005, Stanford University and Peter Loan.
 * 
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
#include <OpenSim/Tools/rdMath.h>
#include <OpenSim/Tools/Mtx.h>
#include <OpenSim/Tools/Function.h>
#include "simmMacros.h"
#include "SimmJoint.h"
#include "SimmRotationDof.h"
#include "SimmTranslationDof.h"
#include "SimmKinematicsEngine.h"
#include "SimmModel.h"
#include "SimmBody.h"

//=============================================================================
// STATICS
//=============================================================================


using namespace OpenSim;
using namespace std;

static char fixedString[] = "fixed";
bool axesAreParallel(const double* axis1, const double* axis2, bool oppositeDirAllowed);

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */
SimmJoint::SimmJoint() :
	_bodies(_bodiesProp.getValueStrArray()),
   _dofs((ArrayPtrs<SimmDof>&)_dofsProp.getValueObjArray()),
	_transformsValid(false),
	_parentBody(NULL),
	_childBody(NULL)
{
	setNull();
}

//_____________________________________________________________________________
/**
 * Constructor from an XML node
 */
SimmJoint::SimmJoint(DOMElement *aElement) :
   Object(aElement),
	_bodies(_bodiesProp.getValueStrArray()),
   _dofs((ArrayPtrs<SimmDof>&)_dofsProp.getValueObjArray()),
	_transformsValid(false),
	_parentBody(NULL),
	_childBody(NULL)
{
	setNull();

	updateFromXMLNode();
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
   Object(aJoint),
	_bodies(_bodiesProp.getValueStrArray()),
   _dofs((ArrayPtrs<SimmDof>&)_dofsProp.getValueObjArray()),
	_transformsValid(false),
	_parentBody(NULL),
	_childBody(NULL)
{
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
//_____________________________________________________________________________
/**
 * Copy this SimmJoint and modify the copy so that it is consistent
 * with a specified XML element node.
 *
 * The copy is constructed by first using
 * SimmJoint::SimmJoint(DOMElement*) in order to establish the
 * relationship of the SimmJoint object with the XML node. Then, the
 * assignment operator is used to set all data members of the copy to the
 * values of this SimmJoint object. Finally, the data members of the copy are
 * updated using SimmJoint::updateFromXMLNode().
 *
 * @param aElement XML element. 
 * @return Pointer to a copy of this SimmJoint.
 */
Object* SimmJoint::copy(DOMElement *aElement) const
{
	SimmJoint *joint = new SimmJoint(aElement);
	*joint = *this;
	joint->updateFromXMLNode();
	return(joint);
}

void SimmJoint::copyData(const SimmJoint &aJoint)
{
	_bodies = aJoint._bodies;
	_dofs = aJoint._dofs;

	_childBody = aJoint._childBody;
	_parentBody = aJoint._parentBody;

	_transformsValid = false;
}


//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the data members of this SimmJoint to their null values.
 */
void SimmJoint::setNull()
{
	setupProperties();
	setType("SimmJoint");
}
//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void SimmJoint::setupProperties()
{
	_bodiesProp.setName("Bodies");
	_propertySet.append(&_bodiesProp);

	_dofsProp.setName("Dofs");
	ArrayPtrs<Object> dof;
	_dofsProp.setValue(dof);
	_propertySet.append(&_dofsProp);
}

SimmJoint& SimmJoint::operator=(const SimmJoint &aJoint)
{
	// BASE CLASS
	Object::operator=(aJoint);

	copyData(aJoint);

	return(*this);
}

/* Perform some set up functions that happen after the
 * object has been deserialized or copied.
 */
void SimmJoint::setup(SimmKinematicsEngine* aEngine)
{
	string errorMessage;

	_forwaTransform.setIdentity();
	_inverseTransform.setIdentity();

	/* Look up the parent and child bodies by name in the
	 * kinematics engine and store pointers to them.
	 */
	_parentBody = aEngine->getBody(_bodies[0]);
	if (!_parentBody)
	{
		errorMessage += "Invalid parent body (" + _bodies[0] + ") specified in joint " + getName();
		throw (Exception(errorMessage.c_str()));
	}

	_childBody = aEngine->getBody(_bodies[1]);
	if (!_childBody)
	{
		errorMessage += "Invalid child body (" + _bodies[0] + ") specified in joint " + getName();
		throw (Exception(errorMessage.c_str()));
	}

	/* Set up each of the dofs. */
   for (int i = 0; i < _dofs.getSize(); i++)
		_dofs[i]->setup(aEngine, this);
}

const Transform& SimmJoint::getForwaTransform()
{
	if (!_transformsValid)
		calcTransforms();

	return _forwaTransform;
}

const Transform& SimmJoint::getInverseTransform()
{
	if (!_transformsValid)
		calcTransforms();

	return _inverseTransform;
}

void SimmJoint::calcTransforms()
{
	int i;
	Transform mat;
	double t[4], translation[4];

	for (i = 0; i < 4; i++)
		translation[i] = 0.0;

   for (i = 0; i < _dofs.getSize(); i++)
   {
		if (_dofs[i]->getDofType() == SimmDof::Translational)
		{
			SimmTranslationDof* td = dynamic_cast<SimmTranslationDof*>(_dofs[i]);
			td->getTranslation(t);
			Mtx::Multiply(1, 4, 4, t, mat.getMatrix(), t);
			Mtx::Add(4, 1, translation, t, translation);
		}
		else if (_dofs[i]->getDofType() == SimmDof::Rotational)
		{
			SimmRotationDof* rd = dynamic_cast<SimmRotationDof*>(_dofs[i]);
			Transform m;
			m.rotateAxis(rd->getValue(), Transform::Degrees, rd->getAxisPtr());
			Mtx::Multiply(4, 4, 4, m.getMatrix(), mat.getMatrix(), mat.getMatrix());
		}
   }

   /* now apply the translation that you calculated earlier */
   mat.translate(translation);

   _forwaTransform = mat;
	Mtx::Invert(4, mat.getMatrix(), _inverseTransform.getMatrix());

   _transformsValid = true;
}

bool SimmJoint::isCoordinateUsed(SimmCoordinate* aCoordinate) const
{
	for (int i = 0; i < _dofs.getSize(); i++)
	{
		if (_dofs[i]->getCoordinate() == aCoordinate)
			return true;
	}

	return false;
}

void convertString(std::string& str, bool prependUnderscore)
{
   for (unsigned int i = 0; i < str.size(); i++)
   {
      if (str[i] >= 'a' && str[i] <= 'z')
         continue;
      if (str[i] >= 'A' && str[i] <= 'Z')
         continue;
      if (str[i] >= '0' && str[i] <= '9')
         continue;
      str[i] = '_';
   }

   /* If the first character is a number, prepend an underscore. */
   if (prependUnderscore && str[0] >= '0' && str[0] <= '9')
		str.insert('_', 0);
}

/* An SD/FAST-compatible joint is one with:
 *  1. no more than 3 translation DOFs
 *  2. no 2 translation DOFs along the same axis (X, Y, Z)
 *  3. no more than 3 rotation DOFs
 *  4. no rotations between the translation DOFs
 */
bool SimmJoint::isSdfastCompatible(void)
{
	int numRotations = 0, numChanges = 0;
	int numTx = 0, numTy = 0, numTz = 0;
	int lastDof = -1;

	for (int i = 0; i < _dofs.getSize(); i++)
	{
		if (_dofs[i]->getDofType() == SimmDof::Translational)
		{
			if (_dofs[i]->getName() == TX_NAME)
				numTx++;
			else if (_dofs[i]->getName() == TY_NAME)
				numTy++;
			else //if (_dofs[i]->getName() == TZ_NAME)
				numTz++;
			if (lastDof == SimmDof::Rotational)
				numChanges++;
		}
		else
		{
			numRotations++;
			if (lastDof == SimmDof::Translational)
				numChanges++;
		}
	}

	if (numRotations <= 3 && numChanges <= 1 &&
		 numTx <= 1 && numTy <= 1 && numTz <= 1)
		return true;

	return false;
}

bool SimmJoint::hasXYZAxes(void)
{
   bool xTaken = false, yTaken = false, zTaken = false;

   for (int i = 0; i < _dofs.getSize(); i++)
   {
		if (_dofs[i]->getDofType() == SimmDof::Rotational)
		{
			const double* axis = _dofs[i]->getAxisPtr();

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

/* This function finds the Nth rotation dof in the joint which
 * is either a function of a coordinate, or is a non-zero constant.
 * Non-zero constants are counted as functions for the purposes
 * of mapping the joint to an SD/FAST joint. N is zero-based.
 */
SimmRotationDof* SimmJoint::findNthFunctionRotation(int n) const
{
	for (int i = 0, count = 0; i < _dofs.getSize(); i++)
	{
		if (_dofs[i]->getDofType() == SimmDof::Rotational &&
			(_dofs[i]->getCoordinate() != NULL || NOT_EQUAL_WITHIN_ERROR(_dofs[i]->getValue(), 0.0)))
		{
			if (count++ == n)
				break;
		}
	}

	if (i == _dofs.getSize())
      return NULL;
	else
		return dynamic_cast<SimmRotationDof*>(_dofs[i]);
}

/* This function finds the Nth translation dof in the joint which
 * is a function of a coordinate. N is zero-based.
 */
SimmTranslationDof* SimmJoint::findNthFunctionTranslation(int n) const
{
	for (int i = 0, count = 0; i < _dofs.getSize(); i++)
	{
		if (_dofs[i]->getDofType() == SimmDof::Translational && _dofs[i]->getCoordinate() != NULL)
		{
			if (count++ == n)
				break;
		}
	}

	if (i == _dofs.getSize())
      return NULL;
	else
		return dynamic_cast<SimmTranslationDof*>(_dofs[i]);
}

/* Given a rotationDof, this functions finds the translationDof which uses
 * the same axis.
 */
SimmTranslationDof* SimmJoint::findMatchingTranslationDof(SimmRotationDof* rotDof)
{
	double rotAxis[3], transAxis[3];

	rotDof->getAxis(rotAxis);

	for (int i = 0; i < _dofs.getSize(); i++)
	{
		if (_dofs[i]->getDofType() == SimmDof::Translational)
		{
			_dofs[i]->getAxis(transAxis);
			if (axesAreParallel(rotAxis, transAxis, false))
				return dynamic_cast<SimmTranslationDof*>(_dofs[i]);
		}
	}

	return NULL;
}

bool axesAreParallel(const double* axis1, const double* axis2, bool oppositeDirAllowed)
{
   if (oppositeDirAllowed)
   {
      if (EQUAL_WITHIN_ERROR(DABS(axis1[0]),DABS(axis2[0])) &&
          EQUAL_WITHIN_ERROR(DABS(axis1[1]),DABS(axis2[1])) &&
          EQUAL_WITHIN_ERROR(DABS(axis1[2]),DABS(axis2[2])))
         return true;
      else
         return false;
   }
   else
   {
      if (EQUAL_WITHIN_ERROR(axis1[0],axis2[0]) &&
          EQUAL_WITHIN_ERROR(axis1[1],axis2[1]) &&
          EQUAL_WITHIN_ERROR(axis1[2],axis2[2]))
         return true;
      else
         return false;
   }
}

void SimmJoint::identifyDpType(SimmModel* aModel)
{
   int i;
   int numConstantRotations = 0, numFunctionRotations = 0;
	int numRealConstantRotations = 0, numRealFunctionRotations = 0;
   int numConstantTranslations = 0, numFunctionTranslations = 0;

   /* Give the joint a one-token name. In most cases, this is just
    * the user-given name of the joint. However, if that name has special
    * characters in it (e.g. -), those characters must be removed. Also,
    * if the name starts with a number, then an underscore is prepended.
    */
	_sdfastInfo.name = getName();
   convertString(_sdfastInfo.name, true);

	/* If the joint has no DOFs it cannot be converted to an SD/FAST joint
	 * (even a weld joint needs a dof to hold gencoord information because
	 * tree welds are modeled as fixed pins).
	 */
	if (_dofs.getSize() < 1)
	{
		_sdfastInfo.type = dpUnknownJoint;
		return;
	}

   /* Constant, non-zero rotations are not supported in SD/FAST. So to
    * implement them, you treat them as gencoords, and then prescribe
    * their motion to be constant (like weld joints).
	 *
    * For the purposes of optimizing the pipeline code, you want to remove
    * from the model segments that:
    *   (1) are leaf nodes (are used in only one joint and are not ground),
    *   (2) have no gencoords in their joint,
    *   (3) have zero mass, and
    *   (4) have no muscle points, wrap objects, or constraint objects.
    * Such segments are often used for holding markers, and so are not needed
    * in the dynamic simulation. They are removed by setting their joint's
	 * type to dpSkippable. Because constant, non-zero rotations are modeled
	 * as functions in SD/FAST, the numRealConstantRotations holds the number
	 * of "real" constant rotations so it can be determined if the joint is
	 * dpSkippable or not.
    */
	for (i = 0; i < _dofs.getSize(); i++)
	{
		if (_dofs[i]->getDofType() == SimmDof::Translational)
		{
			if (_dofs[i]->getCoordinate() == NULL)
				numConstantTranslations++;
			else
				numFunctionTranslations++;
		}
		else
		{
			if (_dofs[i]->getCoordinate() == NULL)
			{
				numRealConstantRotations++;
				if (EQUAL_WITHIN_ERROR(_dofs[i]->getValue(), 0.0))
					numConstantRotations++;
				else
					numFunctionRotations++;
			}
			else
			{
				numFunctionRotations++;
				numRealFunctionRotations++;
			}
		}
   }

	/* An SD/FAST-compatible joint is one with:
	 *  1. no more than 3 translation DOFs
	 *  2. no 2 translation DOFs along the same axis (X, Y, Z)
	 *  3. no more than 3 rotation DOFs
	 *  4. no rotations between the translation DOFs
	 */
	if (!isSdfastCompatible())
	{
		_sdfastInfo.type = dpUnknownJoint;
		return;
	}

   /* If the joint has no degrees of freedom, check to see if one of
    * the bodies is a leaf node. If it is, and the body does not have
	 * any elements needed for dynamics, mark the joint as dpSkippable
	 * so it will not be included in the dynamics.
    */
   if (numRealFunctionRotations == 0 && numFunctionTranslations == 0)
   {
		SimmBody* leafBody = aModel->getSimmKinematicsEngine().getLeafBody(this);

		if (leafBody != NULL && !aModel->bodyNeededForDynamics(leafBody))
		{
			_sdfastInfo.type = dpSkippable;
			return;
		}
   }

   if (numFunctionRotations == 0 && numFunctionTranslations == 0)
	{
      _sdfastInfo.type = dpWeld;
		return;
	}

	if (numFunctionRotations == 3 && numFunctionTranslations == 3)
   {
      /* In SD/FAST, bushing joints use the axes of rotation as
       * the axes of translation as well. Thus for a SIMM joint
       * to properly convert to an SD/FAST bushing, the rotation
       * axes must be X, Y, and Z, since the translations are
       * always w.r.t. these axes.
       */
      if (!hasXYZAxes())
         _sdfastInfo.type = dpUnknownJoint;
      else if (_dofs[0]->getDofType() == SimmDof::Translational) // translation first
         _sdfastInfo.type = dpBushing;
      else if (_dofs[0]->getDofType() == SimmDof::Rotational) // rotation first
         _sdfastInfo.type = dpReverseBushing;
      else
         _sdfastInfo.type = dpUnknownJoint;
		return;
   }

   if (numFunctionRotations == 1 && numFunctionTranslations == 0)
   {
      /* If the one rotation happens after the translations,
       * then this is a [normal] pin joint. If it happens
       * before, then this is a reverse pin joint, and the
       * translations have to be added to the other body segment.
       */
		for (i = 0; i < _dofs.getSize(); i++)
		{
			if (_dofs[i]->getDofType() == SimmDof::Rotational &&
				 (_dofs[i]->getCoordinate() != NULL || NOT_EQUAL_WITHIN_ERROR(_dofs[i]->getValue(), 0.0)))
			{
				if (_dofs[0]->getDofType() == SimmDof::Translational) // translation first
					_sdfastInfo.type = dpPin;
				else if (_dofs[0]->getDofType() == SimmDof::Rotational) // rotation first
					_sdfastInfo.type = dpReversePin;
				else
					_sdfastInfo.type = dpUnknownJoint;
				return;
			}
		}

		// Couldn't find the appropriate rotation
		_sdfastInfo.type = dpUnknownJoint;
		return;
   }

   if (numFunctionRotations == 0 && numFunctionTranslations == 1)
	{
      _sdfastInfo.type = dpSlider;
		return;
	}

   if (numFunctionRotations == 3 && numFunctionTranslations == 0)
   {
      if (_dofs[0]->getDofType() == SimmDof::Translational) // translation first
         _sdfastInfo.type = dpGimbal;
      else if (_dofs[0]->getDofType() == SimmDof::Rotational) // rotation first
         _sdfastInfo.type = dpReverseGimbal;
      else
         _sdfastInfo.type = dpUnknownJoint;
		return;
   }

   if (numFunctionRotations == 2 && numFunctionTranslations == 0)
   {
      if (_dofs[0]->getDofType() == SimmDof::Translational) // translation first
         _sdfastInfo.type = dpUniversal;
      else if (_dofs[0]->getDofType() == SimmDof::Rotational) // rotation first
         _sdfastInfo.type = dpReverseUniversal;
      else
         _sdfastInfo.type = dpUnknownJoint;
		return;
   }

   if (numFunctionRotations == 1 && numFunctionTranslations == 1)
   {
		SimmRotationDof* rd = findNthFunctionRotation(0);
		SimmTranslationDof* td = findNthFunctionTranslation(0);

		if (rd && td)
		{
			const double* axis1 = rd->getAxisPtr();
			const double* axis2 = td->getAxisPtr();

			if (axesAreParallel(axis1, axis2, false))
			{
				/* If the [one] rotation happens after the translations,
				 * then this is a normal cylinder joint. If it happens
				 * before, then this is a reverse cylinder joint, and the
				 * translations have to be added to the other body segment.
				 */
				if (_dofs[0]->getDofType() == SimmDof::Translational) // translation first
					_sdfastInfo.type = dpCylindrical;
				else if (_dofs[0]->getDofType() == SimmDof::Rotational) // rotation first
					_sdfastInfo.type = dpReverseCylindrical;
				else
					_sdfastInfo.type = dpUnknownJoint;
				return;
			}
		}

		_sdfastInfo.type = dpUnknownJoint;
		return;
   }

   if (numFunctionRotations == 1 && numFunctionTranslations == 2)
   {
		SimmRotationDof* rd = findNthFunctionRotation(0);
		SimmTranslationDof* td1 = findNthFunctionTranslation(0);
		SimmTranslationDof* td2 = findNthFunctionTranslation(1);

		if (rd && td1 && td2)
		{
			const double* axis1 = rd->getAxisPtr();
			const double* axis2 = td1->getAxisPtr();
			const double* axis3 = td2->getAxisPtr();

		   /* As long as the rotation axis is not parallel to either of
          * the translation axes, and the translation is not in the
          * middle of the transformation order, this is a valid planar joint.
          */
			if (axesAreParallel(axis1, axis2, true) || axesAreParallel(axis1, axis3, true))
			{
				_sdfastInfo.type = dpUnknownJoint;
				return;
			}
			else
			{
				if (_dofs[0]->getDofType() == SimmDof::Translational) // translation first
					_sdfastInfo.type = dpPlanar;
				else if (_dofs[0]->getDofType() == SimmDof::Rotational) // rotation first
					_sdfastInfo.type = dpReversePlanar;
				else
					_sdfastInfo.type = dpUnknownJoint;
				return;
			}
		}

		_sdfastInfo.type = dpUnknownJoint;
		return;
	}

	_sdfastInfo.type = dpUnknownJoint;
}

void SimmJoint::makeSdfastWeld(ofstream& out, int* dofCount, int* constrainedCount, bool writeFile)
{
	if (writeFile)
	{
		out << "pin = 1.0? 0.0? 0.0?" << endl;
		out << "prescribed = 1" << endl;
	}

	/* A [tree] weld joint is implemented as a fixed pin. You don't
	 * know how many DOFs a weld joint has, but you know it has at
	 * least one and that none of them is a function. Use the first
	 * one to hold the fixed pin information.
	 */
	char numString[10];
	sprintf(numString, "%d", *dofCount);
	_dofs[0]->_sdfastInfo.name = fixedString;
	_dofs[0]->_sdfastInfo.name.append(numString);
	_dofs[0]->_sdfastInfo.stateNumber = (*dofCount)++;
	_dofs[0]->_sdfastInfo.constrained = false;
	_dofs[0]->_sdfastInfo.fixed = true;
	_dofs[0]->_sdfastInfo.conversion = RTOD * _dofs[0]->_sdfastInfo.conversionSign;
	_dofs[0]->_sdfastInfo.joint = _sdfastInfo.index;
	_dofs[0]->_sdfastInfo.axis = 0;
	_dofs[0]->_sdfastInfo.initialValue = 0.0;
}

void SimmJoint::makeSdfastPin(ofstream& out, int* dofCount, int* constrainedCount, bool writeFile)
{
	double axis[3];
	SimmRotationDof* rotDof = findNthFunctionRotation(0);
	const SimmCoordinate* coord = rotDof->getCoordinate();

	rotDof->getAxis(axis);

	if (_sdfastInfo.direction == SimmStep::inverse)
	{
		axis[0] = -axis[0];
		axis[1] = -axis[1];
		axis[2] = -axis[2];
	}

	if (writeFile)
		out << "pin = " << axis[0] << "? " << axis[1] << "? " << axis[2] << "?" << endl;

	if (coord == NULL)
	{
		if (writeFile)
			out << "prescribed = 1" << endl;
		char numString[10];
		sprintf(numString, "%d", *dofCount); // TODO
		rotDof->_sdfastInfo.name = fixedString;
		rotDof->_sdfastInfo.name.append(numString);
		rotDof->_sdfastInfo.constrained = false;
		rotDof->_sdfastInfo.fixed = true;
		rotDof->_sdfastInfo.initialValue = rotDof->getValue();
	}
	else
	{
		rotDof->_sdfastInfo.fixed = false;
		if (rotDof->_sdfastInfo.constrained)
			rotDof->_sdfastInfo.errorNumber = (*constrainedCount)++;
		else if (writeFile)
			out << "prescribed = 0?" << endl;
		if (!rotDof->_sdfastInfo.constrained)
			rotDof->_sdfastInfo.initialValue = coord->getValue();
		else
			rotDof->_sdfastInfo.initialValue = rotDof->getValue();
	}

	rotDof->_sdfastInfo.stateNumber = (*dofCount)++;
	rotDof->_sdfastInfo.conversion = RTOD * rotDof->_sdfastInfo.conversionSign;
	rotDof->_sdfastInfo.joint = _sdfastInfo.index;
	rotDof->_sdfastInfo.axis = 0;
}

void SimmJoint::makeSdfastSlider(ofstream& out, int* dofCount, int* constrainedCount, bool writeFile)
{
	double axis[3];
	SimmTranslationDof* transDof = findNthFunctionTranslation(0);
	const SimmCoordinate* coord = transDof->getCoordinate();

	transDof->getAxis(axis);

	if (_sdfastInfo.direction == SimmStep::inverse)
	{
		axis[0] = -axis[0];
		axis[1] = -axis[1];
		axis[2] = -axis[2];
	}

	if (writeFile)
		out << "pin = " << axis[0] << "? " << axis[1] << "? " << axis[2] << "?" << endl;

	transDof->_sdfastInfo.stateNumber = (*dofCount)++;
	transDof->_sdfastInfo.fixed = false;
	if (transDof->_sdfastInfo.constrained)
		transDof->_sdfastInfo.errorNumber = (*constrainedCount)++;
	else if (writeFile)
		out << "prescribed = 0?" << endl;
	transDof->_sdfastInfo.conversion = transDof->_sdfastInfo.conversionSign;
	transDof->_sdfastInfo.joint = _sdfastInfo.index;
	transDof->_sdfastInfo.axis = 0;
	if (!transDof->_sdfastInfo.constrained)
		transDof->_sdfastInfo.initialValue = coord->getValue();
	else
		transDof->_sdfastInfo.initialValue = transDof->getValue();
}

void SimmJoint::makeSdfastPlanar(ofstream& out, int* dofCount, int* constrainedCount, bool writeFile)
{
	int i, axisNum = 0;
	double axis[3];
	string prescribedString;

	if ((_sdfastInfo.type == dpPlanar && _sdfastInfo.direction == SimmStep::forward) ||
		 (_sdfastInfo.type == dpReversePlanar && _sdfastInfo.direction == SimmStep::inverse))
	{
		/* Process the translations first. If the joint type was determined
		 * correctly, exactly two of them should be functions.
		 */
		for (i = 0; i < _dofs.getSize(); i++)
		{
			if (_dofs[i]->getDofType() == SimmDof::Translational &&
				 _dofs[i]->getCoordinate() != NULL)
			{
				if (writeFile)
				{
					_dofs[i]->getAxis(axis);
					if (_sdfastInfo.direction == SimmStep::inverse)
						out << "pin = " << -axis[0] << "? " << -axis[1] << "? " << -axis[2] << "?" << endl;
					else
						out << "pin = " << axis[0] << "? " << axis[1] << "? " << axis[2] << "?" << endl;
				}
				_dofs[i]->_sdfastInfo.stateNumber = (*dofCount)++;
				_dofs[i]->_sdfastInfo.fixed = false;
				if (_dofs[i]->_sdfastInfo.constrained)
				{
					_dofs[i]->_sdfastInfo.errorNumber = (*constrainedCount)++;
					prescribedString.append(" 0");
				}
				else
				{
					prescribedString.append(" 0?");
				}
				_dofs[i]->_sdfastInfo.initialValue = _dofs[i]->getValue();
				_dofs[i]->_sdfastInfo.conversion = _dofs[i]->_sdfastInfo.conversionSign;
				_dofs[i]->_sdfastInfo.joint = _sdfastInfo.index;
				_dofs[i]->_sdfastInfo.axis = axisNum++;
			}
		}

		/* Now process the rotation. If the joint type was determined
		 * correctly, exactly one of them should be a function or non-zero constant.
		 */
		SimmRotationDof* rotDof = findNthFunctionRotation(0);
		const SimmCoordinate* coord = rotDof->getCoordinate();
		rotDof->getAxis(axis);

		if (writeFile)
		{
			if (_sdfastInfo.direction == SimmStep::inverse)
				out << "pin = " << -axis[0] << "? " << -axis[1] << "? " << -axis[2] << "?" << endl;
			else
				out << "pin = " << axis[0] << "? " << axis[1] << "? " << axis[2] << "?" << endl;
		}

		if (coord == NULL)
		{
			prescribedString.append(" 1");
			char numString[10];
			sprintf(numString, "%d", *dofCount);
			rotDof->_sdfastInfo.name = fixedString;
			rotDof->_sdfastInfo.name.append(numString);
			rotDof->_sdfastInfo.constrained = false;
			rotDof->_sdfastInfo.fixed = true;
			rotDof->_sdfastInfo.initialValue = rotDof->getValue();
		}
		else
		{
			rotDof->_sdfastInfo.fixed = false;
			if (rotDof->_sdfastInfo.constrained)
			{
				rotDof->_sdfastInfo.errorNumber = (*constrainedCount)++;
				prescribedString.append(" 0");
				rotDof->_sdfastInfo.initialValue = rotDof->getValue();
			}
			else
			{
				prescribedString.append(" 0?");
				rotDof->_sdfastInfo.initialValue = coord->getValue();
			}
		}
		rotDof->_sdfastInfo.stateNumber = (*dofCount)++;
		rotDof->_sdfastInfo.conversion = RTOD * rotDof->_sdfastInfo.conversionSign;
		rotDof->_sdfastInfo.joint = _sdfastInfo.index;
		rotDof->_sdfastInfo.axis = 2;
	}
	else // (dpReversePlanar && forward) || (dpPlanar && inverse)
	{
		/* First process the rotation. If the joint type was determined
		 * correctly, exactly one of them should be a function or non-zero constant.
		 */
		SimmRotationDof* rotDof = findNthFunctionRotation(0);
		const SimmCoordinate* coord = rotDof->getCoordinate();
		rotDof->getAxis(axis);

		if (writeFile)
		{
			if (_sdfastInfo.direction == SimmStep::inverse)
				out << "pin = " << -axis[0] << "? " << -axis[1] << "? " << -axis[2] << "?" << endl;
			else
				out << "pin = " << axis[0] << "? " << axis[1] << "? " << axis[2] << "?" << endl;
		}

		if (coord == NULL)
		{
			prescribedString.append(" 1");
			char numString[10];
			sprintf(numString, "%d", *dofCount);
			rotDof->_sdfastInfo.name = fixedString;
			rotDof->_sdfastInfo.name.append(numString);
			rotDof->_sdfastInfo.constrained = false;
			rotDof->_sdfastInfo.fixed = true;
			rotDof->_sdfastInfo.initialValue = rotDof->getValue();
		}
		else
		{
			rotDof->_sdfastInfo.fixed = false;
			if (rotDof->_sdfastInfo.constrained)
			{
				rotDof->_sdfastInfo.errorNumber = (*constrainedCount)++;
				prescribedString.append(" 0");
				rotDof->_sdfastInfo.initialValue = rotDof->getValue();
			}
			else
			{
				prescribedString.append(" 0?");
				rotDof->_sdfastInfo.initialValue = coord->getValue();
			}
		}
		rotDof->_sdfastInfo.stateNumber = (*dofCount)++;
		rotDof->_sdfastInfo.conversion = RTOD * rotDof->_sdfastInfo.conversionSign;
		rotDof->_sdfastInfo.joint = _sdfastInfo.index;
		rotDof->_sdfastInfo.axis = axisNum++;

		/* Now process the translations, starting from the end of the
		 * dof array. If the joint type was determined correctly, exactly
		 * two of them should be functions.
		 */
		for (i = _dofs.getSize() - 1; i >= 0; i--)
		{
			if (_dofs[i]->getDofType() == SimmDof::Translational &&
				 _dofs[i]->getCoordinate() != NULL)
			{
				if (writeFile)
				{
					_dofs[i]->getAxis(axis);
					if (_sdfastInfo.direction == SimmStep::inverse)
						out << "pin = " << -axis[0] << "? " << -axis[1] << "? " << -axis[2] << "?" << endl;
					else
						out << "pin = " << axis[0] << "? " << axis[1] << "? " << axis[2] << "?" << endl;
				}
				_dofs[i]->_sdfastInfo.stateNumber = (*dofCount)++;
				_dofs[i]->_sdfastInfo.fixed = false;
				if (_dofs[i]->_sdfastInfo.constrained)
				{
					_dofs[i]->_sdfastInfo.errorNumber = (*constrainedCount)++;
					prescribedString.append(" 0");
				}
				else
				{
					prescribedString.append(" 0?");
				}
				_dofs[i]->_sdfastInfo.initialValue = _dofs[i]->getValue();
				_dofs[i]->_sdfastInfo.conversion = _dofs[i]->_sdfastInfo.conversionSign;
				_dofs[i]->_sdfastInfo.joint = _sdfastInfo.index;
				_dofs[i]->_sdfastInfo.axis = axisNum++;
			}
		}
	}

	if (writeFile)
		out << "prescribed =" << prescribedString << endl;
}

void SimmJoint::makeSdfastUniversal(ofstream& out, int* dofCount, int* constrainedCount, bool writeFile)
{
	int axisCount = 0;
	SimmDof* dof;
	string prescribedString;

	/* If the joint type has been determined properly, there should be
	 * exactly two rotationDofs that are functions or non-zero constants.
	 */
	for (int i = 0; i < _dofs.getSize(); i++)
	{
		if (_sdfastInfo.direction == SimmStep::forward)
			dof = _dofs[i];
		else
			dof = _dofs[_dofs.getSize() - 1 - i];

		const SimmCoordinate* coord = dof->getCoordinate();

		if (dof->getDofType() == SimmDof::Rotational &&
			(coord != NULL || NOT_EQUAL_WITHIN_ERROR(dof->getValue(), 0.0)))
		{
			if (writeFile)
			{
				double axis[3];

				dof->getAxis(axis);
				if (_sdfastInfo.direction == SimmStep::inverse)
					out << "pin = " << -axis[0] << "? " << -axis[1] << "? " << -axis[2] << "?" << endl;
				else
					out << "pin = " << axis[0] << "? " << axis[1] << "? " << axis[2] << "?" << endl;
			}

			if (coord == NULL)
			{
				prescribedString.append(" 1");
				char numString[10];
				sprintf(numString, "%d", *dofCount);
				dof->_sdfastInfo.name = fixedString;
				dof->_sdfastInfo.name.append(numString);
				dof->_sdfastInfo.constrained = false;
				dof->_sdfastInfo.fixed = true;
				dof->_sdfastInfo.initialValue = dof->getValue();
			}
			else
			{
				dof->_sdfastInfo.fixed = false;
				if (dof->_sdfastInfo.constrained)
				{
					dof->_sdfastInfo.errorNumber = (*constrainedCount)++;
					prescribedString.append(" 0");
					dof->_sdfastInfo.initialValue = dof->getValue();
				}
				else
				{
					prescribedString.append(" 0?");
					dof->_sdfastInfo.initialValue = coord->getValue();
				}
			}

			dof->_sdfastInfo.stateNumber = (*dofCount)++;
			dof->_sdfastInfo.conversion = RTOD * dof->_sdfastInfo.conversionSign;
			dof->_sdfastInfo.joint = _sdfastInfo.index;
			dof->_sdfastInfo.axis = axisCount++;
		}
	}

	if (writeFile)
		out << "prescribed =" << prescribedString << endl;
}

void SimmJoint::makeSdfastCylindrical(ofstream& out, int* dofCount, int* constrainedCount, bool writeFile)
{
	string prescribedString;

	/* Do the translation first. If the joint type has been determined
	 * properly, there should be exactly one translationDof that is a
	 * function.
	 */
	SimmTranslationDof* transDof = findNthFunctionTranslation(0);

	transDof->_sdfastInfo.stateNumber = (*dofCount)++;
	transDof->_sdfastInfo.fixed = false;
	if (transDof->_sdfastInfo.constrained)
	{
		transDof->_sdfastInfo.errorNumber = (*constrainedCount)++;
		prescribedString.append(" 0");
		transDof->_sdfastInfo.initialValue = transDof->getValue();
	}
	else
	{
		prescribedString.append(" 0?");
		transDof->_sdfastInfo.initialValue = transDof->getCoordinate()->getValue();
	}
	transDof->_sdfastInfo.conversion = transDof->_sdfastInfo.conversionSign;
	transDof->_sdfastInfo.joint = _sdfastInfo.index;
	transDof->_sdfastInfo.axis = 0;

	/* Now do the rotation. If the joint type has been determined properly,
	 * there should be exactly one rotationDof that is a function.
	 */
	SimmRotationDof* rotDof = findNthFunctionRotation(0);
	const SimmCoordinate* coord = rotDof->getCoordinate();

	if (writeFile)
	{
		double axis[3];

		rotDof->getAxis(axis);

		if (_sdfastInfo.direction == SimmStep::inverse)
			out << "pin = " << -axis[0] << "? " << -axis[1] << "? " << -axis[2] << "?" << endl;
		else
			out << "pin = " << axis[0] << "? " << axis[1] << "? " << axis[2] << "?" << endl;
	}

	if (coord == NULL)
	{
		prescribedString.append(" 1");
		char numString[10];
		sprintf(numString, "%d", *dofCount);
		rotDof->_sdfastInfo.name = fixedString;
		rotDof->_sdfastInfo.name.append(numString);
		rotDof->_sdfastInfo.constrained = false;
		rotDof->_sdfastInfo.fixed = true;
		rotDof->_sdfastInfo.initialValue = rotDof->getValue();
	}
	else
	{
		rotDof->_sdfastInfo.fixed = false;
		if (rotDof->_sdfastInfo.constrained)
		{
			rotDof->_sdfastInfo.errorNumber = (*constrainedCount)++;
			prescribedString.append(" 0");
			rotDof->_sdfastInfo.initialValue = rotDof->getValue();
		}
		else
		{
			prescribedString.append(" 0?");
			rotDof->_sdfastInfo.initialValue = coord->getValue();
		}
	}

	rotDof->_sdfastInfo.stateNumber = (*dofCount)++;
	rotDof->_sdfastInfo.conversion = RTOD * rotDof->_sdfastInfo.conversionSign;
	rotDof->_sdfastInfo.joint = _sdfastInfo.index;
	rotDof->_sdfastInfo.axis = 1;

	if (writeFile)
		out << "prescribed =" << prescribedString << endl;
}

void SimmJoint::makeSdfastGimbal(ofstream& out, int* dofCount, int* constrainedCount, bool writeFile)
{
	int axisCount = 0;
	string prescribedString;
	SimmDof* dof;

	/* If the joint type has been determined properly, there should be
	 * exactly three rotationDofs that are functions or non-zero constants.
	 */
	for (int i = 0; i < _dofs.getSize(); i++)
	{
		if (_sdfastInfo.direction == SimmStep::forward)
			dof = _dofs[i];
		else
			dof = _dofs[_dofs.getSize() - 1 - i];

		const SimmCoordinate* coord = dof->getCoordinate();

		if (dof->getDofType() == SimmDof::Rotational &&
			(coord != NULL || NOT_EQUAL_WITHIN_ERROR(dof->getValue(), 0.0)))
		{
			if (writeFile)
			{
				double axis[3];

				dof->getAxis(axis);
				if (_sdfastInfo.direction == SimmStep::inverse)
					out << "pin = " << -axis[0] << "? " << -axis[1] << "? " << -axis[2] << "?" << endl;
				else
					out << "pin = " << axis[0] << "? " << axis[1] << "? " << axis[2] << "?" << endl;
			}

			if (coord == NULL)
			{
				prescribedString.append(" 1");
				char numString[10];
				sprintf(numString, "%d", *dofCount);
				dof->_sdfastInfo.name = fixedString;
				dof->_sdfastInfo.name.append(numString);
				dof->_sdfastInfo.constrained = false;
				dof->_sdfastInfo.fixed = true;
				dof->_sdfastInfo.initialValue = dof->getValue();
			}
			else
			{
				dof->_sdfastInfo.fixed = false;
				if (dof->_sdfastInfo.constrained)
				{
					dof->_sdfastInfo.errorNumber = (*constrainedCount)++;
					prescribedString.append(" 0");
					dof->_sdfastInfo.initialValue = dof->getValue();
				}
				else
				{
					prescribedString.append(" 0?");
					dof->_sdfastInfo.initialValue = coord->getValue();
				}
			}

			dof->_sdfastInfo.stateNumber = (*dofCount)++;
			dof->_sdfastInfo.conversion = RTOD * dof->_sdfastInfo.conversionSign;
			dof->_sdfastInfo.joint = _sdfastInfo.index;
			dof->_sdfastInfo.axis = axisCount++;
		}
	}

	if (writeFile)
		out << "prescribed =" << prescribedString << endl;
}

void SimmJoint::makeSdfastBushing(ofstream& out, int* dofCount, int* constrainedCount, bool writeFile)
{
	int i, axisNum = 0;
	string prescribedString;
	SimmRotationDof* rotDofs[3];
	SimmTranslationDof* transDofs[3];

	if ((_sdfastInfo.type == dpBushing && _sdfastInfo.direction == SimmStep::forward) ||
		(_sdfastInfo.type == dpReverseBushing && _sdfastInfo.direction == SimmStep::inverse))
	{
		/* The translations must be processed in the same order as the rotations
		 * (since they share axes), so first make a list of the translations
		 * that is ordered the same way as the rotations.
		 */
		for (i = 0; i < 3; i++)
		{
			if (_sdfastInfo.direction == SimmStep::forward)
				rotDofs[i] = findNthFunctionRotation(i);
			else
				rotDofs[i] = findNthFunctionRotation(2-i);

			transDofs[i] = findMatchingTranslationDof(rotDofs[i]);
		}

		/* Now process the translations in that order. */
		for (i = 0; i < 3; i++)
		{
			transDofs[i]->_sdfastInfo.stateNumber = (*dofCount)++;
			transDofs[i]->_sdfastInfo.fixed = false;
			if (transDofs[i]->_sdfastInfo.constrained)
			{
				transDofs[i]->_sdfastInfo.errorNumber = (*constrainedCount)++;
				prescribedString.append(" 0");
				transDofs[i]->_sdfastInfo.initialValue = transDofs[i]->getValue();
			}
			else
			{
				prescribedString.append(" 0?");
				transDofs[i]->_sdfastInfo.initialValue = transDofs[i]->getValue();
			}
			transDofs[i]->_sdfastInfo.conversion = transDofs[i]->_sdfastInfo.conversionSign;
			transDofs[i]->_sdfastInfo.joint = _sdfastInfo.index;
			transDofs[i]->_sdfastInfo.axis = axisNum++;
		}

		/* Now process the rotations in that same order. */
		for (i = 0; i < 3; i++)
		{
			if (writeFile)
			{
				double axis[3];

				rotDofs[i]->getAxis(axis);
				if (_sdfastInfo.direction == SimmStep::inverse)
					out << "pin = " << -axis[0] << "? " << -axis[1] << "? " << -axis[2] << "?" << endl;
				else
					out << "pin = " << axis[0] << "? " << axis[1] << "? " << axis[2] << "?" << endl;
			}

			const SimmCoordinate* coord = rotDofs[i]->getCoordinate();

			if (coord == NULL)
			{
				prescribedString.append(" 1");
				char numString[10];
				sprintf(numString, "%d", *dofCount);
				rotDofs[i]->_sdfastInfo.name = fixedString;
				rotDofs[i]->_sdfastInfo.name.append(numString);
				rotDofs[i]->_sdfastInfo.constrained = false;
				rotDofs[i]->_sdfastInfo.fixed = true;
				rotDofs[i]->_sdfastInfo.initialValue = rotDofs[i]->getValue();
			}
			else
			{
				rotDofs[i]->_sdfastInfo.fixed = false;
				if (rotDofs[i]->_sdfastInfo.constrained)
				{
					rotDofs[i]->_sdfastInfo.errorNumber = (*constrainedCount)++;
					prescribedString.append(" 0");
					rotDofs[i]->_sdfastInfo.initialValue = rotDofs[i]->getValue();
				}
				else
				{
					prescribedString.append(" 0?");
					rotDofs[i]->_sdfastInfo.initialValue = coord->getValue();
				}
			}

			rotDofs[i]->_sdfastInfo.stateNumber = (*dofCount)++;
			rotDofs[i]->_sdfastInfo.conversion = RTOD * rotDofs[i]->_sdfastInfo.conversionSign;
			rotDofs[i]->_sdfastInfo.joint = _sdfastInfo.index;
			rotDofs[i]->_sdfastInfo.axis = axisNum++;
		}
	}
	else
	{
		/* The translations must be processed in the same order as the rotations
		 * (since they share axes), so first make a list of the translations
		 * that is ordered the same way as the rotations.
		 */
		for (i = 0; i < 3; i++)
		{
			if (_sdfastInfo.direction == SimmStep::forward)
				rotDofs[i] = findNthFunctionRotation(i);
			else
				rotDofs[i] = findNthFunctionRotation(2-i);

			transDofs[i] = findMatchingTranslationDof(rotDofs[i]);
		}

		/* Now process the rotations in that order. */
		for (i = 0; i < 3; i++)
		{
			if (writeFile)
			{
				double axis[3];

				rotDofs[i]->getAxis(axis);
				if (_sdfastInfo.direction == SimmStep::inverse)
					out << "pin = " << -axis[0] << "? " << -axis[1] << "? " << -axis[2] << "?" << endl;
				else
					out << "pin = " << axis[0] << "? " << axis[1] << "? " << axis[2] << "?" << endl;
			}

			const SimmCoordinate* coord = rotDofs[i]->getCoordinate();

			if (coord == NULL)
			{
				prescribedString.append(" 1");
				char numString[10];
				sprintf(numString, "%d", *dofCount);
				rotDofs[i]->_sdfastInfo.name = fixedString;
				rotDofs[i]->_sdfastInfo.name.append(numString);
				rotDofs[i]->_sdfastInfo.constrained = false;
				rotDofs[i]->_sdfastInfo.fixed = true;
				rotDofs[i]->_sdfastInfo.initialValue = rotDofs[i]->getValue();
			}
			else
			{
				rotDofs[i]->_sdfastInfo.fixed = false;
				if (rotDofs[i]->_sdfastInfo.constrained)
				{
					rotDofs[i]->_sdfastInfo.errorNumber = (*constrainedCount)++;
					prescribedString.append(" 0");
					rotDofs[i]->_sdfastInfo.initialValue = rotDofs[i]->getValue();
				}
				else
				{
					prescribedString.append(" 0?");
					rotDofs[i]->_sdfastInfo.initialValue = coord->getValue();
				}
			}

			rotDofs[i]->_sdfastInfo.stateNumber = (*dofCount)++;
			rotDofs[i]->_sdfastInfo.conversion = RTOD * rotDofs[i]->_sdfastInfo.conversionSign;
			rotDofs[i]->_sdfastInfo.joint = _sdfastInfo.index;
			rotDofs[i]->_sdfastInfo.axis = axisNum++;
		}

		/* Now process the translations in that same order. */
		for (i = 0; i < 3; i++)
		{
			transDofs[i]->_sdfastInfo.stateNumber = (*dofCount)++;
			transDofs[i]->_sdfastInfo.fixed = false;
			if (transDofs[i]->_sdfastInfo.constrained)
			{
				transDofs[i]->_sdfastInfo.errorNumber = (*constrainedCount)++;
				prescribedString.append(" 0");
				transDofs[i]->_sdfastInfo.initialValue = transDofs[i]->getValue();
			}
			else
			{
				prescribedString.append(" 0?");
				transDofs[i]->_sdfastInfo.initialValue = transDofs[i]->getValue();
			}
			transDofs[i]->_sdfastInfo.conversion = transDofs[i]->_sdfastInfo.conversionSign;
			transDofs[i]->_sdfastInfo.joint = _sdfastInfo.index;
			transDofs[i]->_sdfastInfo.axis = axisNum++;
		}
	}

	if (writeFile)
		out << "prescribed =" << prescribedString << endl;
}

char* getDpJointName(dpJointType type, SimmStep::Direction direction)
{
   if (type == dpPin || type == dpReversePin)
      return ("pin");
   else if (type == dpCylindrical || type == dpReverseCylindrical)
      return ("cylinder");
   else if (type == dpPlanar)
   {
		if (direction == SimmStep::forward)
         return ("planar");
      else
         return ("rplanar");
   }
   else if (type == dpReversePlanar)
   {
      if (direction == SimmStep::forward)
         return ("rplanar");
      else
         return ("planar");
   }
   else if (type == dpSlider)
      return ("slider");
   else if (type == dpUniversal || type == dpReverseUniversal)
      return ("ujoint");
   else if (type == dpGimbal || type == dpReverseGimbal)
      return ("gimbal");
   else if (type == dpWeld)
      return ("pin");                  /* welds are prescribed pins */
   else if (type == dpBushing)
   {
      if (direction == SimmStep::forward)
         return ("bushing");
      else
         return ("rbushing");
   }
   else if (type == dpReverseBushing)
   {
      if (direction == SimmStep::forward)
         return ("rbushing");
      else
         return ("bushing");
   }

   return ("unknown");
}

SimmTranslationDof* SimmJoint::getTranslationDof(int axis) const
{
	for (int i = 0; i < _dofs.getSize(); i++)
	{
		if (_dofs[i]->getDofType() == SimmDof::Translational)
		{
			double vec[3];
			_dofs[i]->getAxis(vec);
			if (EQUAL_WITHIN_ERROR(vec[axis], 1.0))
				return dynamic_cast<SimmTranslationDof*>(_dofs[i]);
		}
	}

	return NULL;
}

void SimmJoint::makeSdfastJoint(ofstream& out, ArrayPtrs<SimmSdfastBody>& sdfastBodies, int* dofCount, int* constrainedCount, bool writeFile)
{
	int i;
	double body1MassCenter[3], body2MassCenter[3];
	SimmTranslationDof* transDof;
   SimmBody *body1, *body2;
	SimmBody::sdfastBodyInfo* body2Info;

	if (_sdfastInfo.direction == SimmStep::forward)
   {
		body1 = getParentBody();
      body2 = getChildBody();
   }
   else
   {
		body1 = getChildBody();
      body2 = getParentBody();
   }
	body2Info = &body2->_sdfastInfo;
	body1->getMassCenter(body1MassCenter);
	body2->getMassCenter(body2MassCenter);

   if (writeFile)
   {
		out << "body = " << _sdfastInfo.outbname << " inb = " << _sdfastInfo.inbname << " joint = " <<
			getDpJointName(_sdfastInfo.type, _sdfastInfo.direction) << endl;
      if (_sdfastInfo.type == dpWeld)
			out << "# this is really a weld joint implemented as a prescribed pin" << endl;
   }

	/* Make a new SimmSdfastBody, which will later be added to the array of them
	 * (the sdfastBodies parameter).
	 */
	SimmSdfastBody* sdfastBody = new SimmSdfastBody;

	sdfastBody->_name = _sdfastInfo.outbname;

   /* If there are loops in the model, then SimmBodys get split and there
    * will be more sdfastBodys than SimmBodys. So that unsplittable body
    * parameters (like contact objects) can be assigned to the sdfastBodys,
    * each sdfastBody has an index of its corresponding SimmBody. But for
    * SimmBodys that were split, only one piece will have a valid index.
    */
	if (!_sdfastInfo.closesLoop)
   {
		if (_sdfastInfo.direction == SimmStep::forward)
			sdfastBody->_SimmBody = getChildBody();
      else
         sdfastBody->_SimmBody = getParentBody();
   }
   else
   {
      sdfastBody->_SimmBody = NULL;
   }

	/* Copy the mass parameters to the sdfastBody, using the massFactor
	 * parameter to split the values (massFactor = 1.0 for unsplit
	 * bodies).
	 */
	sdfastBody->_mass = body2->getMass() / body2Info->massFactor;
	const Array<double>& inertia = body2->getInertia();
	for (i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			sdfastBody->_inertia[i][j] = inertia[i * 3 + j] / body2Info->massFactor;
	body2->getMassCenter(sdfastBody->_massCenter);

   if (writeFile)
   {
		out << "mass = " << sdfastBody->_mass << "? inertia = " << sdfastBody->_inertia[0][0] << "? " <<
			sdfastBody->_inertia[0][1] << "? " << sdfastBody->_inertia[0][2] << "?" << endl;
		out << "                                 " << sdfastBody->_inertia[1][0] << "? " <<
			sdfastBody->_inertia[1][1] << "? " << sdfastBody->_inertia[1][2] << "?" << endl;
		out << "                                 " << sdfastBody->_inertia[2][0] << "? " <<
			sdfastBody->_inertia[2][1] << "? " << sdfastBody->_inertia[2][2] << "?" << endl;
   }

   /* The bodytojoint vector (inbtojoint vector for INVERSE joints) is always
    * defined by the negative of the mass center vector. The inbtojoint vector
    * (bodytojoint for INVERSE joints) can change depending on whether or not
    * some of the translations in the joint are functions. For all translation
    * components that are functions, you want to use just the mass center
    * vector component so that the DOF value is the same as the SIMM gencoord
    * or SIMM constraint function. For components that are constants, you want
    * to add the DOF value to the mass center vector so that the origin ends
    * up in the right place. 4/3/97.
    * In general, the above method works only for joints in which the translations
    * occur before the rotations. For cases in which the translations occur
    * between two or more rotations, the joint cannot be modeled easily in
    * SD/FAST. For cases in which the translations are after the rotations,
    * SD/FAST has "reverse" joints (e.g., rbushing) that automatically handle
    * the translations properly. Rplanar correctly handles this case because
    * the rplanar joint itself handles two of the translations, and the third
    * one is along the axis of rotation, so it does not need to be handled
    * separately. For joints which do not have a reverse (e.g., pin, cylinder,
    * universal, gimbal), you need to attach the translations to the "other"
    * body segment than you normally would. 4/10/97.
    */
   if (_sdfastInfo.type == dpReversePin || _sdfastInfo.type == dpReverseGimbal ||
       _sdfastInfo.type == dpReverseUniversal || _sdfastInfo.type == dpReverseCylindrical)
   {
		if (_sdfastInfo.direction == SimmStep::forward)
      {
			transDof = getTranslationDof(0);
			if (transDof->getCoordinate() == NULL)
				sdfastBody->_bodyToJoint[0] = -transDof->getValue() - body2MassCenter[0];
         else
            sdfastBody->_bodyToJoint[0] = -body2MassCenter[0];

			transDof = getTranslationDof(1);
			if (transDof->getCoordinate() == NULL)
            sdfastBody->_bodyToJoint[1] = -transDof->getValue() - body2MassCenter[1];
         else
            sdfastBody->_bodyToJoint[1] = -body2MassCenter[1];

			transDof = getTranslationDof(2);
			if (transDof->getCoordinate() == NULL)
            sdfastBody->_bodyToJoint[2] = -transDof->getValue() - body2MassCenter[2];
         else
            sdfastBody->_bodyToJoint[2] = -body2MassCenter[2];

			sdfastBody->_inboardToJoint[0] = -body1MassCenter[0];
         sdfastBody->_inboardToJoint[1] = -body1MassCenter[1];
         sdfastBody->_inboardToJoint[2] = -body1MassCenter[2];
      }
      else
      {
         sdfastBody->_bodyToJoint[0] = -body2MassCenter[0];
         sdfastBody->_bodyToJoint[1] = -body2MassCenter[1];
         sdfastBody->_bodyToJoint[2] = -body2MassCenter[2];

			transDof = getTranslationDof(0);
         if (transDof->getCoordinate() == NULL)
            sdfastBody->_inboardToJoint[0] = -transDof->getValue() - body1MassCenter[0];
         else
            sdfastBody->_inboardToJoint[0] = -body1MassCenter[0];

			transDof = getTranslationDof(1);
         if (transDof->getCoordinate() == NULL)
            sdfastBody->_inboardToJoint[1] = -transDof->getValue() - body1MassCenter[1];
         else
            sdfastBody->_inboardToJoint[1] = -body1MassCenter[1];

			transDof = getTranslationDof(2);
         if (transDof->getCoordinate() == NULL)
            sdfastBody->_inboardToJoint[2] = -transDof->getValue() - body1MassCenter[2];
         else
            sdfastBody->_inboardToJoint[2] = -body1MassCenter[2];
      }
   }
   else
   {
      if (_sdfastInfo.direction == SimmStep::forward)
      {
         sdfastBody->_bodyToJoint[0] = -body2MassCenter[0];
         sdfastBody->_bodyToJoint[1] = -body2MassCenter[1];
         sdfastBody->_bodyToJoint[2] = -body2MassCenter[2];

			transDof = getTranslationDof(0);
         if (transDof->getCoordinate() == NULL)
            sdfastBody->_inboardToJoint[0] = transDof->getValue() - body1MassCenter[0];
         else
            sdfastBody->_inboardToJoint[0] = -body1MassCenter[0];

			transDof = getTranslationDof(1);
         if (transDof->getCoordinate() == NULL)
            sdfastBody->_inboardToJoint[1] = transDof->getValue() - body1MassCenter[1];
         else
            sdfastBody->_inboardToJoint[1] = -body1MassCenter[1];

			transDof = getTranslationDof(2);
         if (transDof->getCoordinate() == NULL)
            sdfastBody->_inboardToJoint[2] = transDof->getValue() - body1MassCenter[2];
         else
            sdfastBody->_inboardToJoint[2] = -body1MassCenter[2];
      }
      else
      {
			transDof = getTranslationDof(0);
         if (transDof->getCoordinate() == NULL)
            sdfastBody->_bodyToJoint[0] = transDof->getValue() - body2MassCenter[0];
         else
            sdfastBody->_bodyToJoint[0] = -body2MassCenter[0];

			transDof = getTranslationDof(1);
         if (transDof->getCoordinate() == NULL)
            sdfastBody->_bodyToJoint[1] = transDof->getValue() - body2MassCenter[1];
         else
            sdfastBody->_bodyToJoint[1] = -body2MassCenter[1];

			transDof = getTranslationDof(2);
         if (transDof->getCoordinate() == NULL)
            sdfastBody->_bodyToJoint[2] = transDof->getValue() - body2MassCenter[2];
         else
            sdfastBody->_bodyToJoint[2] = -body2MassCenter[2];

         sdfastBody->_inboardToJoint[0] = -body1MassCenter[0];
         sdfastBody->_inboardToJoint[1] = -body1MassCenter[1];
         sdfastBody->_inboardToJoint[2] = -body1MassCenter[2];
      }
   }

   if (writeFile)
   {
		out << "bodytojoint = " << sdfastBody->_bodyToJoint[0] << "? " << sdfastBody->_bodyToJoint[1] <<
			"? " << sdfastBody->_bodyToJoint[2] << "? " << "   inbtojoint = " << sdfastBody->_inboardToJoint[0] <<
			"? " << sdfastBody->_inboardToJoint[1] << "? " << sdfastBody->_inboardToJoint[2] << "?" << endl;
   }

	switch (_sdfastInfo.type)
	{
	   case dpWeld:
		   makeSdfastWeld(out, dofCount, constrainedCount, writeFile);
		   break;
		case dpPin:
		case dpReversePin:
			makeSdfastPin(out, dofCount, constrainedCount, writeFile);
			break;
		case dpSlider:
			makeSdfastSlider(out, dofCount, constrainedCount, writeFile);
			break;
		case dpPlanar:
		case dpReversePlanar:
			makeSdfastPlanar(out, dofCount, constrainedCount, writeFile);
			break;
		case dpUniversal:
		case dpReverseUniversal:
			makeSdfastUniversal(out, dofCount, constrainedCount, writeFile);
			break;
		case dpCylindrical:
		case dpReverseCylindrical:
			makeSdfastCylindrical(out, dofCount, constrainedCount, writeFile);
			break;
		case dpGimbal:
		case dpReverseGimbal:
			makeSdfastGimbal(out, dofCount, constrainedCount, writeFile);
			break;
		case dpBushing:
		case dpReverseBushing:
			makeSdfastBushing(out, dofCount, constrainedCount, writeFile);
			break;
   }

   if (writeFile)
		out << endl;

	/* Now add the sdfastBody to the array of others. */
   sdfastBodies.append(sdfastBody);
}

void SimmJoint::scale(const ScaleSet& aScaleSet)
{
	int i;
	Array<double> scaleFactors(1.0, 3);

	// Joint kinematics are scaled by the scale factors for the
	// parent body, so get those body's factors
	const string& parentName = getParentBody()->getName();

	for (i = 0; i < aScaleSet.getSize(); i++)
	{
		Scale *aScale = aScaleSet.get(i);
		if (aScale->getSegmentName() == parentName)
		{
			aScale->getScaleFactors(scaleFactors);
			break;
		}
	}

	// If all three factors are equal to 1.0, do nothing.
	if (EQUAL_WITHIN_ERROR(scaleFactors[0], 1.0) &&
		 EQUAL_WITHIN_ERROR(scaleFactors[1], 1.0) &&
		 EQUAL_WITHIN_ERROR(scaleFactors[2], 1.0))
		 return;

	// Scaling will happen, so invalidate the transforms.
	_transformsValid = false;

	/* This code assumes that if the DOF is a function with 2 points, then it
	 * acts as the independent gencoord, and should not be scaled. It
	 * also assumes that if the function has 3 or more points, it should be
	 * scaled.
	 */
   for (i = 0; i < _dofs.getSize(); i++)
   {
		if (_dofs[i]->getDofType() == SimmDof::Translational)
		{
			SimmTranslationDof* transDof = dynamic_cast<SimmTranslationDof*>(_dofs[i]);
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

void SimmJoint::writeSIMM(ofstream& out, int& aFunctionIndex) const
{
	int transDofIndex = 0, rotDofIndex = 0;
	string order;
	char* translationLabels[] = {"tx", "ty", "tz"};
	int* funcIndex = new int [_dofs.getSize()];

	int i;
	for (i = 0; i < _dofs.getSize(); i++)
		funcIndex[i] = -1;

	out << "beginjoint " << getName() << endl;
	out << "segments " << _parentBody->getName() << " " << _childBody->getName() << endl;
	for (i = 0; i < _dofs.getSize(); i++)
	{
		if (_dofs[i]->getDofType() == SimmDof::Translational)
		{
			SimmTranslationDof* td = dynamic_cast<SimmTranslationDof*>(_dofs[i]);
			SimmTranslationDof::AxisIndex axis = td->getAxisIndex();
			out << translationLabels[axis];
			if (td->getCoordinate() == NULL)
				out << " constant " << td->getValue() << endl;
			else
			{
				funcIndex[i] = aFunctionIndex++;
				out << " function f" << funcIndex[i] << "(" << td->getCoordinate()->getName() << ")" << endl;
			}
			if (transDofIndex++ == 0)
				order += " t";
		}
		else if (_dofs[i]->getDofType() == SimmDof::Rotational)
		{
			char buffer[5];
			SimmRotationDof* rd = dynamic_cast<SimmRotationDof*>(_dofs[i]);
			rotDofIndex++;
			out << "r" << rotDofIndex;
			if (rd->getCoordinate() == NULL)
				out << " constant " << rd->getValue() << endl;
			else
			{
				funcIndex[i] = aFunctionIndex++;
				out << " function f" << funcIndex[i] << "(" << rd->getCoordinate()->getName() << ")" << endl;
			}
			const double* axis = rd->getAxisPtr();
			out << "axis" << rotDofIndex << " " << axis[0] << " " << axis[1] << " " << axis[2] << endl;
			order += " r" + string(itoa(rotDofIndex, buffer, 10));
		}
	}
	out << "order" << order << endl;
	out << "endjoint" << endl << endl;

	for (i = 0; i < _dofs.getSize(); i++)
	{
		if (funcIndex[i] >= 0)
		{
			_dofs[i]->getFunction()->writeSIMM(out, funcIndex[i]);
		}
	}

	delete funcIndex;
}

void SimmJoint::peteTest()
{
	cout << "Joint: " << getName() << endl;
	cout << "   bodies: " << _bodies << endl;

	if (_dofs.getSize() == 0)
		cout << "no dofs" << endl;
	else
	{
		for (int i = 0; i < _dofs.getSize(); i++)
			_dofs[i]->peteTest();
	}

	calcTransforms();
	_forwaTransform.printMatrix();
}


