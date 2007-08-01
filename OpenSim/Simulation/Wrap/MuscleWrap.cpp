// MuscleWrap.cpp
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
#include "MuscleWrap.h"
#include <OpenSim/Simulation/Model/BodySet.h>
#include <OpenSim/Simulation/Model/AbstractDynamicsEngine.h>
#include <OpenSim/Simulation/Model/AbstractMuscle.h>
#include <OpenSim/Common/rdMath.h>

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
MuscleWrap::MuscleWrap() :
	Object(),
	_wrapObjectName(_wrapObjectNameProp.getValueStr()),
	_methodName(_methodNameProp.getValueStr()),
   _range(_rangeProp.getValueIntArray())
{
	setNull();
	setupProperties();
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
MuscleWrap::~MuscleWrap()
{
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aMuscleWrap MuscleWrap to be copied.
 */
MuscleWrap::MuscleWrap(const MuscleWrap& aMuscleWrap) :
	Object(aMuscleWrap),
	_wrapObjectName(_wrapObjectNameProp.getValueStr()),
	_methodName(_methodNameProp.getValueStr()),
   _range(_rangeProp.getValueIntArray())
{
	setNull();
	setupProperties();
	copyData(aMuscleWrap);
}

//_____________________________________________________________________________
/**
 * Copy this MuscleWrap and return a pointer to the copy.
 * The copy constructor for this class is used.
 *
 * @return Pointer to a copy of this MuscleWrap.
 */
Object* MuscleWrap::copy() const
{
	MuscleWrap *muscWrap = new MuscleWrap(*this);
	return(muscWrap);
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the data members of this MuscleWrap to their null values.
 */
void MuscleWrap::setNull()
{
	setType("MuscleWrap");

	_method = hybrid;

	resetPreviousWrap();
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void MuscleWrap::setupProperties()
{
	_wrapObjectNameProp.setName("wrap_object");
	_propertySet.append(&_wrapObjectNameProp);

	_methodNameProp.setName("method");
	_methodNameProp.setValue("Unassigned");
	_propertySet.append(&_methodNameProp);

	const int defaultRange[] = {-1, -1};
	_rangeProp.setName("range");
	_rangeProp.setValue(2, defaultRange);
	_rangeProp.setAllowableArraySize(2);
	_propertySet.append(&_rangeProp);
}

//_____________________________________________________________________________
/**
 * Perform some set up functions that happen after the
 * object has been deserialized or copied.
 *
 * @param aEngine dynamics engine containing this SimmBody.
 */
void MuscleWrap::setup(AbstractDynamicsEngine* aEngine, AbstractMuscle* aMuscle)
{
	int i;
	const BodySet* bodySet = aEngine->getBodySet();

	_muscle = aMuscle;

	for (i = 0; i < bodySet->getSize(); i++) {
		AbstractWrapObject* wo = bodySet->get(i)->getWrapObject(getWrapObjectName());
		if (wo) {
			_wrapObject = wo;
			_wrapPoints[0].setBody(*bodySet->get(i));
			_wrapPoints[0].setWrapObject(wo);
			_wrapPoints[1].setBody(*bodySet->get(i));
			_wrapPoints[1].setWrapObject(wo);
			break;
		}
	}

	// setup() must be called after setBody() because it requires
	// that _bodyName already be assigned.
	_wrapPoints[0].setup(aEngine->getModel(), aMuscle);
	_wrapPoints[1].setup(aEngine->getModel(), aMuscle);

	if (_methodName == "hybrid" || _methodName == "Hybrid" || _methodName == "HYBRID")
		_method = hybrid;
	else if (_methodName == "midpoint" || _methodName == "Midpoint" || _methodName == "MIDPOINT")
		_method = midpoint;
	else if (_methodName == "axial" || _methodName == "Axial" || _methodName == "AXIAL")
		_method = axial;
	else if (_methodName == "Unassigned") {  // method was not specified in wrap object definition; use default
		_method = hybrid;
		_methodName = "hybrid";
	} else {  // method was specified incorrectly in wrap object definition; throw an exception
		string errorMessage = "Error: wrapping method for wrap object " + getName() + " was either not specified, or specified incorrectly.";
		throw Exception(errorMessage);
	}
}

//_____________________________________________________________________________
/**
 * Copy data members from one MuscleWrap to another.
 *
 * @param aMuscleWrap MuscleWrap to be copied.
 */
void MuscleWrap::copyData(const MuscleWrap& aMuscleWrap)
{
	_wrapObjectName = aMuscleWrap._wrapObjectName;
	_methodName = aMuscleWrap._methodName;
	_method = aMuscleWrap._method;
	_range = aMuscleWrap._range;
	_wrapObject = aMuscleWrap._wrapObject;
	_previousWrap = aMuscleWrap._previousWrap;

	_wrapPoints[0] = aMuscleWrap._wrapPoints[0];
	_wrapPoints[1] = aMuscleWrap._wrapPoints[1];
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
MuscleWrap& MuscleWrap::operator=(const MuscleWrap& aMuscleWrap)
{
	// BASE CLASS
	Object::operator=(aMuscleWrap);

	return(*this);
}

MuscleWrapPoint& MuscleWrap::getWrapPoint(int aIndex)
{
	if (aIndex < 0 || aIndex > 1)
	{
		// TODO string errorMessage = "MuscleWrap::getWrapPoint(): invalid index (" + aIndex + ")";
		// throw Exception(errorMessage);
	}

	return _wrapPoints[aIndex];
}

void MuscleWrap::setStartPoint(int aIndex)
{
	if ((aIndex != _range[0]) && (aIndex == -1 || _range[1] == -1 || (aIndex >= 1 && aIndex <= _range[1])))
	{
		_range[0] = aIndex;
		_muscle->invalidatePath();
	}
}

void MuscleWrap::setEndPoint(int aIndex)
{
	if ((aIndex != _range[1]) && (aIndex == -1 || _range[0] == -1 || (aIndex >= _range[0] && aIndex <= _muscle->getAttachmentSet().getSize())))
	{
		_range[1] = aIndex;
		_muscle->invalidatePath();
	}
}

void MuscleWrap::resetPreviousWrap()
{
	_previousWrap.startPoint = -1;
	_previousWrap.endPoint = -1;

	_previousWrap.wrap_pts.setSize(0);
	_previousWrap.wrap_path_length = 0.0;

	int i;
	for (i = 0; i < 3; i++) {
		_previousWrap.r1[i] = rdMath::MINUS_INFINITY;
		_previousWrap.r2[i] = rdMath::MINUS_INFINITY;
		_previousWrap.sv[i] = rdMath::MINUS_INFINITY;
	}
}

void MuscleWrap::setPreviousWrap(const WrapResult& aWrapResult)
{
	_previousWrap = aWrapResult;
}

void MuscleWrap::setWrapObject(AbstractWrapObject& aWrapObject)
{
	_wrapObject = &aWrapObject;
	_wrapObjectName = aWrapObject.getName();
}

void MuscleWrap::setMethod(WrapMethod aMethod)
{
	if (aMethod == axial) {
		_method = axial;
		_methodName = "axial";
	} else if (aMethod == midpoint) {
		_method = midpoint;
		_methodName = "midpoint";
	} else if (aMethod == hybrid) {
		_method = hybrid;
		_methodName = "hybrid";
	}
}
